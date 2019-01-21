// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Arman Pazouki
// =============================================================================
//
// Implementation of methods specific to the parallel smooth-contact solver.
//
// These functions implement the basic time update for a multibody system using
// a penalty-based approach for including frictional contact. It is assumed that
// geometric contact information has been already computed and is available.
// The current algorithm is based on a semi-implicit Euler scheme and projection
// on the velocity manifold of the bilateral constraints.
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono_parallel/solver/ChIterativeSolverParallel.h"

#include <thrust/sort.h>

#if defined(CHRONO_OPENMP_ENABLED)
#include <thrust/system/omp/execution_policy.h>
#elif defined(CHRONO_TBB_ENABLED)
#include <thrust/system/tbb/execution_policy.h>
#endif

#if defined _WIN32
#include <cstdint>
#endif

using namespace chrono;

// -----------------------------------------------------------------------------
// Main worker function for calculating contact forces. Calculates the contact
// force and torque for the contact pair identified by 'index' and stores them
// in the 'extended' output arrays. The calculated force and torque vectors are
// therefore duplicated in the output arrays, once for each body involved in the
// contact (with opposite signs for the two bodies).
// -----------------------------------------------------------------------------
void function_CalcContactForces(
    int index,                                            // index of this contact pair
    ChSystemSMC::ContactForceModel contact_model,         // contact force model
    ChSystemSMC::AdhesionForceModel adhesion_model,       // contact force model
    ChSystemSMC::TangentialDisplacementModel displ_mode,  // type of tangential displacement history
    ChMaterialCompositionStrategy<real>* strategy,        // material composition strategy
    bool use_mat_props,                                   // flag specifying how coefficients are obtained
    real char_vel,                                        // characteristic velocity (Hooke)
    real min_slip_vel,                                    // threshold tangential velocity
    real min_roll_vel,                                    // threshold rolling velocity
    real min_spin_vel,                                    // threshold spinning velocity
    real dT,                                              // integration time step
    real* mass,                                           // body masses
    real3* pos,                                           // body positions
    quaternion* rot,                                      // body orientations
    real* vel,                                            // body linear and angular velocities
    real2* elastic_moduli,                                // Young's modulus (per body)
    real* cr,                                             // coefficient of restitution (per body)
    real4* smc_coeffs,                                    // stiffness and damping coefficients (per body)
    real* mu,                                             // coefficient of friction (per body)
    real* muRoll,                                         // coefficient of rolling friction (per body)
    real* muSpin,                                         // coefficient of spinning friction (per body)
    real* adhesion,                                       // constant force (per body)
    real* adhesionMultDMT,                                // Adhesion force multiplier (per body), in DMT model.
    real* adhesionSPerko,                                 // Cleanliness factor (per body), in Perko model.
    vec2* body_id,                                        // body IDs (per contact)
    vec2* shape_id,                                       // shape IDs (per contact)
    real3* pt1,                                           // point on shape 1 (per contact)
    real3* pt2,                                           // point on shape 2 (per contact)
    real3* normal,                                        // contact normal (per contact)
    real* depth,                                          // penetration depth (per contact)
    real* eff_radius,                                     // effective contact radius (per contact)
    vec3* shear_neigh,          // neighbor list of contacting bodies and shapes (max_shear per body)
    char* shear_touch,          // flag if contact in neighbor list is persistent (max_shear per body)
    real3* shear_disp,          // accumulated shear displacement for each neighbor (max_shear per body)
    real4* contact_coeff,       // stiffness and damping coefficients per contact pair, calculated at first contact
                                // (max_shear per body)
    real* contact_relvel_init,  // initial relative normal velocity manitude per contact pair, calculated at first
                                // contact (max_shear per body)
    int* ext_body_id,           // [output] body IDs (two per contact)
    real3* ext_body_force,      // [output] body force (two per contact)
    real3* ext_body_torque      // [output] body torque (two per contact)
) {
    // File properties and print lines for debugging. Delete during code clean-up
    real p1;             // relvel_n_mag (signed)
    real p2;             // relvel_t_mag (always positive)
    std::string p3;      // Tangential dispplacement method
    std::string p4;      // Method used to obtain k and g
    std::string p5;      // Contact force model
    std::string p6;      // Adhesion model
    real p7, p8, p9;     // Rotational velocity of body1
    real p10, p11, p12;  // Rotational velocity of body2
    real p13, p14, p15;  // Velocity of contact point1
    real p16, p17, p18;  // Velocity of contact point2
    real p19, p20, p21;  // Rolling friction torque
    real p22, p23, p24;  // Spinning friction torque

    bool print_data = GetPrint();
    static int runs = 0;
    uint prec = 10;
    uint w = 17;

    std::ofstream datao;

    if (runs == 0) {
        datao.open(GetChronoOutputPath() + "/chronodat.txt");
        datao << std::left << std::setw(w - 5) << "stp"
              << "\t" << std::left << std::setw(w - 5) << "body1"
              << "\t" << std::left << std::setw(w - 5) << "body2"
              << "\t" << std::left << std::setw(w - 5) << "newcontact"
              << "\t" << std::left << std::setw(w - 5) << "contact_id"
              << "\t" << std::left << std::setw(w) << "delta_n"
              << "\t" << std::left << std::setw(w) << "delta_t"
              << "\t" << std::left << std::setw(w) << "relvel_n_mag"
              << "\t" << std::left << std::setw(w) << "relvel_t_mag"
              << "\t" << std::left << std::setw(w) << "forceN_mag"
              << "\t" << std::left << std::setw(w) << "forceT_stiff"
              << "\t" << std::left << std::setw(w) << "forceT_damp"
              << "\t" << std::left << std::setw(w) << "force"
              << "\t" << std::left << std::setw(w) << "torque1"
              << "\t" << std::left << std::setw(w) << "torque2"
              << "\t" << std::left << std::setw(w) << "t_rf_x"
              << "\t" << std::left << std::setw(w) << "t_rf_y"
              << "\t" << std::left << std::setw(w) << "t_rf_z"
              << "\t" << std::left << std::setw(w) << "t_sf_x"
              << "\t" << std::left << std::setw(w) << "t_sf_y"
              << "\t" << std::left << std::setw(w) << "t_sf_z";
        if (!print_data) {
            datao.close();
            datao.clear();
        }
        runs++;
    } else if (print_data) {
        datao.open(GetChronoOutputPath() + "/chronodat.txt", std::ios::app);
        runs++;
    }

    // Identify the two bodies in contact.
    int body1 = body_id[index].x;
    int body2 = body_id[index].y;

    // If the two contact shapes are actually separated, set zero forces and torques.
    if (depth[index] >= 0) {
        ext_body_id[2 * index] = body1;
        ext_body_id[2 * index + 1] = body2;
        ext_body_force[2 * index] = real3(0);
        ext_body_force[2 * index + 1] = real3(0);
        ext_body_torque[2 * index] = real3(0);
        ext_body_torque[2 * index + 1] = real3(0);
        chrono::GetLog() << "\n"
                         << "WARNING: Exited function_CalcContactForce() without calculating forces.";
        return;
    }

    // Kinematic information
    // ---------------------

    // Express contact point locations in local frames
    //   s' = At * s = At * (rP - r)
    real3 pt1_loc = TransformParentToLocal(pos[body1], rot[body1], pt1[index]);
    real3 pt2_loc = TransformParentToLocal(pos[body2], rot[body2], pt2[index]);

    // Calculate velocities of the contact points (in global frame)
    //   vP = v + omg x s = v + A * (omg' x s')
    real3 v_body1 = real3(vel[body1 * 6 + 0], vel[body1 * 6 + 1], vel[body1 * 6 + 2]);
    real3 v_body2 = real3(vel[body2 * 6 + 0], vel[body2 * 6 + 1], vel[body2 * 6 + 2]);

    real3 o_body1 = real3(vel[body1 * 6 + 3], vel[body1 * 6 + 4], vel[body1 * 6 + 5]);
    real3 o_body2 = real3(vel[body2 * 6 + 3], vel[body2 * 6 + 4], vel[body2 * 6 + 5]);

    real3 vel1 = v_body1 + Rotate(Cross(o_body1, pt1_loc), rot[body1]);
    real3 vel2 = v_body2 + Rotate(Cross(o_body2, pt2_loc), rot[body2]);

    p7 = o_body1.x, p8 = o_body1.y, p9 = o_body1.z;
    p10 = o_body2.x, p11 = o_body2.y, p12 = o_body2.z;
    p13 = vel1.x, p14 = vel1.y, p15 = vel1.z;
    p16 = vel2.x, p17 = vel2.y, p18 = vel2.z;

    // Calculate relative velocity (in global frame)
    // Note that relvel_n_mag is a signed quantity, while relvel_t_mag is an
    // actual magnitude (always positive).
    real3 relvel = vel2 - vel1;
    real relvel_n_mag = Dot(relvel, normal[index]);
    real3 relvel_n = relvel_n_mag * normal[index];
    real3 relvel_t = relvel - relvel_n;
    real relvel_t_mag = Length(relvel_t);

    p1 = relvel_n_mag;
    p2 = relvel_t_mag;

    // Calculate composite material properties
    // ---------------------------------------
    real mu_eff = strategy->CombineFriction(mu[body1], mu[body2]);
    real muRoll_eff = strategy->CombineFriction(muRoll[body1], muRoll[body2]);
    real muSpin_eff = strategy->CombineFriction(muSpin[body1], muSpin[body2]);
    real adhesion_eff = strategy->CombineCohesion(adhesion[body1], adhesion[body2]);
    real adhesionMultDMT_eff = strategy->CombineAdhesionMultiplier(adhesionMultDMT[body1], adhesionMultDMT[body2]);
    real adhesionSPerko_eff = strategy->CombineAdhesionMultiplier(adhesionSPerko[body1], adhesionSPerko[body2]);

    // Contact force
    // -------------

    // All models use the following formulas for normal and tangential forces:
    //     Fn = kn * delta_n - gn * v_n
    //     Ft = kt * delta_t - gt * v_t
    // The stiffness and damping coefficients are obtained differently, based
    // on the force model and on how coefficients are specified.
    real kn;
    real kt;
    real gn;
    real gt;
    real relvel_init;

    real delta_n = -depth[index];
    real3 delta_t = real3(0);

    int i;
    int contact_id;
    int shear_body1;
    int shear_body2;
    int shear_shape1;
    int shear_shape2;
    bool newcontact = true;
    double eps = std::numeric_limits<double>::epsilon();

    if (displ_mode == ChSystemSMC::TangentialDisplacementModel::OneStep) {
        delta_t = relvel_t * dT;
        p3 = "TangentialDisplacementModel = OneStep";

        real E_eff, G_eff, cr_eff;
        real user_kn, user_kt, user_gn, user_gt;

        real m_eff = mass[body1] * mass[body2] / (mass[body1] + mass[body2]);

        if (use_mat_props) {
            real Y1 = elastic_moduli[body1].x;
            real Y2 = elastic_moduli[body2].x;
            real nu1 = elastic_moduli[body1].y;
            real nu2 = elastic_moduli[body2].y;
            real inv_E = (1 - nu1 * nu1) / Y1 + (1 - nu2 * nu2) / Y2;
            real inv_G = 2 * (2 - nu1) * (1 + nu1) / Y1 + 2 * (2 - nu2) * (1 + nu2) / Y2;

            E_eff = 1 / inv_E;
            G_eff = 1 / inv_G;
            cr_eff = strategy->CombineRestitution(cr[body1], cr[body2]);
        } else {
            user_kn = strategy->CombineStiffnessCoefficient(smc_coeffs[body1].x, smc_coeffs[body2].x);
            user_kt = strategy->CombineStiffnessCoefficient(smc_coeffs[body1].y, smc_coeffs[body2].y);
            user_gn = strategy->CombineDampingCoefficient(smc_coeffs[body1].z, smc_coeffs[body2].z);
            user_gt = strategy->CombineDampingCoefficient(smc_coeffs[body1].w, smc_coeffs[body2].w);
        }

        relvel_init = abs(relvel_n_mag);

        switch (contact_model) {
            case ChSystemSMC::ContactForceModel::Hooke:
                if (use_mat_props) {
                    real tmp_k = (16.0 / 15) * Sqrt(eff_radius[index]) * E_eff;
                    real v2 = relvel_init * relvel_init;
                    real loge = (cr_eff < eps) ? Log(eps) : Log(cr_eff);
                    loge = (cr_eff > 1 - eps) ? Log(1 - eps) : loge;
                    real tmp_g = 1 + Pow(CH_C_PI / loge, 2);
                    kn = tmp_k * Pow(m_eff * v2 / tmp_k, 1.0 / 5);
                    kt = kn;
                    gn = Sqrt(4 * m_eff * kn / tmp_g);
                    gt = gn;
                    p4 = "use_mat_props = TRUE";
                } else {
                    kn = user_kn;
                    kt = user_kt;
                    gn = m_eff * user_gn;
                    gt = m_eff * user_gt;
                    p4 = "use_mat_props = FALSE";
                }
                break;

            case ChSystemSMC::ContactForceModel::Hertz:
                if (use_mat_props) {
                    real sqrt_Rd = Sqrt(eff_radius[index]);
                    real Sn = 2 * E_eff * sqrt_Rd;
                    real St = 8 * G_eff * sqrt_Rd;
                    real loge = (cr_eff < eps) ? Log(eps) : Log(cr_eff);
                    real beta = loge / Sqrt(loge * loge + CH_C_PI * CH_C_PI);
                    kn = (2.0 / 3) * Sn;
                    kt = St;
                    gn = -2 * Sqrt(5.0 / 6) * beta * Sqrt(Sn * m_eff);
                    gt = -2 * Sqrt(5.0 / 6) * beta * Sqrt(St * m_eff);
                    p4 = "use_mat_props = TRUE";
                } else {
                    real tmp = eff_radius[index];
                    kn = tmp * user_kn;
                    kt = tmp * user_kt;
                    gn = tmp * m_eff * user_gn;
                    gt = tmp * m_eff * user_gt;
                    p4 = "use_mat_props = FALSE";
                }
                break;

            case ChSystemSMC::Flores:
                if (use_mat_props) {
                    real sqrt_R = Sqrt(eff_radius[index]);
                    double cr = (cr_eff < eps) ? eps : cr_eff;
                    cr = (cr_eff > 1 - eps) ? 1 - eps : cr;
                    kn = (4.0 / 3.0) * E_eff * sqrt_R;
                    kt = kn;
                    gn = 8.0 * (1.0 - cr) * kn / (5.0 * cr * relvel_init);
                    gt = gn;
                    p4 = "use_mat_props = TRUE";
                } else {
                    real tmp = eff_radius[index];
                    kn = tmp * user_kn;
                    kt = tmp * user_kt;
                    gn = tmp * m_eff * user_gn;
                    gt = tmp * m_eff * user_gt;
                    p4 = "use_mat_props = FALSE";
                }
                break;

            case ChSystemSMC::ContactForceModel::PlainCoulomb:
                if (use_mat_props) {
                    real Sn = 2 * E_eff;
                    real St = 8 * G_eff;
                    real loge = (cr_eff < eps) ? Log(eps) : Log(cr_eff);
                    real beta = loge / Sqrt(loge * loge + CH_C_PI * CH_C_PI);
                    kn = (2.0 / 3) * Sn;
                    kt = 0;
                    gn = -2 * Sqrt(5.0 / 6) * beta * Sqrt(Sn * m_eff);
                    gt = 0;
                    p4 = "use_mat_props = TRUE";
                } else {
                    kn = user_kn;
                    kt = 0;
                    gn = user_gn;
                    gt = 0;
                    p4 = "use_mat_props = FALSE";
                }
                break;
        }
    } else if (displ_mode == ChSystemSMC::TangentialDisplacementModel::MultiStep) {
        delta_t = relvel_t * dT;
        p3 = "TangentialDisplacementModel = MultiStep";

        // Contact history information should be stored on the body with
        // the smaller shape in contact or the body with larger index.
        // Currently, it is assumed that the smaller shape is on the body
        // with larger ID.
        // We call this body shear_body1.
        int shape1 = shape_id[index].x;
        int shape2 = shape_id[index].y;

        shear_body1 = (int)Max(body1, body2);
        shear_body2 = (int)Min(body1, body2);
        shear_shape1 = (int)Max(shape1, shape2);
        shear_shape2 = (int)Min(shape1, shape2);

        // Check if contact history already exists.
        // If not, initialize new contact history.
        for (i = 0; i < max_shear; i++) {
            int ctIdUnrolled = max_shear * shear_body1 + i;
            if (shear_neigh[ctIdUnrolled].x == shear_body2 && shear_neigh[ctIdUnrolled].y == shear_shape1 &&
                shear_neigh[ctIdUnrolled].z == shear_shape2) {
                contact_id = i;
                newcontact = false;
                break;
            }
        }
        if (newcontact == true) {
            for (i = 0; i < max_shear; i++) {
                int ctIdUnrolled = max_shear * shear_body1 + i;
                if (shear_neigh[ctIdUnrolled].x == -1) {
                    contact_id = i;
                    shear_neigh[ctIdUnrolled].x = shear_body2;
                    shear_neigh[ctIdUnrolled].y = shear_shape1;
                    shear_neigh[ctIdUnrolled].z = shear_shape2;
                    shear_disp[ctIdUnrolled].x = 0;
                    shear_disp[ctIdUnrolled].y = 0;
                    shear_disp[ctIdUnrolled].z = 0;
                    contact_relvel_init[ctIdUnrolled] = abs(relvel_n_mag);

                    real E_eff, G_eff, cr_eff;
                    real user_kn, user_kt, user_gn, user_gt;

                    real m_eff = mass[body1] * mass[body2] / (mass[body1] + mass[body2]);

                    if (use_mat_props) {
                        real Y1 = elastic_moduli[body1].x;
                        real Y2 = elastic_moduli[body2].x;
                        real nu1 = elastic_moduli[body1].y;
                        real nu2 = elastic_moduli[body2].y;
                        real inv_E = (1 - nu1 * nu1) / Y1 + (1 - nu2 * nu2) / Y2;
                        real inv_G = 2 * (2 - nu1) * (1 + nu1) / Y1 + 2 * (2 - nu2) * (1 + nu2) / Y2;

                        E_eff = 1 / inv_E;
                        G_eff = 1 / inv_G;
                        cr_eff = strategy->CombineRestitution(cr[body1], cr[body2]);
                    } else {
                        user_kn = strategy->CombineStiffnessCoefficient(smc_coeffs[body1].x, smc_coeffs[body2].x);
                        user_kt = strategy->CombineStiffnessCoefficient(smc_coeffs[body1].y, smc_coeffs[body2].y);
                        user_gn = strategy->CombineDampingCoefficient(smc_coeffs[body1].z, smc_coeffs[body2].z);
                        user_gt = strategy->CombineDampingCoefficient(smc_coeffs[body1].w, smc_coeffs[body2].w);
                    }

                    switch (contact_model) {
                        case ChSystemSMC::ContactForceModel::Hooke:
                            if (use_mat_props) {
                                real tmp_k = (16.0 / 15) * Sqrt(eff_radius[index]) * E_eff;
                                real v2 = contact_relvel_init[ctIdUnrolled] * contact_relvel_init[ctIdUnrolled];
                                real loge = (cr_eff < eps) ? Log(eps) : Log(cr_eff);
                                loge = (cr_eff > 1 - eps) ? Log(1 - eps) : loge;
                                real tmp_g = 1 + Pow(CH_C_PI / loge, 2);
                                contact_coeff[ctIdUnrolled].x = tmp_k * Pow(m_eff * v2 / tmp_k, 1.0 / 5);
                                contact_coeff[ctIdUnrolled].y = contact_coeff[ctIdUnrolled].x;
                                contact_coeff[ctIdUnrolled].z = Sqrt(4 * m_eff * contact_coeff[ctIdUnrolled].x / tmp_g);
                                contact_coeff[ctIdUnrolled].w = contact_coeff[ctIdUnrolled].z;
                                p4 = "use_mat_props = TRUE";
                            } else {
                                contact_coeff[ctIdUnrolled].x = user_kn;
                                contact_coeff[ctIdUnrolled].y = user_kt;
                                contact_coeff[ctIdUnrolled].z = m_eff * user_gn;
                                contact_coeff[ctIdUnrolled].w = m_eff * user_gt;
                                p4 = "use_mat_props = FALSE";
                            }
                            break;

                        case ChSystemSMC::ContactForceModel::Hertz:
                            if (use_mat_props) {
                                real sqrt_Rd = Sqrt(eff_radius[index]);
                                real Sn = 2 * E_eff * sqrt_Rd;
                                real St = 8 * G_eff * sqrt_Rd;
                                real loge = (cr_eff < eps) ? Log(eps) : Log(cr_eff);
                                real beta = loge / Sqrt(loge * loge + CH_C_PI * CH_C_PI);
                                contact_coeff[ctIdUnrolled].x = (2.0 / 3) * Sn;
                                contact_coeff[ctIdUnrolled].y = St;
                                contact_coeff[ctIdUnrolled].z = -2 * Sqrt(5.0 / 6) * beta * Sqrt(Sn * m_eff);
                                contact_coeff[ctIdUnrolled].w = -2 * Sqrt(5.0 / 6) * beta * Sqrt(St * m_eff);
                                p4 = "use_mat_props = TRUE";
                            } else {
                                real tmp = eff_radius[index];
                                contact_coeff[ctIdUnrolled].x = tmp * user_kn;
                                contact_coeff[ctIdUnrolled].y = tmp * user_kt;
                                contact_coeff[ctIdUnrolled].z = tmp * m_eff * user_gn;
                                contact_coeff[ctIdUnrolled].w = tmp * m_eff * user_gt;
                                p4 = "use_mat_props = FALSE";
                            }
                            break;

                        case ChSystemSMC::Flores:
                            if (use_mat_props) {
                                real sqrt_R = Sqrt(eff_radius[index]);
                                double cr = (cr_eff < eps) ? eps : cr_eff;
                                cr = (cr_eff > 1 - eps) ? 1 - eps : cr;
                                contact_coeff[ctIdUnrolled].x = (4.0 / 3.0) * E_eff * sqrt_R;
                                contact_coeff[ctIdUnrolled].y = contact_coeff[ctIdUnrolled].x;
                                contact_coeff[ctIdUnrolled].z = 8.0 * (1.0 - cr) * contact_coeff[ctIdUnrolled].x /
                                                                (5.0 * cr * contact_relvel_init[ctIdUnrolled]);
                                contact_coeff[ctIdUnrolled].w = contact_coeff[ctIdUnrolled].z;
                                p4 = "use_mat_props = TRUE";
                            } else {
                                real tmp = eff_radius[index];
                                contact_coeff[ctIdUnrolled].x = tmp * user_kn;
                                contact_coeff[ctIdUnrolled].y = tmp * user_kt;
                                contact_coeff[ctIdUnrolled].z = tmp * m_eff * user_gn;
                                contact_coeff[ctIdUnrolled].w = tmp * m_eff * user_gt;
                                p4 = "use_mat_props = FALSE";
                            }
                            break;

                        case ChSystemSMC::ContactForceModel::PlainCoulomb:
                            if (use_mat_props) {
                                real Sn = 2 * E_eff;
                                real St = 8 * G_eff;
                                real loge = (cr_eff < eps) ? Log(eps) : Log(cr_eff);
                                real beta = loge / Sqrt(loge * loge + CH_C_PI * CH_C_PI);
                                contact_coeff[ctIdUnrolled].x = (2.0 / 3) * Sn;
                                contact_coeff[ctIdUnrolled].y = 0;
                                contact_coeff[ctIdUnrolled].z = -2 * Sqrt(5.0 / 6) * beta * Sqrt(Sn * m_eff);
                                contact_coeff[ctIdUnrolled].w = 0;
                                p4 = "use_mat_props = TRUE";
                            } else {
                                contact_coeff[ctIdUnrolled].x = user_kn;
                                contact_coeff[ctIdUnrolled].y = 0;
                                contact_coeff[ctIdUnrolled].z = user_gn;
                                contact_coeff[ctIdUnrolled].w = 0;
                                p4 = "use_mat_props = FALSE";
                            }
                            break;
                    }
                    break;
                }
            }
        }

        // Record that these two bodies are really in contact at this time.
        int ctSaveId = max_shear * shear_body1 + contact_id;
        shear_touch[ctSaveId] = true;

        // Increment stored contact history tangential (shear) displacement vector
        // and project it onto the <current> contact plane.

        if (shear_body1 == body1) {
            shear_disp[ctSaveId] += delta_t;
            shear_disp[ctSaveId] -= Dot(shear_disp[ctSaveId], normal[index]) * normal[index];
            delta_t = shear_disp[ctSaveId];
        } else {
            shear_disp[ctSaveId] -= delta_t;
            shear_disp[ctSaveId] -= Dot(shear_disp[ctSaveId], normal[index]) * normal[index];
            delta_t = -shear_disp[ctSaveId];
        }

        // Load kn, kt, gn, gt, and initial relative impact velocity from the contact history
        kn = contact_coeff[ctSaveId].x;
        kt = contact_coeff[ctSaveId].y;
        gn = contact_coeff[ctSaveId].z;
        gt = contact_coeff[ctSaveId].w;
        relvel_init = contact_relvel_init[ctSaveId];
    }

    // Calculate the the normal and tangential contact forces.
    // The normal force is a magnitude, and it will be applied along the contact
    // normal direction (negative relative to body1 & positive relative to body2).
    // The tangential force is a vector with two parts: one depends on the stored
    // contact history tangential (or shear) displacement vector delta_t, and the
    // other depends on the current relative velocity vector (for viscous damping).
    real forceN_mag;
    real3 forceT_stiff, forceT_damp;

    switch (contact_model) {
        case ChSystemSMC::Hooke:
            forceN_mag = kn * delta_n - gn * relvel_n_mag;
            forceT_stiff = kt * delta_t;
            forceT_damp = gt * relvel_t;
            p5 = "ForceModel = Hooke";
            break;

        case ChSystemSMC::Hertz:
            forceN_mag = kn * Pow(delta_n, 1.5) - gn * Pow(delta_n, 0.25) * relvel_n_mag;
            forceT_stiff = kt * Pow(delta_n, 0.5) * delta_t;
            forceT_damp = gt * Pow(delta_n, 0.25) * relvel_t;
            p5 = "ForceModel = Hertz";
            break;

        case ChSystemSMC::Flores:
            forceN_mag = kn * Pow(delta_n, 1.5) - gn * Pow(delta_n, 1.5) * relvel_n_mag;
            forceT_stiff = kt * Pow(delta_n, 0.5) * delta_t;
            forceT_damp = gt * Pow(delta_n, 0.25) * relvel_t;
            p5 = "ForceModel = Flores";
            break;

        case ChSystemSMC::PlainCoulomb: {
            forceN_mag = kn * Pow(delta_n, 1.5) - gn * Pow(delta_n, 0.25) * relvel_n_mag;
            // if (forceN_mag < 0)
            //     forceN_mag = 0;
            real forceT_mag = mu_eff * Tanh(5.0 * relvel_t_mag) * forceN_mag;
            p5 = "ForceModel = PlainCoulomb";

            /*
            // Include adhesion force.
            switch (adhesion_model) {
                case ChSystemSMC::AdhesionForceModel::Constant:
                    forceN_mag -= adhesion_eff;
                    p6 = "AdhesionModel = Constant";
                    break;
                case ChSystemSMC::AdhesionForceModel::DMT:
                    forceN_mag -= adhesionMultDMT_eff * Sqrt(eff_radius[index]);
                    p6 = "AdhesionModel = DMT";
                    break;
                case ChSystemSMC::AdhesionForceModel::Perko:
                    forceN_mag -= adhesionSPerko_eff * adhesionSPerko_eff * 3.6E-2 * eff_radius[index];
                    p6 = "AdhesionModel = Perko";
                    break;
            }
            */

            real3 force = forceN_mag * normal[index];
            if (relvel_t_mag >= (real)1e-4)
                force -= (forceT_mag / relvel_t_mag) * relvel_t;

            real3 torque1_loc = Cross(pt1_loc, RotateT(force, rot[body1]));
            real3 torque2_loc = Cross(pt2_loc, RotateT(force, rot[body2]));

            // Calculate rolling friction torque as M_roll = �_r * R * (F_N x v_rot) / |v_rot|
            real3 v_rot = Rotate(Cross(o_body1, pt1_loc), rot[body1]) + Rotate(Cross(o_body2, pt2_loc), rot[body2]);
            if (Length(v_rot) > min_roll_vel) {
                real3 torque_buff =
                    muRoll_eff * eff_radius[index] * Cross(forceN_mag * normal[index], v_rot) / Length(v_rot);
                torque1_loc += torque_buff;
                torque2_loc += torque_buff;

                p19 = torque_buff.x, p20 = torque_buff.y, p21 = torque_buff.z;
            }

            // Calculate twisting friction torque as M_twist = -�_t * r_c * ((w_n - w_p) . F_n / |w_n - w_p|) * n
            // r_c is the radius of the circle resulting from the intersecting body surfaces
            if (Length(o_body2 - o_body1) > min_spin_vel) {
                double R1 = Length(pt1_loc), R2 = Length(pt2_loc);
                double R_center = (R1 * R1 - R2 * R2) / (2 * (R1 + R2 - delta_n)) + 0.5 * (R1 + R2 - delta_n);
                double r_c = sqrt(pow(R1, 2) - pow(R_center, 2));
                real3 torque_buff = muSpin_eff * r_c *
                                    (Dot(o_body2 - o_body1, forceN_mag * normal[index]) / Length(o_body1 - o_body2)) *
                                    normal[index];
                torque1_loc -= torque_buff;
                torque2_loc -= torque_buff;

                p22 = torque_buff.x, p23 = torque_buff.y, p24 = torque_buff.z;
            }

            // Include adhesion force.
            switch (adhesion_model) {
                case ChSystemSMC::AdhesionForceModel::Constant:
                    force -= adhesion_eff * normal[index];
                    p6 = "AdhesionModel = Constant";
                    break;
                case ChSystemSMC::AdhesionForceModel::DMT:
                    force -= adhesionMultDMT_eff * Sqrt(eff_radius[index] * normal[index]);
                    p6 = "AdhesionModel = DMT";
                    break;
                case ChSystemSMC::AdhesionForceModel::Perko:
                    force -= adhesionSPerko_eff * adhesionSPerko_eff * 3.6E-2 * eff_radius[index] * normal[index];
                    p6 = "AdhesionModel = Perko";
                    break;
            }

            ext_body_id[2 * index] = body1;
            ext_body_id[2 * index + 1] = body2;
            ext_body_force[2 * index] = -force;
            ext_body_force[2 * index + 1] = force;
            ext_body_torque[2 * index] = -torque1_loc;
            ext_body_torque[2 * index + 1] = torque2_loc;

            // Print one-time collision information to userlog
            if (runs == 1 && print_data) {
                chrono::GetLog() << "\n"
                                 << body1 << "_" << body2 << "_kn " << kn << "\n"
                                 << body1 << "_" << body2 << "_kt " << kt << "\n"
                                 << body1 << "_" << body2 << "_gn " << gn << "\n"
                                 << body1 << "_" << body2 << "_gt " << gt;
            }

            // Print collision metadata to tab dilineated chronodat.txt file
            if (print_data) {
                datao << "\n"
                      << std::left << std::setw(w - 5) << runs << "\t" << std::left << std::setw(w - 5) << body1 << "\t"
                      << std::left << std::setw(w - 5) << body2 << "\t" << std::left << std::setw(w - 5) << newcontact
                      << "\t" << std::left << std::setw(w - 5) << contact_id << "\t" << std::left << std::setw(w)
                      << std::setprecision(prec) << delta_n << "\t" << std::left << std::setw(w)
                      << std::setprecision(prec) << Length(delta_t) << "\t" << std::left << std::setw(w)
                      << std::setprecision(prec) << p1 << "\t" << std::left << std::setw(w) << std::setprecision(prec)
                      << p2 << "\t" << std::left << std::setw(w) << std::setprecision(prec) << forceN_mag << "\t"
                      << std::left << std::setw(w) << std::setprecision(prec) << forceT_mag << "\t" << std::left
                      << std::setw(w) << std::setprecision(prec) << 0 << "\t" << std::left << std::setw(w)
                      << std::setprecision(prec) << Length(force) << "\t" << std::left << std::setw(w)
                      << std::setprecision(prec) << Length(torque1_loc) << "\t" << std::left << std::setw(w)
                      << std::setprecision(prec) << Length(torque2_loc) << "\t" << std::left << std::setw(w)
                      << std::setprecision(prec) << p19 << "\t" << std::left << std::setw(w) << std::setprecision(prec)
                      << p20 << "\t" << std::left << std::setw(w) << std::setprecision(prec) << p21 << "\t" << std::left
                      << std::setw(w) << std::setprecision(prec) << p22 << "\t" << std::left << std::setw(w)
                      << std::setprecision(prec) << p23 << "\t" << std::left << std::setw(w) << std::setprecision(prec)
                      << p24;
                datao.close();
            }

            return;
        }
    }

    /*
    // If the resulting normal force is negative, then the two shapes are
    // moving away from each other so fast that no contact force is generated.
    if (forceN_mag < 0) {
        forceN_mag = 0;
        forceT_stiff.x = 0;
        forceT_stiff.y = 0;
        forceT_stiff.z = 0;
        forceT_damp.x = 0;
        forceT_damp.y = 0;
        forceT_damp.z = 0;
    }
*/

    /*
        // Include adhesion force.
        switch (adhesion_model) {
            case ChSystemSMC::AdhesionForceModel::Constant:
                // (This is a very simple model, which can perhaps be improved later.)
                forceN_mag -= adhesion_eff;
                p6 = "AdhesionModel = Constant";
                break;
            case ChSystemSMC::AdhesionForceModel::DMT:
                // Derjaguin, Muller and Toporov (DMT) adhesion force,
                forceN_mag -= adhesionMultDMT_eff * Sqrt(eff_radius[index]);
                p6 = "AdhesionModel = DMT";
                break;
            case ChSystemSMC::AdhesionForceModel::Perko:
                forceN_mag -= adhesionSPerko_eff * adhesionSPerko_eff * 3.6E-2 * eff_radius[index];
                p6 = "AdhesionModel = Perko";
                break;
        }
    */

    // Apply Coulomb friction law.
    // We must enforce force_T_mag <= mu_eff * |forceN_mag|.
    // If force_T_mag > mu_eff * |forceN_mag| and there is shear displacement
    // due to contact history, then the shear displacement is scaled so that
    // the tangential force will be correct if force_T_mag subsequently drops
    // below the Coulomb limit.  Also, if there is sliding, then there is no
    // viscous damping in the tangential direction (to keep the Coulomb limit
    // strict, and independent of velocity).
    //  real forceT_mag = Length(forceT_stiff + forceT_damp);  // This seems correct
    real forceT_stiff_mag = Length(forceT_stiff);  // This is what LAMMPS/LIGGGHTS does
    real delta_t_mag = Length(delta_t);
    real forceT_slide = mu_eff * Abs(forceN_mag);
    if (forceT_stiff_mag > forceT_slide) {
        if (delta_t_mag > eps) {
            real ratio = forceT_slide / forceT_stiff_mag;
            forceT_stiff *= ratio;
            if (displ_mode == ChSystemSMC::TangentialDisplacementModel::MultiStep) {
                if (shear_body1 == body1) {
                    shear_disp[max_shear * shear_body1 + contact_id] = forceT_stiff / kt;
                } else {
                    shear_disp[max_shear * shear_body1 + contact_id] = -forceT_stiff / kt;
                }
            }
        } else {
            forceT_stiff.x = 0.0;
            forceT_stiff.y = 0.0;
            forceT_stiff.z = 0.0;
        }
        forceT_damp.x = 0.0;
        forceT_damp.y = 0.0;
        forceT_damp.z = 0.0;
    }

    // Accumulate normal and tangential forces
    real3 force = forceN_mag * normal[index];
    force -= forceT_stiff;
    force -= forceT_damp;

    // Body forces (in global frame) & torques (in local frame)
    // --------------------------------------------------------

    // Convert force into the local body frames and calculate induced torques
    //    n' = s' x F' = s' x (A*F)
    real3 torque1_loc = Cross(pt1_loc, RotateT(force, rot[body1]));
    real3 torque2_loc = Cross(pt2_loc, RotateT(force, rot[body2]));

    // Calculate rolling friction torque as M_roll = �_r * R * (F_N x v_rot) / |v_rot|
    real3 v_rot = Rotate(Cross(o_body1, pt1_loc), rot[body1]) + Rotate(Cross(o_body2, pt2_loc), rot[body2]);
    if (Length(v_rot) > min_roll_vel) {
        real3 torque_buff = muRoll_eff * eff_radius[index] * Cross(forceN_mag * normal[index], v_rot) / Length(v_rot);
        torque1_loc += torque_buff;
        torque2_loc += torque_buff;

        p19 = torque_buff.x, p20 = torque_buff.y, p21 = torque_buff.z;
    }

    // Calculate spinning friction torque as M_spin = -�_t * r_c * ((w_n - w_p) . F_n / |w_n - w_p|) * n
    // r_c is the radius of the circle resulting from the intersecting body surfaces
    if (Length(o_body2 - o_body1) > min_spin_vel) {
        double R1 = Length(pt1_loc), R2 = Length(pt2_loc);
        double R_center = (R1 * R1 - R2 * R2) / (2 * (R1 + R2 - delta_n)) + 0.5 * (R1 + R2 - delta_n);
        double r_c = sqrt(pow(R1, 2) - pow(R_center, 2));
        real3 torque_buff = muSpin_eff * r_c *
                            (Dot(o_body2 - o_body1, forceN_mag * normal[index]) / Length(o_body1 - o_body2)) *
                            normal[index];
        torque1_loc -= torque_buff;
        torque2_loc -= torque_buff;

        p22 = torque_buff.x, p23 = torque_buff.y, p24 = torque_buff.z;
    }

    switch (adhesion_model) {
        case ChSystemSMC::AdhesionForceModel::Constant:
            // (This is a very simple model, which can perhaps be improved later.)
            force -= adhesion_eff * normal[index];
            p6 = "AdhesionModel = Constant";
            break;
        case ChSystemSMC::AdhesionForceModel::DMT:
            // Derjaguin, Muller and Toporov (DMT) adhesion force,
            force -= adhesionMultDMT_eff * Sqrt(eff_radius[index] * normal[index]);
            p6 = "AdhesionModel = DMT";
            break;
        case ChSystemSMC::AdhesionForceModel::Perko:
            force -= adhesionSPerko_eff * adhesionSPerko_eff * 3.6E-2 * eff_radius[index] * normal[index];
            p6 = "AdhesionModel = Perko";
            break;
    }

    // Store body forces and torques, duplicated for the two bodies.
    ext_body_id[2 * index] = body1;
    ext_body_id[2 * index + 1] = body2;
    ext_body_force[2 * index] = -force;
    ext_body_force[2 * index + 1] = force;
    ext_body_torque[2 * index] = -torque1_loc;
    ext_body_torque[2 * index + 1] = torque2_loc;

    // Print one-time collision information to userlog
    if (runs == 1 && print_data) {
        chrono::GetLog() << "\n"
                         << body1 << "_" << body2 << "_kn " << kn << "\n"
                         << body1 << "_" << body2 << "_kt " << kt << "\n"
                         << body1 << "_" << body2 << "_gn " << gn << "\n"
                         << body1 << "_" << body2 << "_gt " << gt;
    }

    // Print collision metadata to tab dilineated chronodat.txt file
    if (print_data) {
        datao << "\n"
              << std::left << std::setw(w - 5) << runs << "\t" << std::left << std::setw(w - 5) << body1 << "\t"
              << std::left << std::setw(w - 5) << body2 << "\t" << std::left << std::setw(w - 5) << newcontact << "\t"
              << std::left << std::setw(w - 5) << contact_id << "\t" << std::left << std::setw(w)
              << std::setprecision(prec) << delta_n << "\t" << std::left << std::setw(w) << std::setprecision(prec)
              << Length(delta_t) << "\t" << std::left << std::setw(w) << std::setprecision(prec) << p1 << "\t"
              << std::left << std::setw(w) << std::setprecision(prec) << p2 << "\t" << std::left << std::setw(w)
              << std::setprecision(prec) << forceN_mag << "\t" << std::left << std::setw(w) << std::setprecision(prec)
              << Length(forceT_stiff) << "\t" << std::left << std::setw(w) << std::setprecision(prec)
              << Length(forceT_damp) << "\t" << std::left << std::setw(w) << std::setprecision(prec) << Length(force)
              << "\t" << std::left << std::setw(w) << std::setprecision(prec) << Length(torque1_loc) << "\t"
              << std::left << std::setw(w) << std::setprecision(prec) << Length(torque2_loc) << "\t" << std::left
              << std::setw(w) << std::setprecision(prec) << p19 << "\t" << std::left << std::setw(w)
              << std::setprecision(prec) << p20 << "\t" << std::left << std::setw(w) << std::setprecision(prec) << p21
              << "\t" << std::left << std::setw(w) << std::setprecision(prec) << p22 << "\t" << std::left
              << std::setw(w) << std::setprecision(prec) << p23 << "\t" << std::left << std::setw(w)
              << std::setprecision(prec) << p24;
        datao.close();
    }
}

// -----------------------------------------------------------------------------
// Calculate contact forces and torques for all contact pairs.
// -----------------------------------------------------------------------------

void ChIterativeSolverParallelSMC::host_CalcContactForces(custom_vector<int>& ext_body_id,
                                                          custom_vector<real3>& ext_body_force,
                                                          custom_vector<real3>& ext_body_torque,
                                                          custom_vector<vec2>& shape_pairs,
                                                          custom_vector<char>& shear_touch) {
#pragma omp parallel for
    for (int index = 0; index < (signed)data_manager->num_rigid_contacts; index++) {
        function_CalcContactForces(
            index, data_manager->settings.solver.contact_force_model,
            data_manager->settings.solver.adhesion_force_model, data_manager->settings.solver.tangential_displ_mode,
            data_manager->composition_strategy.get(), data_manager->settings.solver.use_material_properties,
            data_manager->settings.solver.characteristic_vel, data_manager->settings.solver.min_slip_vel,
            data_manager->settings.solver.min_roll_vel, data_manager->settings.solver.min_spin_vel,
            data_manager->settings.step_size, data_manager->host_data.mass_rigid.data(),
            data_manager->host_data.pos_rigid.data(), data_manager->host_data.rot_rigid.data(),
            data_manager->host_data.v.data(), data_manager->host_data.elastic_moduli.data(),
            data_manager->host_data.cr.data(), data_manager->host_data.smc_coeffs.data(),
            data_manager->host_data.mu.data(), data_manager->host_data.muRoll.data(),
            data_manager->host_data.muSpin.data(), data_manager->host_data.cohesion_data.data(),
            data_manager->host_data.adhesionMultDMT_data.data(), data_manager->host_data.adhesionSPerko_data.data(),
            data_manager->host_data.bids_rigid_rigid.data(), shape_pairs.data(),
            data_manager->host_data.cpta_rigid_rigid.data(), data_manager->host_data.cptb_rigid_rigid.data(),
            data_manager->host_data.norm_rigid_rigid.data(), data_manager->host_data.dpth_rigid_rigid.data(),
            data_manager->host_data.erad_rigid_rigid.data(), data_manager->host_data.shear_neigh.data(),
            shear_touch.data(), data_manager->host_data.shear_disp.data(), data_manager->host_data.contact_coeff.data(),
            data_manager->host_data.contact_relvel_init.data(), ext_body_id.data(), ext_body_force.data(),
            ext_body_torque.data());
    }
}

// -----------------------------------------------------------------------------
// Include contact impulses (linear and rotational) for all bodies that are
// involved in at least one contact. For each such body, the corresponding
// entries in the arrays 'ct_body_force' and 'ct_body_torque' contain the
// cummulative force and torque, respectively, over all contacts involving that
// body.
// -----------------------------------------------------------------------------
void ChIterativeSolverParallelSMC::host_AddContactForces(uint ct_body_count, const custom_vector<int>& ct_body_id) {
    const custom_vector<real3>& ct_body_force = data_manager->host_data.ct_body_force;
    const custom_vector<real3>& ct_body_torque = data_manager->host_data.ct_body_torque;

#pragma omp parallel for
    for (int index = 0; index < (signed)ct_body_count; index++) {
        real3 contact_force = data_manager->settings.step_size * ct_body_force[index];
        real3 contact_torque = data_manager->settings.step_size * ct_body_torque[index];
        data_manager->host_data.hf[ct_body_id[index] * 6 + 0] += contact_force.x;
        data_manager->host_data.hf[ct_body_id[index] * 6 + 1] += contact_force.y;
        data_manager->host_data.hf[ct_body_id[index] * 6 + 2] += contact_force.z;
        data_manager->host_data.hf[ct_body_id[index] * 6 + 3] += contact_torque.x;
        data_manager->host_data.hf[ct_body_id[index] * 6 + 4] += contact_torque.y;
        data_manager->host_data.hf[ct_body_id[index] * 6 + 5] += contact_torque.z;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIterativeSolverParallelSMC::host_SetContactForcesMap(uint ct_body_count, const custom_vector<int>& ct_body_id) {
    custom_vector<int>& ct_body_map = data_manager->host_data.ct_body_map;

#pragma omp parallel for
    for (int index = 0; index < (signed)ct_body_count; index++) {
        ct_body_map[ct_body_id[index]] = index;
    }
}

// Binary operation for adding two-object tuples
struct sum_tuples {
    thrust::tuple<real3, real3> operator()(const thrust::tuple<real3, real3>& a,
                                           const thrust::tuple<real3, real3>& b) const {
        return thrust::tuple<real3, real3>(thrust::get<0>(a) + thrust::get<0>(b),
                                           thrust::get<1>(a) + thrust::get<1>(b));
    }
};

// -----------------------------------------------------------------------------
// Process contact information reported by the narrowphase collision detection,
// generate contact forces, and update the (linear and rotational) impulses for
// all bodies involved in at least one contact.
// -----------------------------------------------------------------------------
void ChIterativeSolverParallelSMC::ProcessContacts() {
    // 1. Calculate contact forces and torques - per contact basis
    //    For each pair of contact shapes that overlap, we calculate and store the
    //    IDs of the two corresponding bodies and the resulting contact forces and
    //    torques on the two bodies.
    custom_vector<int> ext_body_id(2 * data_manager->num_rigid_contacts);
    custom_vector<real3> ext_body_force(2 * data_manager->num_rigid_contacts);
    custom_vector<real3> ext_body_torque(2 * data_manager->num_rigid_contacts);
    custom_vector<vec2> shape_pairs;
    custom_vector<char> shear_touch;

    if (data_manager->settings.solver.tangential_displ_mode == ChSystemSMC::TangentialDisplacementModel::MultiStep) {
        shape_pairs.resize(data_manager->num_rigid_contacts);
        shear_touch.resize(max_shear * data_manager->num_rigid_bodies);
        Thrust_Fill(shear_touch, false);
#pragma omp parallel for
        for (int i = 0; i < (signed)data_manager->num_rigid_contacts; i++) {
            vec2 pair = I2(int(data_manager->host_data.contact_pairs[i] >> 32),
                           int(data_manager->host_data.contact_pairs[i] & 0xffffffff));
            shape_pairs[i] = pair;
        }
    }

    host_CalcContactForces(ext_body_id, ext_body_force, ext_body_torque, shape_pairs, shear_touch);

    if (data_manager->settings.solver.tangential_displ_mode == ChSystemSMC::TangentialDisplacementModel::MultiStep) {
#pragma omp parallel for
        for (int index = 0; index < (signed)data_manager->num_rigid_bodies; index++) {
            for (int i = 0; i < max_shear; i++) {
                if (shear_touch[max_shear * index + i] == false)
                    data_manager->host_data.shear_neigh[max_shear * index + i].x = -1;
            }
        }
    }

    // 2. Calculate contact forces and torques - per body basis
    //    Accumulate the contact forces and torques for all bodies that are
    //    involved in at least one contact, by reducing the contact forces and
    //    torques from all contacts these bodies are involved in. The number of
    //    bodies that experience at least one contact is 'ct_body_count'.
    thrust::sort_by_key(THRUST_PAR ext_body_id.begin(), ext_body_id.end(),
                        thrust::make_zip_iterator(thrust::make_tuple(ext_body_force.begin(), ext_body_torque.begin())));

    custom_vector<int> ct_body_id(data_manager->num_rigid_bodies);
    custom_vector<real3>& ct_body_force = data_manager->host_data.ct_body_force;
    custom_vector<real3>& ct_body_torque = data_manager->host_data.ct_body_torque;

    ct_body_force.resize(data_manager->num_rigid_bodies);
    ct_body_torque.resize(data_manager->num_rigid_bodies);

    // Reduce contact forces from all contacts and count bodies currently involved
    // in contact. We do this simultaneously for contact forces and torques, using
    // zip iterators.
    uint ct_body_count =
        (uint)(thrust::reduce_by_key(
                   THRUST_PAR ext_body_id.begin(), ext_body_id.end(),
                   thrust::make_zip_iterator(thrust::make_tuple(ext_body_force.begin(), ext_body_torque.begin())),
                   ct_body_id.begin(),
                   thrust::make_zip_iterator(thrust::make_tuple(ct_body_force.begin(), ct_body_torque.begin())),
#if defined _WIN32
                   // Windows compilers require an explicit-width type
                   thrust::equal_to<int64_t>(), sum_tuples()
#else
                   thrust::equal_to<int>(), sum_tuples()
#endif
                       )
                   .first -
               ct_body_id.begin());

    ct_body_force.resize(ct_body_count);
    ct_body_torque.resize(ct_body_count);

    // 3. Add contact forces and torques to existing forces (impulses):
    //    For all bodies involved in a contact, update the body forces and torques
    //    (scaled by the integration time step).
    host_AddContactForces(ct_body_count, ct_body_id);

    // 4. Set up map from all bodies in the system to bodies involved in a contact.
    host_SetContactForcesMap(ct_body_count, ct_body_id);
}

void ChIterativeSolverParallelSMC::ComputeD() {
    uint num_constraints = data_manager->num_constraints;
    if (num_constraints <= 0) {
        return;
    }

    uint num_bodies = data_manager->num_rigid_bodies;
    uint num_shafts = data_manager->num_shafts;
    uint num_dof = data_manager->num_dof;
    uint num_contacts = data_manager->num_rigid_contacts;
    uint num_bilaterals = data_manager->num_bilaterals;
    uint nnz_bilaterals = data_manager->nnz_bilaterals;

    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    if (D_T.capacity() > 0) {
        clear(D_T);
    }

    D_T.reserve(nnz_bilaterals);
    D_T.resize(num_constraints, num_dof, false);

    data_manager->bilateral->GenerateSparsity();
    data_manager->bilateral->Build_D();

    data_manager->host_data.D = trans(D_T);
    data_manager->host_data.M_invD = data_manager->host_data.M_inv * data_manager->host_data.D;
}

void ChIterativeSolverParallelSMC::ComputeE() {
    if (data_manager->num_constraints <= 0) {
        return;
    }

    data_manager->host_data.E.resize(data_manager->num_constraints);
    reset(data_manager->host_data.E);

    data_manager->bilateral->Build_E();
}

void ChIterativeSolverParallelSMC::ComputeR() {
    if (data_manager->num_constraints <= 0) {
        return;
    }

    data_manager->host_data.b.resize(data_manager->num_constraints);
    reset(data_manager->host_data.b);
    data_manager->bilateral->Build_b();

    data_manager->host_data.R_full =
        -data_manager->host_data.b - data_manager->host_data.D_T * data_manager->host_data.M_invk;
}

// -----------------------------------------------------------------------------
// This is the main function for advancing the system state in time. On entry,
// geometric contact information is available as calculated by the narrowphase
// collision detection. This function calculates contact forces, updates the
// generalized velocities, then enforces the velocity-level constraints for any
// bilateral (joint) constraints present in the system.
// -----------------------------------------------------------------------------
void ChIterativeSolverParallelSMC::RunTimeStep() {
    // This is the total number of constraints, note that there are no contacts
    data_manager->num_constraints = data_manager->num_bilaterals;
    data_manager->num_unilaterals = 0;

    // Calculate contact forces (impulses) and append them to the body forces
    data_manager->host_data.ct_body_map.resize(data_manager->num_rigid_bodies);
    Thrust_Fill(data_manager->host_data.ct_body_map, -1);

    if (data_manager->num_rigid_contacts > 0) {
        data_manager->system_timer.start("ChIterativeSolverParallelSMC_ProcessContact");
        ProcessContacts();
        data_manager->system_timer.stop("ChIterativeSolverParallelSMC_ProcessContact");
    }

    // Generate the mass matrix and compute M_inv_k
    ComputeInvMassMatrix();

    // If there are (bilateral) constraints, calculate Lagrange multipliers.
    if (data_manager->num_constraints != 0) {
        data_manager->system_timer.start("ChIterativeSolverParallel_Setup");

        data_manager->bilateral->Setup(data_manager);

        solver->current_iteration = 0;
        data_manager->measures.solver.total_iteration = 0;
        data_manager->measures.solver.maxd_hist.clear();            ////
        data_manager->measures.solver.maxdeltalambda_hist.clear();  ////  currently not used

        solver->Setup(data_manager);

        data_manager->system_timer.stop("ChIterativeSolverParallel_Setup");

        // Set the initial guess for the iterative solver to zero.
        data_manager->host_data.gamma.resize(data_manager->num_constraints);
        data_manager->host_data.gamma.reset();

        // Compute the jacobian matrix, the compliance matrix and the right hand side
        data_manager->system_timer.start("ChIterativeSolverParallel_Matrices");
        ComputeD();
        ComputeE();
        ComputeR();
        data_manager->system_timer.stop("ChIterativeSolverParallel_Matrices");

        ShurProductBilateral.Setup(data_manager);

        bilateral_solver->Setup(data_manager);

        // Solve for the Lagrange multipliers associated with bilateral constraints.
        PerformStabilization();
    }

    // Update velocity (linear and angular)
    ComputeImpulses();

    for (int i = 0; i < data_manager->measures.solver.maxd_hist.size(); i++) {
        AtIterationEnd(data_manager->measures.solver.maxd_hist[i], data_manager->measures.solver.maxdeltalambda_hist[i],
                       i);
    }
    tot_iterations = (int)data_manager->measures.solver.maxd_hist.size();
}

void ChIterativeSolverParallelSMC::ComputeImpulses() {
    DynamicVector<real>& v = data_manager->host_data.v;
    const DynamicVector<real>& M_invk = data_manager->host_data.M_invk;
    const DynamicVector<real>& gamma = data_manager->host_data.gamma;

    uint num_unilaterals = data_manager->num_unilaterals;
    uint num_bilaterals = data_manager->num_bilaterals;

    if (data_manager->num_constraints > 0) {
        ConstSubVectorType gamma_b = blaze::subvector(gamma, num_unilaterals, num_bilaterals);
        v = M_invk + data_manager->host_data.M_invD * gamma_b;
    } else {
        v = M_invk;
    }
}
