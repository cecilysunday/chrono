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
    real* contact_relvel_init,  // initial relative normal velocity manitude per contact pair
    real* contact_duration,     // running duration of persistant contact between contact pairs
    int* ext_body_id,           // [output] body IDs (two per contact)
    real3* ext_body_force,      // [output] body force (two per contact)
    real3* ext_body_torque      // [output] body torque (two per contact)
) {
    // Format header and output file for debugging lines. Delete this after completing simple sphere validation tests
    /*real p1;  // relvel_n_mag (signed)
    real p2;  // relvel_t_mag (always positive)

    bool print_data = GetPrint();
    std::ofstream datao;
    static int runs = 0;
    
    if (runs == 0 && print_data) {
        datao.open(GetChronoOutputPath() + "/chronodat.txt");
        datao << "stp\t"
              << "body1\t"
              << "body2\t"
              << "v_rot_x\t"
              << "v_rot_y\t"
              << "v_rot_z\t"
              << "delta_n\t"
              << "delta_t\t"
              << "relvel_n\t"
              << "relvel_t\t"
              << "force_n_mag\t"
              << "force_t_stiff\t"
              << "force_t_damp\t"
              << "force_t_x\t"
              << "force_t_y\t"
              << "force_t_z\t"
              << "force_x\t"
              << "force_y\t"
              << "force_z\t"
              << "torque1_x\t"
              << "torque1_y\t"
              << "torque1_z\t"
              << "torque2_x\t"
              << "torque2_y\t"
              << "torque2_z\t"
              << "m_roll1_x\t"
              << "m_roll1_y\t"
              << "m_roll1_z\t"
              << "m_roll2_x\t"
              << "m_roll2_y\t"
              << "m_roll2_z\t"
              << "m_spin1_x\t"
              << "m_spin1_y\t"
              << "m_spin1_z\t"
              << "m_spin2_x\t"
              << "m_spin2_y\t"
              << "m_spin2_z\n";
        runs++;
    } else if (print_data) {
        datao.open(GetChronoOutputPath() + "/chronodat.txt", std::ios::app);
        runs++;
    }*/
    
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
        //chrono::GetLog() << "\nWARNING: Exited function_CalcContactForce() at contact step " << runs;
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

    // Calculate relative velocity (in global frame)
    // Note that relvel_n_mag is a signed quantity, while relvel_t_mag is an
    // actual magnitude (always positive).
    real3 relvel = vel2 - vel1;
    real relvel_n_mag = Dot(relvel, normal[index]);
    real3 relvel_n = relvel_n_mag * normal[index];
    real3 relvel_t = relvel - relvel_n;
    real relvel_t_mag = Length(relvel_t);

    //p1 = relvel_n_mag;
    //p2 = relvel_t_mag;

    // Calculate composite material properties
    // ---------------------------------------

    real m_eff = mass[body1] * mass[body2] / (mass[body1] + mass[body2]);

    real mu_eff = std::max(mu[body1], mu[body2]);  // strategy->CombineFriction(mu[body1], mu[body2]);
    real muRoll_eff = strategy->CombineFriction(muRoll[body1], muRoll[body2]);
    real muSpin_eff = strategy->CombineFriction(muSpin[body1], muSpin[body2]);
    real adhesion_eff = strategy->CombineCohesion(adhesion[body1], adhesion[body2]);
    real adhesionMultDMT_eff = strategy->CombineAdhesionMultiplier(adhesionMultDMT[body1], adhesionMultDMT[body2]);
    real adhesionSPerko_eff = strategy->CombineAdhesionMultiplier(adhesionSPerko[body1], adhesionSPerko[body2]);

    real E_eff, G_eff, cr_eff;
    real user_kn, user_kt, user_gn, user_gt;

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
    real kn_simple;
    real gn_simple;

    real t_contact = 0;
    real relvel_init = abs(relvel_n_mag);
    real delta_n = -depth[index];
    real3 delta_t = real3(0);

    int i;
    int contact_id;
    int shear_body1;
    int shear_body2;
    int shear_shape1;
    int shear_shape2;
    bool newcontact = true;

    if (displ_mode == ChSystemSMC::TangentialDisplacementModel::OneStep) {
        delta_t = relvel_t * dT;

    } else if (displ_mode == ChSystemSMC::TangentialDisplacementModel::MultiStep) {
        delta_t = relvel_t * dT;

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
                contact_duration[ctIdUnrolled] += dT;
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
                    contact_relvel_init[ctIdUnrolled] = relvel_init;
                    contact_duration[ctIdUnrolled] = 0;
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

        // Load the charecteristic velocity and contact_duration from the contact history
        relvel_init = contact_relvel_init[ctSaveId];
        t_contact = contact_duration[ctSaveId];
    }

    double eps = std::numeric_limits<double>::epsilon();

    switch (contact_model) {
        case ChSystemSMC::ContactForceModel::Hooke:
            if (use_mat_props) {
                real tmp_k = (16.0 / 15) * Sqrt(eff_radius[index]) * E_eff;
                char_vel = (displ_mode == ChSystemSMC::TangentialDisplacementModel::MultiStep) ? relvel_init : char_vel;
                real v2 = char_vel * char_vel;
                real loge = (cr_eff < eps) ? Log(eps) : Log(cr_eff);
                loge = (cr_eff > 1 - eps) ? Log(1 - eps) : loge;
                real tmp_g = 1 + Pow(CH_C_PI / loge, 2);
                kn = tmp_k * Pow(m_eff * v2 / tmp_k, 1.0 / 5);
                kt = kn;
                gn = Sqrt(4 * m_eff * kn / tmp_g);
                gt = gn;
            } else {
                kn = user_kn;
                kt = user_kt;
                gn = m_eff * user_gn;
                gt = m_eff * user_gt;
            }

            kn_simple = kn;
            gn_simple = gn;

            break;

        case ChSystemSMC::ContactForceModel::Hertz:
            if (use_mat_props) {
                real sqrt_Rd = Sqrt(eff_radius[index] * delta_n);
                real Sn = 2 * E_eff * sqrt_Rd;
                real St = 8 * G_eff * sqrt_Rd;
                real loge = (cr_eff < eps) ? Log(eps) : Log(cr_eff);
                real beta = loge / Sqrt(loge * loge + CH_C_PI * CH_C_PI);
                kn = (2.0 / 3) * Sn;
                kt = St;
                gn = -2 * Sqrt(5.0 / 6) * beta * Sqrt(Sn * m_eff);
                gt = -2 * Sqrt(5.0 / 6) * beta * Sqrt(St * m_eff);
            } else {
                real tmp = eff_radius[index] * Sqrt(delta_n);
                kn = tmp * user_kn;
                kt = tmp * user_kt;
                gn = tmp * m_eff * user_gn;
                gt = tmp * m_eff * user_gt;
            }

            kn_simple = kn / Sqrt(delta_n);
            gn_simple = gn / Pow(delta_n, 1.0 / 4.0);

            break;

        case ChSystemSMC::Flores:
            if (use_mat_props) {
                real sqrt_Rd = Sqrt(eff_radius[index] * delta_n);
                real Sn = 2 * E_eff * sqrt_Rd;
                real St = 8 * G_eff * sqrt_Rd;
                real loge = (cr_eff < eps) ? Log(eps) : Log(cr_eff);
                real beta = loge / Sqrt(loge * loge + CH_C_PI * CH_C_PI);
                real cr = (cr_eff < eps) ? 0.01 : cr_eff;
                cr = (cr_eff > 1.0 - eps) ? 1.0 - eps : cr;
                char_vel = (displ_mode == ChSystemSMC::TangentialDisplacementModel::MultiStep) ? relvel_init : char_vel;
                kn = (2.0 / 3.0) * Sn;
                kt = (2.0 / 3.0) * St;
                gn = 8.0 * (1.0 - cr) * kn * delta_n / (5.0 * cr * char_vel);
                gt = -2 * Sqrt(5.0 / 6) * beta * Sqrt(St * m_eff);  // Need to multiply St by 2/3 here as well ?
            } else {
                real tmp = eff_radius[index] * Sqrt(delta_n);
                kn = tmp * user_kn;
                kt = tmp * user_kt;
                gn = tmp * m_eff * user_gn * delta_n;
                gt = tmp * m_eff * user_gt;
            }

            kn_simple = kn / Sqrt(delta_n);
            gn_simple = gn / Pow(delta_n, 3.0 / 2.0);

            break;

        case ChSystemSMC::ContactForceModel::PlainCoulomb:
            if (use_mat_props) {
                real sqrt_Rd = Sqrt(delta_n);
                real Sn = 2 * E_eff * sqrt_Rd;
                real St = 8 * G_eff * sqrt_Rd;
                real loge = (cr_eff < eps) ? Log(eps) : Log(cr_eff);
                real beta = loge / Sqrt(loge * loge + CH_C_PI * CH_C_PI);
                kn = (2.0 / 3) * Sn;
                gn = -2 * Sqrt(5.0 / 6) * beta * Sqrt(Sn * m_eff);
            } else {
                real tmp = Sqrt(delta_n);
                kn = tmp * user_kn;
                gn = tmp * user_gn;
            }

            kn_simple = kn / Sqrt(delta_n);
            gn_simple = gn / Pow(delta_n, 1.0 / 4.0);

            kt = 0;
            gt = 0;

            {
                real forceN_mag = kn * delta_n - gn * relvel_n_mag;
                real forceT_mag = mu_eff * Tanh(5.0 * relvel_t_mag) * forceN_mag;

                // Accumulate normal and tangential forces
                real3 force = forceN_mag * normal[index];
                if (relvel_t_mag >= min_slip_vel)
                    force -= (forceT_mag / relvel_t_mag) * relvel_t;

                // Convert force into the local body frames and calculate induced torques
                real3 torque1_loc = Cross(pt1_loc, RotateT(force, rot[body1]));
                real3 torque2_loc = Cross(pt2_loc, RotateT(force, rot[body2]));

                // If the duration of the current contact is less than the durration of a typical collision,
                // do not apply friction. Rolling and spinning friction should only be applied to persistant contacts
                real d_coeff = gn_simple / (2.0 * m_eff * Sqrt(kn_simple / m_eff));
                real t_collision = CH_C_PI * Sqrt(m_eff / (kn_simple * (1 - d_coeff * d_coeff)));

                if (t_contact <= t_collision) {
                    muRoll_eff = 0.0;
                    muSpin_eff = 0.0;
                }

				// Compute some additional vales needed for the rolling and twisting friction calculations
                real3 v_rot = Rotate(Cross(o_body2, pt2_loc), rot[body2]) - Rotate(Cross(o_body1, pt1_loc), rot[body1]);
                real3 rel_o = Rotate(o_body2, rot[body2]) - Rotate(o_body1, rot[body1]);

                // Calculate rolling friction torque as M_roll = mu_r * R * (F_N x v_rot) / |v_rot| (Schwartz et al. 2012)
                real3 m_roll1 = real3(0);
                real3 m_roll2 = real3(0);

                if (Length(v_rot) > min_roll_vel) {
                    m_roll1 = muRoll_eff * Cross(forceN_mag * pt1_loc, RotateT(v_rot, rot[body1])) / Length(v_rot);
                    m_roll2 = muRoll_eff * Cross(forceN_mag * pt2_loc, RotateT(v_rot, rot[body2])) / Length(v_rot);
                }

                // Calculate spinning friction torque as M_spin = -mu_t * r_c * ((w_n - w_p) . F_n / |w_n - w_p|) * n
                // r_c is the radius of the circle resulting from the intersecting body surfaces (Schwartz et al. 2012)
                real3 m_spin1 = real3(0);
                real3 m_spin2 = real3(0);

                if (Length(rel_o) > min_spin_vel) {
                    real r1 = Length(pt1_loc);  // r1 = eff_radius[index];
                    real r2 = Length(pt2_loc);  // r2 = r1;
                    real xc = (r1 * r1 - r2 * r2) / (2 * (r1 + r2 - delta_n)) + 0.5 * (r1 + r2 - delta_n);
                    real rc = r1 * r1 - xc * xc;
                    rc = (rc < eps) ? eps : Sqrt(rc);

                    m_spin1 = muSpin_eff * rc *
                              RotateT(Dot(rel_o, forceN_mag * normal[index]) * normal[index], rot[body1]) /
                              Length(rel_o);
                    m_spin2 = muSpin_eff * rc *
                              RotateT(Dot(rel_o, forceN_mag * normal[index]) * normal[index], rot[body2]) /
                              Length(rel_o);
				}

                // Account for adhesion
                switch (adhesion_model) {
                    case ChSystemSMC::AdhesionForceModel::Constant:
                        force -= adhesion_eff * normal[index];
                        break;
                    case ChSystemSMC::AdhesionForceModel::DMT:
                        force -= adhesionMultDMT_eff * Sqrt(eff_radius[index]) * normal[index];
                        break;
                    case ChSystemSMC::AdhesionForceModel::Perko:
                        force -= adhesionSPerko_eff * adhesionSPerko_eff * 3.6E-2 * eff_radius[index] * normal[index];
                        break;
                }

                ext_body_id[2 * index] = body1;
                ext_body_id[2 * index + 1] = body2;
                ext_body_force[2 * index] = -force;
                ext_body_force[2 * index + 1] = force;
                ext_body_torque[2 * index] = -torque1_loc + m_roll1 + m_spin1;
                ext_body_torque[2 * index + 1] = torque2_loc - m_roll2 - m_spin2;
            }

            return;
    }

    // Calculate the the normal and tangential contact forces.
    // The normal force is a magnitude, and it will be applied along the contact
    // normal direction (negative relative to body1 & positive relative to body2).
    // The tangential force is a vector with two parts: one depends on the stored
    // contact history tangential (or shear) displacement vector delta_t, and the
    // other depends on the current relative velocity vector (for viscous damping).
    real forceN_mag = kn * delta_n - gn * relvel_n_mag;
    real3 forceT_stiff = kt * delta_t;
    real3 forceT_damp = gt * relvel_t;

    // Apply Coulomb friction law.
    // We must enforce force_T_mag <= mu_eff * |forceN_mag|.
    // If force_T_mag > mu_eff * |forceN_mag| and there is shear displacement
    // due to contact history, then the shear displacement is scaled so that
    // the tangential force will be correct if force_T_mag subsequently drops
    // below the Coulomb limit.
    real3 forceT = forceT_stiff + forceT_damp;
    real forceT_mag = Length(forceT);
    real forceT_slide = mu_eff * forceN_mag;
    if (forceT_mag > abs(forceT_slide)) {
        if (forceT_mag > eps && kt > eps) {
            real3 forceT_dir = forceT / forceT_mag;
            forceT_mag = forceT_slide;
            forceT = forceT_mag * forceT_dir;
            if (displ_mode == ChSystemSMC::TangentialDisplacementModel::MultiStep) {
                delta_t = (forceT - forceT_damp) / kt;
                if (shear_body1 == body1) {
                    shear_disp[max_shear * shear_body1 + contact_id] = delta_t;
                } else {
                    shear_disp[max_shear * shear_body1 + contact_id] = -delta_t;
                }
            }
        } else {
            forceT = real3(0);
        }
    }

    // Accumulate normal and tangential forces
    real3 force = forceN_mag * normal[index] - forceT;

    // Body forces (in global frame) & torques (in local frame)
    // --------------------------------------------------------

    // Convert force into the local body frames and calculate induced torques
    //    n' = s' x F' = s' x (A*F)
    real3 torque1_loc = Cross(pt1_loc, RotateT(force, rot[body1]));
    real3 torque2_loc = Cross(pt2_loc, RotateT(force, rot[body2]));

    // If the duration of the current contact is less than the durration of a typical collision, 
    // do not apply friction. Rolling and spinning friction should only be applied to persistant contacts
    real d_coeff = gn_simple / (2.0 * m_eff * Sqrt(kn_simple / m_eff));
    real t_collision = CH_C_PI * Sqrt(m_eff / (kn_simple * (1 - d_coeff * d_coeff)));

    if (t_contact <= t_collision) {
        muRoll_eff = 0.0;
        muSpin_eff = 0.0;
    }

    // Compute some additional vales needed for the rolling and twisting friction calculations
    real3 v_rot = Rotate(Cross(o_body2, pt2_loc), rot[body2]) - Rotate(Cross(o_body1, pt1_loc), rot[body1]);
    real3 rel_o = Rotate(o_body2, rot[body2]) - Rotate(o_body1, rot[body1]);

    // Calculate rolling friction torque as M_roll = mu_r * R * (F_N x v_rot) / |v_rot| (Schwartz et al. 2012)
    real3 m_roll1 = real3(0);
    real3 m_roll2 = real3(0);

    if (Length(v_rot) > min_roll_vel) {
        m_roll1 = muRoll_eff * Cross(forceN_mag * pt1_loc, RotateT(v_rot, rot[body1])) / Length(v_rot);
        m_roll2 = muRoll_eff * Cross(forceN_mag * pt2_loc, RotateT(v_rot, rot[body2])) / Length(v_rot);
    }

    // Calculate spinning friction torque as M_spin = -mu_t * r_c * ((w_n - w_p) . F_n / |w_n - w_p|) * n
    // r_c is the radius of the circle resulting from the intersecting body surfaces (Schwartz et al. 2012)
    real3 m_spin1 = real3(0);
    real3 m_spin2 = real3(0);

    if (Length(rel_o) > min_spin_vel) {
        real r1 = Length(pt1_loc); // r1 = eff_radius[index]; 
        real r2 = Length(pt2_loc); // r2 = r1;
        real xc = (r1 * r1 - r2 * r2) / (2 * (r1 + r2 - delta_n)) + 0.5 * (r1 + r2 - delta_n);
        real rc = r1 * r1 - xc * xc;
        rc = (rc < eps) ? eps : Sqrt(rc);

        m_spin1 = muSpin_eff * rc * RotateT(Dot(rel_o, forceN_mag * normal[index]) * normal[index], rot[body1]) /
                  Length(rel_o);
        m_spin2 = muSpin_eff * rc * RotateT(Dot(rel_o, forceN_mag * normal[index]) * normal[index], rot[body2]) /
                  Length(rel_o);
    }

    // Account for adhesion
    switch (adhesion_model) {
        case ChSystemSMC::AdhesionForceModel::Constant:
            force -= adhesion_eff * normal[index];
            break;
        case ChSystemSMC::AdhesionForceModel::DMT:
            force -= adhesionMultDMT_eff * Sqrt(eff_radius[index]) * normal[index];
            break;
        case ChSystemSMC::AdhesionForceModel::Perko:
            force -= adhesionSPerko_eff * adhesionSPerko_eff * 3.6E-2 * eff_radius[index] * normal[index];
            break;
    }

    // Store body forces and torques, duplicated for the two bodies.
    ext_body_id[2 * index] = body1;
    ext_body_id[2 * index + 1] = body2;
    ext_body_force[2 * index] = -force;
    ext_body_force[2 * index + 1] = force;
    ext_body_torque[2 * index] = -torque1_loc + m_roll1 + m_spin1;
    ext_body_torque[2 * index + 1] = torque2_loc - m_roll2 - m_spin2;

    // Print collision metadata to tab chronodat.txt file. Delete this after completing simple sphere validation tests
    /* if (print_data) {
        datao << runs << "\t" 
			  << body1 << "\t" 
			  << body2 << "\t" 
              << v_rot.x << "\t"
              << v_rot.y << "\t"
              << v_rot.z << "\t"
              << delta_n << "\t"
              << Length(delta_t) << "\t"
              << p1 << "\t"
              << p2 << "\t"
              << forceN_mag << "\t"
              << Length(forceT_stiff) << "\t"
              << Length(forceT_damp) << "\t"
              << forceT.x << "\t"
              << forceT.y << "\t"
              << forceT.z << "\t"
              << force.x << "\t"
              << force.y << "\t"
              << force.z << "\t"
              << Rotate(torque1_loc, rot[body1]).x << "\t"
              << Rotate(torque1_loc, rot[body1]).y << "\t"
              << Rotate(torque1_loc, rot[body1]).z << "\t"
              << Rotate(torque2_loc, rot[body2]).x << "\t"
              << Rotate(torque2_loc, rot[body2]).y << "\t"
              << Rotate(torque2_loc, rot[body2]).z << "\t"
              << Rotate(m_roll1, rot[body1]).x << "\t"
              << Rotate(m_roll1, rot[body1]).y << "\t"
              << Rotate(m_roll1, rot[body1]).z << "\t"
              << Rotate(m_roll2, rot[body2]).x << "\t"
              << Rotate(m_roll2, rot[body2]).y << "\t"
              << Rotate(m_roll2, rot[body2]).z << "\t"
              << Rotate(m_spin1, rot[body1]).x << "\t"
              << Rotate(m_spin1, rot[body1]).y << "\t"
              << Rotate(m_spin1, rot[body1]).z << "\t"
              << Rotate(m_spin2, rot[body2]).x << "\t"
              << Rotate(m_spin2, rot[body2]).y << "\t"
              << Rotate(m_spin2, rot[body2]).z << "\n";
        datao.close();
    }*/
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
            data_manager->host_data.bids_rigid_rigid.data(), shape_pairs.data(), data_manager->host_data.cpta_rigid_rigid.data(), 
            data_manager->host_data.cptb_rigid_rigid.data(), data_manager->host_data.norm_rigid_rigid.data(), 
            data_manager->host_data.dpth_rigid_rigid.data(), data_manager->host_data.erad_rigid_rigid.data(),
            data_manager->host_data.shear_neigh.data(), shear_touch.data(), data_manager->host_data.shear_disp.data(),
            data_manager->host_data.contact_relvel_init.data(), data_manager->host_data.contact_duration.data(), ext_body_id.data(), 
            ext_body_force.data(), ext_body_torque.data());
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
            ).first -
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
    uint num_motors = data_manager->num_motors;
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
    m_iterations = (int)data_manager->measures.solver.maxd_hist.size();
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
