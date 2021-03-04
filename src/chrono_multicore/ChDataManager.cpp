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
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================
//
// Description: This class contains manages all data associated with a multicore
// system. Rather than passing in individual data parameters to different parts
// of the code like the collision detection and the solver, passing a pointer to
// a data manager is more convenient from a development perspective.
//
// =============================================================================

#include "chrono_multicore/ChDataManager.h"
#include "chrono_multicore/physics/Ch3DOFContainer.h"
#include "chrono_multicore/collision/ChCollision.h"

#include "chrono/serialization/ChArchive.h"
#include "chrono/serialization/ChBlazeArchive.h"
#include "chrono/core/ChStream.h"

using namespace chrono;
using namespace chrono::collision;


ChMulticoreDataManager::ChMulticoreDataManager()
    : num_rigid_contacts(0),
      num_rigid_fluid_contacts(0),
      num_fluid_contacts(0),
      num_rigid_shapes(0),
      num_rigid_bodies(0),
      num_fluid_bodies(0),
      num_unilaterals(0),
      num_bilaterals(0),
      num_constraints(0),
      num_shafts(0),
      num_motors(0),
      num_linmotors(0),
      num_rotmotors(0),
      num_dof(0),
      num_fea_nodes(0),
      num_fea_tets(0),
      num_rigid_tet_contacts(0),
      num_rigid_tet_node_contacts(0),
      num_marker_tet_contacts(0),
      nnz_bilaterals(0),
      add_contact_callback(nullptr),
      composition_strategy(new ChMaterialCompositionStrategy) {
    node_container = chrono_types::make_shared<Ch3DOFContainer>();
    fea_container = chrono_types::make_shared<Ch3DOFContainer>();
    node_container->data_manager = this;
    fea_container->data_manager = this;

    broadphase = new ChCBroadphase;
    narrowphase = new ChCNarrowphaseDispatch;
    aabb_generator = new ChCAABBGenerator;
    broadphase->data_manager = this;
    narrowphase->data_manager = this;
    aabb_generator->data_manager = this;
}

ChMulticoreDataManager::~ChMulticoreDataManager() {
    delete narrowphase;
    delete broadphase;
    delete aabb_generator;
}

int ChMulticoreDataManager::OutputBlazeVector(DynamicVector<real> src, std::string filename) {
    const char* numformat = "%.16g";
    ChStreamOutAsciiFile stream(filename.c_str());
    stream.SetNumFormat(numformat);

    for (int i = 0; i < src.size(); i++)
        stream << src[i] << "\n";

    return 0;
}

int ChMulticoreDataManager::OutputBlazeMatrix(CompressedMatrix<real> src, std::string filename) {
    const char* numformat = "%.16g";
    ChStreamOutAsciiFile stream(filename.c_str());
    stream.SetNumFormat(numformat);

    stream << src.rows() << " " << src.columns() << "\n";
    for (int i = 0; i < src.rows(); ++i) {
        for (CompressedMatrix<real>::Iterator it = src.begin(i); it != src.end(i); ++it) {
            stream << i << " " << it->index() << " " << it->value() << "\n";
        }
    }

    return 0;
}


int ChMulticoreDataManager::ExportCurrentSystem(std::string output_dir) {
    int offset = 0;
    if (settings.solver.solver_mode == SolverMode::NORMAL) {
        offset = num_rigid_contacts;
    } else if (settings.solver.solver_mode == SolverMode::SLIDING) {
        offset = 3 * num_rigid_contacts;
    } else if (settings.solver.solver_mode == SolverMode::SPINNING) {
        offset = 6 * num_rigid_contacts;
    }

    // fill in the information for constraints and friction
    DynamicVector<real> fric(num_constraints, -2.0);
    for (unsigned int i = 0; i < num_rigid_contacts; i++) {
        if (settings.solver.solver_mode == SolverMode::NORMAL) {
            fric[i] = host_data.fric_rigid_rigid[i].x;
        } else if (settings.solver.solver_mode == SolverMode::SLIDING) {
            fric[3 * i] = host_data.fric_rigid_rigid[i].x;
            fric[3 * i + 1] = -1;
            fric[3 * i + 2] = -1;
        } else if (settings.solver.solver_mode == SolverMode::SPINNING) {
            fric[6 * i] = host_data.fric_rigid_rigid[i].x;
            fric[6 * i + 1] = -1;
            fric[6 * i + 2] = -1;
            fric[6 * i + 3] = -1;
            fric[6 * i + 4] = -1;
            fric[6 * i + 5] = -1;
        }
    }

    // output r
    std::string filename = output_dir + "dump_r.dat";
    OutputBlazeVector(host_data.R, filename);

    // output b
    filename = output_dir + "dump_b.dat";
    OutputBlazeVector(host_data.b, filename);

    // output friction data
    filename = output_dir + "dump_fric.dat";
    OutputBlazeVector(fric, filename);

    CompressedMatrix<real> D_T;
    uint nnz_total = nnz_bilaterals;

    filename = output_dir + "dump_D.dat";
    OutputBlazeMatrix(host_data.D_T, filename);

    // output M_inv
    filename = output_dir + "dump_Minv.dat";
    OutputBlazeMatrix(host_data.M_inv, filename);

    return 0;
}

void ChMulticoreDataManager::PrintMatrix(CompressedMatrix<real> src) {
    const char* numformat = "%.16g";
    std::cout << src.rows() << " " << src.columns() << "\n";
    for (int i = 0; i < src.rows(); ++i) {
        std::cout << i << " ";
        for (int j = 0; j < src.columns(); j++) {
            std::cout << src(i, j) << " ";
        }
        std::cout << "\n";
    }
}

    // MEMBER FUNCTIONS FOR BINARY I/O
    // NOTE!!!In order to allow serialization with Chrono approach,
    // at least implement these two functions, with the exact names
    // ArchiveIN() and ArchiveOUT():

void ChMulticoreDataManager::ArchiveOUT(ChArchiveOut& marchive)  //##### for Chrono serialization
{
    marchive.VersionWrite<ChMulticoreDataManager>();
    // stream out all member data
    ArchiveOUTHostData(marchive);
    ArchiveOUTShapeData(marchive);
    marchive << CHNVP(node_container.get());
    marchive << CHNVP(fea_container.get());
    marchive << CHNVP(*rigid_rigid); // need to set pointer to data manager
    //marchive << CHNVP(*bilateral); // need to set pointer to data manager
    // broadphase // ??
    ArchiveOUTIndexingVariables(marchive);
    marchive << Fc_current;
}

void ChMulticoreDataManager::ArchiveIN(ChArchiveIn& marchive)  //##### for Chrono serialization
{
    // suggested: use versioning
    int version = marchive.VersionRead<ChMulticoreDataManager>();
    // stream in all member data
    ArchiveINHostData(marchive);
    ArchiveINShapeData(marchive);
    marchive >> CHNVP(*node_container);
    marchive >> CHNVP(*fea_container);
    ArchiveINIndexingVariables(marchive);
    marchive >> Fc_current;
}

void ChMulticoreDataManager::ArchiveOUTShapeData(ChArchiveOut& marchive) {
    marchive << CHNVP(shape_data.fam_rigid);
    marchive << CHNVP(shape_data.id_rigid);
    marchive << CHNVP(shape_data.typ_rigid);
    marchive << CHNVP(shape_data.local_rigid);
    marchive << CHNVP(shape_data.start_rigid);
    marchive << CHNVP(shape_data.length_rigid);

    marchive << CHNVP(shape_data.ObR_rigid);
    marchive << CHNVP(shape_data.ObA_rigid);

    marchive << CHNVP(shape_data.sphere_rigid);
    marchive << CHNVP(shape_data.box_like_rigid);
    marchive << CHNVP(shape_data.triangle_rigid);
    marchive << CHNVP(shape_data.capsule_rigid);
    marchive << CHNVP(shape_data.rbox_like_rigid);
    marchive << CHNVP(shape_data.convex_rigid);
    marchive << CHNVP(shape_data.tetrahedron_rigid);

    marchive << CHNVP(shape_data.triangle_global);
    marchive << CHNVP(shape_data.obj_data_A_global);
    marchive << CHNVP(shape_data.obj_data_R_global);
}


void ChMulticoreDataManager::ArchiveOUTHostData(ChArchiveOut& marchive) {
    marchive << CHNVP(host_data.aabb_min);
    marchive << CHNVP(host_data.aabb_max);

    marchive << CHNVP(host_data.aabb_min_tet);
    marchive << CHNVP(host_data.aabb_max_tet);

    marchive << CHNVP(host_data.pair_shapeIDs);
    marchive << CHNVP(host_data.contact_shapeIDs);

    marchive << CHNVP(host_data.norm_rigid_rigid);
    marchive << CHNVP(host_data.cpta_rigid_rigid);
    marchive << CHNVP(host_data.cptb_rigid_rigid);
    marchive << CHNVP(host_data.dpth_rigid_rigid);
    marchive << CHNVP(host_data.erad_rigid_rigid);
    marchive << CHNVP(host_data.bids_rigid_rigid);

    marchive << CHNVP(host_data.norm_rigid_fluid);
    marchive << CHNVP(host_data.cpta_rigid_fluid);
    marchive << CHNVP(host_data.dpth_rigid_fluid);
    marchive << CHNVP(host_data.neighbor_rigid_fluid);
    marchive << CHNVP(host_data.c_counts_rigid_fluid);

    marchive << CHNVP(host_data.neighbor_3dof_3dof);
    marchive << CHNVP(host_data.c_counts_3dof_3dof);
    marchive << CHNVP(host_data.particle_indices_3dof);
    marchive << CHNVP(host_data.reverse_mapping_3dof);

    marchive << CHNVP(host_data.norm_rigid_tet);
    marchive << CHNVP(host_data.cpta_rigid_tet);
    marchive << CHNVP(host_data.cptb_rigid_tet);
    marchive << CHNVP(host_data.dpth_rigid_tet);
    marchive << CHNVP(host_data.neighbor_rigid_tet);
    marchive << CHNVP(host_data.face_rigid_tet);
    marchive << CHNVP(host_data.c_counts_rigid_tet);

    marchive << CHNVP(host_data.norm_rigid_tet_node);
    marchive << CHNVP(host_data.cpta_rigid_tet_node);
    marchive << CHNVP(host_data.dpth_rigid_tet_node);
    marchive << CHNVP(host_data.neighbor_rigid_tet_node);
    marchive << CHNVP(host_data.c_counts_rigid_tet_node);

    marchive << CHNVP(host_data.norm_marker_tet);
    marchive << CHNVP(host_data.cpta_marker_tet);
    marchive << CHNVP(host_data.cptb_marker_tet);
    marchive << CHNVP(host_data.dpth_marker_tet);
    marchive << CHNVP(host_data.neighbor_marker_tet);
    marchive << CHNVP(host_data.face_marker_tet);
    marchive << CHNVP(host_data.c_counts_marker_tet);

    marchive << CHNVP(host_data.ct_force);
    marchive << CHNVP(host_data.ct_torque);

    marchive << CHNVP(host_data.ct_body_force);
    marchive << CHNVP(host_data.ct_body_torque);

    marchive << CHNVP(host_data.shear_neigh);
    marchive << CHNVP(host_data.shear_disp);
    marchive << CHNVP(host_data.contact_relvel_init);
    marchive << CHNVP(host_data.contact_duration);

    marchive << CHNVP(host_data.ct_body_map);

    marchive << CHNVP(host_data.fric_rigid_rigid);

    marchive << CHNVP(host_data.coh_rigid_rigid);

    marchive << CHNVP(host_data.compliance_rigid_rigid);

    marchive << CHNVP(host_data.modulus_rigid_rigid);
    marchive << CHNVP(host_data.adhesion_rigid_rigid);
    marchive << CHNVP(host_data.cr_rigid_rigid);
    marchive << CHNVP(host_data.smc_rigid_rigid);

    marchive << CHNVP(host_data.pos_rigid);
    marchive << CHNVP(host_data.rot_rigid);
    marchive << CHNVP(host_data.active_rigid);
    marchive << CHNVP(host_data.collide_rigid);
    marchive << CHNVP(host_data.mass_rigid);

    marchive << CHNVP(host_data.pos_3dof);
    marchive << CHNVP(host_data.sorted_pos_3dof);
    marchive << CHNVP(host_data.vel_3dof);
    marchive << CHNVP(host_data.sorted_vel_3dof);

    marchive << CHNVP(host_data.pos_node_fea);
    marchive << CHNVP(host_data.vel_node_fea);
    marchive << CHNVP(host_data.mass_node_fea);
    marchive << CHNVP(host_data.tet_indices);

    marchive << CHNVP(host_data.boundary_triangles_fea);
    marchive << CHNVP(host_data.boundary_node_fea);
    marchive << CHNVP(host_data.boundary_element_fea);
    marchive << CHNVP(host_data.boundary_family_fea);

    marchive << CHNVP(host_data.bilateral_type);

    marchive << CHNVP(host_data.bilateral_mapping);

    marchive << CHNVP(host_data.shaft_rot);
    marchive << CHNVP(host_data.shaft_inr);
    marchive << CHNVP(host_data.shaft_active);

    marchive << CHNVP(host_data.sliding_friction);
    marchive << CHNVP(host_data.cohesion);

    ChBlazeArchive::ArchiveOUTBlazeCompressedMatrix(marchive, host_data.Nshur);
    ChBlazeArchive::ArchiveOUTBlazeCompressedMatrix(marchive, host_data.D);
    ChBlazeArchive::ArchiveOUTBlazeCompressedMatrix(marchive, host_data.D_T);
    ChBlazeArchive::ArchiveOUTBlazeCompressedMatrix(marchive, host_data.M);
    ChBlazeArchive::ArchiveOUTBlazeCompressedMatrix(marchive, host_data.M_inv);
    ChBlazeArchive::ArchiveOUTBlazeCompressedMatrix(marchive, host_data.M_invD);

    ChBlazeArchive::ArchiveOUTBlazeDynamicVector(marchive, host_data.R_full);
    ChBlazeArchive::ArchiveOUTBlazeDynamicVector(marchive, host_data.R);
    ChBlazeArchive::ArchiveOUTBlazeDynamicVector(marchive, host_data.b);
    ChBlazeArchive::ArchiveOUTBlazeDynamicVector(marchive, host_data.s);
    ChBlazeArchive::ArchiveOUTBlazeDynamicVector(marchive, host_data.M_invk);
    ChBlazeArchive::ArchiveOUTBlazeDynamicVector(marchive, host_data.v);
    ChBlazeArchive::ArchiveOUTBlazeDynamicVector(marchive, host_data.hf);
    ChBlazeArchive::ArchiveOUTBlazeDynamicVector(marchive, host_data.gamma);
    ChBlazeArchive::ArchiveOUTBlazeDynamicVector(marchive, host_data.E);
    ChBlazeArchive::ArchiveOUTBlazeDynamicVector(marchive, host_data.Fc);

    marchive << CHNVP(host_data.bin_intersections);
    marchive << CHNVP(host_data.bin_number);
    marchive << CHNVP(host_data.bin_number_out);
    marchive << CHNVP(host_data.bin_aabb_number);
    marchive << CHNVP(host_data.bin_start_index);
    marchive << CHNVP(host_data.bin_num_contact);
}

void ChMulticoreDataManager::ArchiveOUTIndexingVariables(ChArchiveOut& marchive) {
    marchive << CHNVP(num_rigid_bodies);
    marchive << CHNVP(num_fluid_bodies);
    marchive << CHNVP(num_shafts);
    marchive << CHNVP(num_motors);
    marchive << CHNVP(num_linmotors);
    marchive << CHNVP(num_rotmotors);
    marchive << CHNVP(num_dof);
    marchive << CHNVP(num_rigid_shapes);
    marchive << CHNVP(num_rigid_contacts);
    marchive << CHNVP(num_rigid_fluid_contacts);
    marchive << CHNVP(num_fluid_contacts);
    marchive << CHNVP(num_unilaterals);
    marchive << CHNVP(num_bilaterals);
    marchive << CHNVP(num_constraints);
    marchive << CHNVP(num_fea_nodes);
    marchive << CHNVP(num_fea_tets);
    marchive << CHNVP(num_rigid_tet_contacts);
    marchive << CHNVP(num_marker_tet_contacts);
    marchive << CHNVP(num_rigid_tet_node_contacts);
    marchive << CHNVP(nnz_bilaterals);
    marchive << CHNVP(Fc_current);
}

void ChMulticoreDataManager::ArchiveINShapeData(ChArchiveIn& marchive) {
    marchive >> CHNVP(shape_data.fam_rigid);
    marchive >> CHNVP(shape_data.id_rigid);
    marchive >> CHNVP(shape_data.typ_rigid);
    marchive >> CHNVP(shape_data.local_rigid);
    marchive >> CHNVP(shape_data.start_rigid);
    marchive >> CHNVP(shape_data.length_rigid);

    marchive >> CHNVP(shape_data.ObR_rigid);
    marchive >> CHNVP(shape_data.ObA_rigid);

    marchive >> CHNVP(shape_data.sphere_rigid);
    marchive >> CHNVP(shape_data.box_like_rigid);
    marchive >> CHNVP(shape_data.triangle_rigid);
    marchive >> CHNVP(shape_data.capsule_rigid);
    marchive >> CHNVP(shape_data.rbox_like_rigid);
    marchive >> CHNVP(shape_data.convex_rigid);
    marchive >> CHNVP(shape_data.tetrahedron_rigid);

    marchive >> CHNVP(shape_data.triangle_global);
    marchive >> CHNVP(shape_data.obj_data_A_global);
    marchive >> CHNVP(shape_data.obj_data_R_global);
}


void ChMulticoreDataManager::ArchiveINHostData(ChArchiveIn& marchive) {
    marchive >> CHNVP(host_data.aabb_min);
    marchive >> CHNVP(host_data.aabb_max);

    marchive >> CHNVP(host_data.aabb_min_tet);
    marchive >> CHNVP(host_data.aabb_max_tet);

    marchive >> CHNVP(host_data.pair_shapeIDs);
    marchive >> CHNVP(host_data.contact_shapeIDs);

    marchive >> CHNVP(host_data.norm_rigid_rigid);
    marchive >> CHNVP(host_data.cpta_rigid_rigid);
    marchive >> CHNVP(host_data.cptb_rigid_rigid);
    marchive >> CHNVP(host_data.dpth_rigid_rigid);
    marchive >> CHNVP(host_data.erad_rigid_rigid);
    marchive >> CHNVP(host_data.bids_rigid_rigid);

    marchive >> CHNVP(host_data.norm_rigid_fluid);
    marchive >> CHNVP(host_data.cpta_rigid_fluid);
    marchive >> CHNVP(host_data.dpth_rigid_fluid);
    marchive >> CHNVP(host_data.neighbor_rigid_fluid);
    marchive >> CHNVP(host_data.c_counts_rigid_fluid);

    marchive >> CHNVP(host_data.neighbor_3dof_3dof);
    marchive >> CHNVP(host_data.c_counts_3dof_3dof);
    marchive >> CHNVP(host_data.particle_indices_3dof);
    marchive >> CHNVP(host_data.reverse_mapping_3dof);

    marchive >> CHNVP(host_data.norm_rigid_tet);
    marchive >> CHNVP(host_data.cpta_rigid_tet);
    marchive >> CHNVP(host_data.cptb_rigid_tet);
    marchive >> CHNVP(host_data.dpth_rigid_tet);
    marchive >> CHNVP(host_data.neighbor_rigid_tet);
    marchive >> CHNVP(host_data.face_rigid_tet);
    marchive >> CHNVP(host_data.c_counts_rigid_tet);

    marchive >> CHNVP(host_data.norm_rigid_tet_node);
    marchive >> CHNVP(host_data.cpta_rigid_tet_node);
    marchive >> CHNVP(host_data.dpth_rigid_tet_node);
    marchive >> CHNVP(host_data.neighbor_rigid_tet_node);
    marchive >> CHNVP(host_data.c_counts_rigid_tet_node);

    marchive >> CHNVP(host_data.norm_marker_tet);
    marchive >> CHNVP(host_data.cpta_marker_tet);
    marchive >> CHNVP(host_data.cptb_marker_tet);
    marchive >> CHNVP(host_data.dpth_marker_tet);
    marchive >> CHNVP(host_data.neighbor_marker_tet);
    marchive >> CHNVP(host_data.face_marker_tet);
    marchive >> CHNVP(host_data.c_counts_marker_tet);

    marchive >> CHNVP(host_data.ct_force);
    marchive >> CHNVP(host_data.ct_torque);

    marchive >> CHNVP(host_data.ct_body_force);
    marchive >> CHNVP(host_data.ct_body_torque);

    marchive >> CHNVP(host_data.shear_neigh);
    marchive >> CHNVP(host_data.shear_disp);
    marchive >> CHNVP(host_data.contact_relvel_init);
    marchive >> CHNVP(host_data.contact_duration);

    marchive >> CHNVP(host_data.ct_body_map);

    marchive >> CHNVP(host_data.fric_rigid_rigid);

    marchive >> CHNVP(host_data.coh_rigid_rigid);

    marchive >> CHNVP(host_data.compliance_rigid_rigid);

    marchive >> CHNVP(host_data.modulus_rigid_rigid);
    marchive >> CHNVP(host_data.adhesion_rigid_rigid);
    marchive >> CHNVP(host_data.cr_rigid_rigid);
    marchive >> CHNVP(host_data.smc_rigid_rigid);

    marchive >> CHNVP(host_data.pos_rigid);
    marchive >> CHNVP(host_data.rot_rigid);
    marchive >> CHNVP(host_data.active_rigid);
    marchive >> CHNVP(host_data.collide_rigid);
    marchive >> CHNVP(host_data.mass_rigid);

    marchive >> CHNVP(host_data.pos_3dof);
    marchive >> CHNVP(host_data.sorted_pos_3dof);
    marchive >> CHNVP(host_data.vel_3dof);
    marchive >> CHNVP(host_data.sorted_vel_3dof);

    marchive >> CHNVP(host_data.pos_node_fea);
    marchive >> CHNVP(host_data.vel_node_fea);
    marchive >> CHNVP(host_data.mass_node_fea);
    marchive >> CHNVP(host_data.tet_indices);

    marchive >> CHNVP(host_data.boundary_triangles_fea);
    marchive >> CHNVP(host_data.boundary_node_fea);
    marchive >> CHNVP(host_data.boundary_element_fea);
    marchive >> CHNVP(host_data.boundary_family_fea);

    marchive >> CHNVP(host_data.bilateral_type);

    marchive >> CHNVP(host_data.bilateral_mapping);

    marchive >> CHNVP(host_data.shaft_rot);
    marchive >> CHNVP(host_data.shaft_inr);
    marchive >> CHNVP(host_data.shaft_active);

    marchive >> CHNVP(host_data.sliding_friction);
    marchive >> CHNVP(host_data.cohesion);

    ChBlazeArchive::ArchiveINBlazeCompressedMatrix(marchive, host_data.Nshur);
    ChBlazeArchive::ArchiveINBlazeCompressedMatrix(marchive, host_data.D);
    ChBlazeArchive::ArchiveINBlazeCompressedMatrix(marchive, host_data.D_T);
    ChBlazeArchive::ArchiveINBlazeCompressedMatrix(marchive, host_data.M);
    ChBlazeArchive::ArchiveINBlazeCompressedMatrix(marchive, host_data.M_inv);
    ChBlazeArchive::ArchiveINBlazeCompressedMatrix(marchive, host_data.M_invD);

    ChBlazeArchive::ArchiveINBlazeDynamicVector(marchive, host_data.R_full);
    ChBlazeArchive::ArchiveINBlazeDynamicVector(marchive, host_data.R);
    ChBlazeArchive::ArchiveINBlazeDynamicVector(marchive, host_data.b);
    ChBlazeArchive::ArchiveINBlazeDynamicVector(marchive, host_data.s);
    ChBlazeArchive::ArchiveINBlazeDynamicVector(marchive, host_data.M_invk);
    ChBlazeArchive::ArchiveINBlazeDynamicVector(marchive, host_data.v);
    ChBlazeArchive::ArchiveINBlazeDynamicVector(marchive, host_data.hf);
    ChBlazeArchive::ArchiveINBlazeDynamicVector(marchive, host_data.gamma);
    ChBlazeArchive::ArchiveINBlazeDynamicVector(marchive, host_data.E);
    ChBlazeArchive::ArchiveINBlazeDynamicVector(marchive, host_data.Fc);

    marchive >> CHNVP(host_data.bin_intersections);
    marchive >> CHNVP(host_data.bin_number);
    marchive >> CHNVP(host_data.bin_number_out);
    marchive >> CHNVP(host_data.bin_aabb_number);
    marchive >> CHNVP(host_data.bin_start_index);
    marchive >> CHNVP(host_data.bin_num_contact);
}

void ChMulticoreDataManager::ArchiveINIndexingVariables(ChArchiveIn& marchive) {
    marchive >> CHNVP(num_rigid_bodies);
    marchive >> CHNVP(num_fluid_bodies);
    marchive >> CHNVP(num_shafts);
    marchive >> CHNVP(num_motors);
    marchive >> CHNVP(num_linmotors);
    marchive >> CHNVP(num_rotmotors);
    marchive >> CHNVP(num_dof);
    marchive >> CHNVP(num_rigid_shapes);
    marchive >> CHNVP(num_rigid_contacts);
    marchive >> CHNVP(num_rigid_fluid_contacts);
    marchive >> CHNVP(num_fluid_contacts);
    marchive >> CHNVP(num_unilaterals);
    marchive >> CHNVP(num_bilaterals);
    marchive >> CHNVP(num_constraints);
    marchive >> CHNVP(num_fea_nodes);
    marchive >> CHNVP(num_fea_tets);
    marchive >> CHNVP(num_rigid_tet_contacts);
    marchive >> CHNVP(num_marker_tet_contacts);
    marchive >> CHNVP(num_rigid_tet_node_contacts);
    marchive >> CHNVP(nnz_bilaterals);
    marchive >> CHNVP(Fc_current);
}

