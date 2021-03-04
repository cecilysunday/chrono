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
    // suggested: use versioning
    marchive.VersionWrite<ChMulticoreDataManager>();
    // stream out all member data

    ArchiveOUTShapeData(marchive);
    ArchiveOUTIndexingVariables(marchive);
}

void ChMulticoreDataManager::ArchiveIN(ChArchiveIn& marchive)  //##### for Chrono serialization
{
    // suggested: use versioning
    int version = marchive.VersionRead<ChMulticoreDataManager>();
    // stream in all member data
    ArchiveINShapeData(marchive);
    ArchiveINIndexingVariables(marchive);
}

void ChMulticoreDataManager::ArchiveOUTHostData(ChArchiveOut& marchive) {
    for (int i = 0; i < host_data.aabb_min.size(); ++i) {
      std::cout << "iOut: " << i << std::endl;
      marchive << CHNVP(host_data.aabb_min[i].x);
    }
}

void ChMulticoreDataManager::ArchiveOUTVectorReal(ChArchiveOut& marchive, custom_vector<real>& vector) {
    for (int i = 0; i < vector.size(); ++i) {
        marchive << CHNVP(vector[i]);
    }
}
void ChMulticoreDataManager::ArchiveOUTVectorReal3(ChArchiveOut& marchive, custom_vector<real3>& vector) {
    for (int i = 0; i < vector.size(); ++i) {
        for (int j = 0; j < 4; ++j) {
            marchive << CHNVP(vector[i][j]);
        }
    }
}
void ChMulticoreDataManager::ArchiveOUTVectorReal4(ChArchiveOut& marchive, custom_vector<real4>& vector) {
    for (int i = 0; i < vector.size(); ++i) {
        for (int j = 0; j < 4; ++j) {
            marchive << CHNVP(vector[i][j]);
        }
    }
}
void ChMulticoreDataManager::ArchiveOUTVectorQuaternion(ChArchiveOut& marchive, custom_vector<quaternion>& vector) {
    for (int i = 0; i < vector.size(); ++i) {
        for (int j = 0; j < 4; ++j) {
            marchive << CHNVP(vector[i][j]);
        }
    }
}
void ChMulticoreDataManager::ArchiveOUTVectorInt(ChArchiveOut& marchive, custom_vector<int>& vector) {
    for (int i = 0; i < vector.size(); ++i) {
        marchive << CHNVP(vector[i]);
    }
}
void ChMulticoreDataManager::ArchiveOUTVectorUInt(ChArchiveOut& marchive, custom_vector<uint>& vector) {
    for (int i = 0; i < vector.size(); ++i) {
        marchive << CHNVP(vector[i]);
    }
}
void ChMulticoreDataManager::ArchiveOUTShapeData(ChArchiveOut& marchive) {
    // for (int i = 0; i < shape_data.fam_rigid.size(); ++i) {
    //     marchive << CHNVP(shape_data.fam_rigid[i].x);
    //     marchive << CHNVP(shape_data.fam_rigid[i].y);
    // }

    ArchiveOUTVectorUInt(marchive, shape_data.id_rigid);
    ArchiveOUTVectorInt(marchive, shape_data.typ_rigid);
    ArchiveOUTVectorInt(marchive, shape_data.local_rigid);
    ArchiveOUTVectorInt(marchive, shape_data.start_rigid);
    ArchiveOUTVectorInt(marchive, shape_data.length_rigid);

    // ArchiveOUTVectorQuaternion(marchive, shape_data.ObR_rigid);
    // ArchiveOUTVectorReal3(marchive, shape_data.ObA_rigid);

    // ArchiveOUTVectorReal(marchive, shape_data.sphere_rigid);
    // ArchiveOUTVectorReal3(marchive, shape_data.box_like_rigid);
    // ArchiveOUTVectorReal3(marchive, shape_data.triangle_rigid);
    // for (int i = 0; i < shape_data.sphere_rigid.size(); ++i) {
    //     marchive << CHNVP(shape_data.capsule_rigid[i].x);
    //     marchive << CHNVP(shape_data.capsule_rigid[i].y);
    // }
    // ArchiveOUTVectorReal4(marchive, shape_data.rbox_like_rigid);
    // ArchiveOUTVectorReal3(marchive, shape_data.convex_rigid);
    // ArchiveOUTVectorInt(marchive, shape_data.tetrahedron_rigid);
    // ArchiveOUTVectorReal3(marchive, shape_data.triangle_global);
    // ArchiveOUTVectorReal3(marchive, shape_data.obj_data_A_global);
    // ArchiveOUTVectorQuaternion(marchive, shape_data.obj_data_R_global);
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

void ChMulticoreDataManager::ArchiveINHostData(ChArchiveIn& marchive) {
    for (int i = 0; i < host_data.aabb_min.size(); ++i) {
      std::cout << "iIn: " << i << std::endl;
      marchive >> CHNVP(host_data.aabb_min[i].x);
    }
}

void ChMulticoreDataManager::ArchiveINVectorReal(ChArchiveIn& marchive, custom_vector<real>& vector) {
    for (int i = 0; i < vector.size(); ++i) {
        marchive >> CHNVP(vector[i]);
    }
}
void ChMulticoreDataManager::ArchiveINVectorReal3(ChArchiveIn& marchive, custom_vector<real3>& vector) {
    for (int i = 0; i < vector.size(); ++i) {
        for (int j = 0; j < 4; ++j) {
            marchive >> CHNVP(vector[i][j]);
        }
    }
}
void ChMulticoreDataManager::ArchiveINVectorReal4(ChArchiveIn& marchive, custom_vector<real4>& vector) {
    for (int i = 0; i < vector.size(); ++i) {
        for (int j = 0; j < 4; ++j) {
            marchive >> CHNVP(vector[i][j]);
        }
    }
}
void ChMulticoreDataManager::ArchiveINVectorQuaternion(ChArchiveIn& marchive, custom_vector<quaternion>& vector) {
    for (int i = 0; i < vector.size(); ++i) {
        for (int j = 0; j < 4; ++j) {
            marchive >> CHNVP(vector[i][j]);
        }
    }
}
void ChMulticoreDataManager::ArchiveINVectorInt(ChArchiveIn& marchive, custom_vector<int>& vector) {
    for (int i = 0; i < vector.size(); ++i) {
        marchive >> CHNVP(vector[i]);
    }
}
void ChMulticoreDataManager::ArchiveINVectorUInt(ChArchiveIn& marchive, custom_vector<uint>& vector) {
    for (int i = 0; i < vector.size(); ++i) {
        marchive >> CHNVP(vector[i]);
    }
}
void ChMulticoreDataManager::ArchiveINShapeData(ChArchiveIn& marchive) {
    // for (int i = 0; i < shape_data.fam_rigid.size(); ++i) {
    //     marchive >> CHNVP(shape_data.fam_rigid[i].x);
    //     marchive >> CHNVP(shape_data.fam_rigid[i].y);
    // }

    ArchiveINVectorUInt(marchive, shape_data.id_rigid);
    ArchiveINVectorInt(marchive, shape_data.typ_rigid);
    ArchiveINVectorInt(marchive, shape_data.local_rigid);
    ArchiveINVectorInt(marchive, shape_data.start_rigid);
    ArchiveINVectorInt(marchive, shape_data.length_rigid);

    // ArchiveINVectorQuaternion(marchive, shape_data.ObR_rigid);
    // ArchiveINVectorReal3(marchive, shape_data.ObA_rigid);

    // ArchiveINVectorReal(marchive, shape_data.sphere_rigid);
    // ArchiveINVectorReal3(marchive, shape_data.box_like_rigid);
    // ArchiveINVectorReal3(marchive, shape_data.triangle_rigid);
    // for (int i = 0; i < shape_data.sphere_rigid.size(); ++i) {
    //     marchive >> CHNVP(shape_data.capsule_rigid[i].x);
    //     marchive >> CHNVP(shape_data.capsule_rigid[i].y);
    // }
    // ArchiveINVectorReal4(marchive, shape_data.rbox_like_rigid);
    // ArchiveINVectorReal3(marchive, shape_data.convex_rigid);
    // ArchiveINVectorInt(marchive, shape_data.tetrahedron_rigid);
    // ArchiveINVectorReal3(marchive, shape_data.triangle_global);
    // ArchiveINVectorReal3(marchive, shape_data.obj_data_A_global);
    // ArchiveINVectorQuaternion(marchive, shape_data.obj_data_R_global);
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

