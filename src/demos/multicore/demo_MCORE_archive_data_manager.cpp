// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo code about
// - archives for serialization,
// - serialization, with versioning and dynamic creation (class factory)
//
// =============================================================================

#include <typeinfo>
#include <assert.h>

#include "chrono/core/ChLog.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChException.h"
#include "chrono/solver/ChConstraintTuple.h"

#include "chrono/core/ChGlobal.h"
#include "chrono/serialization/ChArchive.h"
#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/serialization/ChArchiveAsciiDump.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include "chrono/serialization/ChArchiveXML.h"
#include "chrono/serialization/ChArchiveExplorer.h"
#include "chrono/serialization/ChBlazeArchive.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_multicore/ChDataManager.h"

using namespace chrono;
using blaze::CompressedMatrix;
using blaze::DynamicVector;
using blaze::Submatrix;
using blaze::Subvector;
using custom_vector;

void Assert_eq(const real3& a, const real3& b) {
    assert(a.x == b.x);
    assert(a.y == b.y);
    assert(a.z == b.z);
}

void Assert_eq(const real4& a, const real4& b) {
    assert(a.w == b.w);
    assert(a.x == b.x);
    assert(a.y == b.y);
    assert(a.z == b.z);
}

void Assert_eq(const uvec4& a, const uvec4& b) {
    assert(a.x == b.x);
    assert(a.y == b.y);
    assert(a.z == b.z);
    assert(a.w == b.w);
}

void Assert_near(const quaternion& a, const quaternion& b, real COMPARE_EPS = C_EPSILON) {
    assert(a.w == b.w);
    assert(a.x == b.x);
    assert(a.y == b.y);
    assert(a.z == b.z);
}

//
// Example on how to serialize OUT some data:
//

void my_serialization_example(ChArchiveOut& marchive)
{
    ChMulticoreDataManager* dataManager = new ChMulticoreDataManager();

    dataManager->shape_data.fam_rigid.push_back(S2(2,3));
    dataManager->shape_data.fam_rigid.push_back(S2(45,90));

    dataManager->shape_data.typ_rigid.push_back(3);
    dataManager->shape_data.typ_rigid.push_back(5);

    dataManager->shape_data.ObA_rigid.push_back(Set3(2,4,12));
    dataManager->shape_data.ObA_rigid.push_back(Set3(55, 44, 33));

    dataManager->shape_data.rbox_like_rigid.push_back(Set4(4,8,12,16));
    dataManager->shape_data.rbox_like_rigid.push_back(Set4(8,12,16,20));

    dataManager->shape_data.sphere_rigid.push_back(2.4);
    dataManager->shape_data.sphere_rigid.push_back(5.4);

    dataManager->shape_data.obj_data_R_global.push_back(SetQ(1,1,1,0));
    dataManager->shape_data.obj_data_R_global.push_back(SetQ(3.14));

    dataManager->host_data.pair_shapeIDs.push_back((long long)4023);

    dataManager->host_data.tet_indices.push_back(U4(5,10,15,20));

    dataManager->host_data.shaft_active.push_back('c');
    dataManager->host_data.shaft_active.push_back('o');
    dataManager->host_data.shaft_active.push_back('o');
    dataManager->host_data.shaft_active.push_back('l');

    dataManager->host_data.cohesion.push_back(1.4f);
    dataManager->host_data.cohesion.push_back(-0.5f);
    dataManager->num_rigid_bodies = 1204;
    dataManager->node_container->max_velocity = 56.2;
    dataManager->fea_container->max_velocity = -24.2;
    CompressedMatrix<real> cm;
    cm.clear(); cm.resize(4,5);
    cm(1,4) = 4; cm(2,3) = .5; cm(3,4) = -23;
    dataManager->host_data.Nshur = cm;

    DynamicVector<real> dv(5UL, 0);
    dv[1] = 2.4; dv[4] = -14.243434;
    dataManager->host_data.R_full = dv;

    dataManager->host_data.bin_num_contact.push_back(45);

    marchive << CHNVP(dataManager);
    delete dataManager;
}

//
// Example on how to deserialize IN some data:
//

void my_deserialization_example(ChArchiveIn& marchive)
{
    ChMulticoreDataManager *dataManager = 0;

    marchive >> CHNVP(dataManager);
    // custom_vector<short2>
    assert(dataManager->shape_data.fam_rigid[0].x == 2);
    assert(dataManager->shape_data.fam_rigid[0].y == 3);
    assert(dataManager->shape_data.fam_rigid[1].x == 45);
    assert(dataManager->shape_data.fam_rigid[1].y == 90);
    // custom_vector<int>
    assert(dataManager->shape_data.typ_rigid[0] == 3);
    assert(dataManager->shape_data.typ_rigid[1] == 5);
    // custom_vector<real3>
    Assert_eq(dataManager->shape_data.ObA_rigid[0], Set3(2,4,12));
    Assert_eq(dataManager->shape_data.ObA_rigid[1], Set3(55, 44, 33));
    // custom_vector<real4>
    Assert_eq(dataManager->shape_data.rbox_like_rigid[0], Set4(4,8,12,16));
    Assert_eq(dataManager->shape_data.rbox_like_rigid[1], Set4(8,12,16,20));
    // custom_vector<real>
    assert(dataManager->shape_data.sphere_rigid[0] == 2.4);
    assert(dataManager->shape_data.sphere_rigid[1] == 5.4);
    // custom_vector<quaternion>
    Assert_near(dataManager->shape_data.obj_data_R_global[0], quaternion(1,1,1,0), 1e-6);
    Assert_near(dataManager->shape_data.obj_data_R_global[1], quaternion(3.14), 1e-6);
    // custom_vector<long long>
    assert(dataManager->host_data.pair_shapeIDs[0] == 4023);
    // custom_vector<uvec4>
    Assert_eq(dataManager->host_data.tet_indices[0], U4(5,10,15,20));
    // custom_vector<char>
    assert(dataManager->host_data.shaft_active[0] == 'c');
    assert(dataManager->host_data.shaft_active[1] == 'o');
    assert(dataManager->host_data.shaft_active[2] == 'o');
    assert(dataManager->host_data.shaft_active[3] == 'l');
    // custom_vector<float>
    assert(dataManager->host_data.cohesion[0] == 1.4f);
    assert(dataManager->host_data.cohesion[1] == -0.5f);
    // uint
    assert(dataManager->num_rigid_bodies == 1204);
    // std::shared_ptr<Ch3DOFContainer>.real
    assert(dataManager->node_container->max_velocity == 56.2);
    assert(dataManager->fea_container->max_velocity == -24.2);
    // Blaze::CompressedMatrix<real>
    assert(dataManager->host_data.Nshur(0,0) == 0);
    assert(dataManager->host_data.Nshur(1,4) == 4);
    assert(dataManager->host_data.Nshur(2,3) == 0.5);
    assert(dataManager->host_data.Nshur(3,4) == -23);
    // Blaze::DynamicVector<real>
    assert(dataManager->host_data.R_full[0] == 0);
    assert(dataManager->host_data.R_full[1] == 2.4);
    assert(dataManager->host_data.R_full[2] == 0);
    assert(dataManager->host_data.R_full[3] == 0);
    assert(dataManager->host_data.R_full[4] == -14.243434);
    // custom_vector<uint>
    assert(dataManager->host_data.bin_num_contact[0] == 45);
    // GetLog() << "num_rigid_bodies: " << dataManager->num_rigid_bodies << "\n";
    // GetLog() << "node_container max vel: " << dataManager->node_container->max_velocity << "\n";
    // GetLog() << "fea_container max vel: " << dataManager->fea_container->max_velocity << "\n";
    // GetLog() << "fam_rigid: \n" << dataManager->shape_data.fam_rigid << "\n";
    // GetLog() << "typ_rigid: \n" << dataManager->shape_data.typ_rigid << "\n";
    // GetLog() << "ObA_rigid: \n" << dataManager->shape_data.ObA_rigid << "\n";
    // GetLog() << "sphere_rigid: \n" << dataManager->shape_data.sphere_rigid << "\n";
    // GetLog() << "rbox_like_rigid: \n" << dataManager->shape_data.rbox_like_rigid << "\n";
    // GetLog() << "obj_data_R_global: \n" << dataManager->shape_data.obj_data_R_global << "\n";
    // GetLog() << "pair_shapeIDs: \n" << dataManager->host_data.pair_shapeIDs << "\n";
    // GetLog() << "tet_indices: \n" << dataManager->host_data.tet_indices << "\n";
    // GetLog() << "shaft_active: \n" << dataManager->host_data.shaft_active << "\n";
    // GetLog() << "cohesion: \n" << dataManager->host_data.cohesion << "\n";
    // GetLog() << "Nshur: ";
    // for (int i = 0; i < dataManager->host_data.Nshur.rows(); ++i) {
    //     GetLog() << "\n";
    //     for (int j = 0; j < dataManager->host_data.Nshur.columns(); ++j) {
    //         real r = dataManager->host_data.Nshur(i, j);
    //         GetLog() << r;
    //         if (j < dataManager->host_data.Nshur.columns() - 1) {
    //             GetLog() << ", ";
    //         }
    //     }
    // }
    // GetLog() << "\n\n";
    // GetLog() << "R_full: \n";
    // for (int i = 0; i < dataManager->host_data.R_full.size(); ++i) {
    //     real r = dataManager->host_data.R_full[i];
    //     GetLog() << r << ",";
    // }
    // GetLog() << "\n\n";
    delete dataManager;
}


int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    GetLog() << "CHRONO foundation classes demo: archives (serialization)\n\n";

    // Create (if needed) output directory
    const std::string out_dir = GetChronoOutputPath() + "DEMO_ARCHIVE";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    //  Archives inherited from the base class ChArchiveOut can be
    // used to serialize objects, and streams inherited from ChArchiveIn
    // can be used to get them back. For example, file streams like
    // ChArchiveOutBinary and ChArchiveInBinary can be used for this
    // purpose.

    try {
        //
        // Example: SERIALIZE TO/FROM BINARY:
        //
        {
            std::string binfile = out_dir + "/foo_archive.dat";
            ChStreamOutBinaryFile mfileo(binfile.c_str());

            // Use a binary archive object to serialize C++ objects into the binary file
            ChArchiveOutBinary marchiveout(mfileo);

            my_serialization_example(marchiveout);
        }

        {
            std::string binfile = out_dir + "/foo_archive.dat";
            ChStreamInBinaryFile mfilei(binfile.c_str());

            // Use a binary archive object to deserialize C++ objects from the binary file
            ChArchiveInBinary marchivein(mfilei);

            my_deserialization_example(marchivein);
        }
        GetLog() << "Serialization test ended with success.\n\n";

    } catch (ChException myex) {
        GetLog() << "ERROR: " << myex.what() << "\n\n";
    }

    return 0;
}
