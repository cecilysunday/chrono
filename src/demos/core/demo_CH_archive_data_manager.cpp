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


#include "chrono/core/ChGlobal.h"

#include "chrono/core/ChLog.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChException.h"
#include "chrono/solver/ChConstraintTuple.h"

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

    dataManager->ArchiveOut(marchive);
    delete dataManager;
}


//
// Example on how to deserialize IN some data:
//

void my_deserialization_example(ChArchiveIn& marchive)
{
    // ChMulticoreDataManager* dataManager = new ChMulticoreDataManager();
    // marchive >> CHNVP(dataManager);
    // GetLog() << "\n"
    //     << "num_rigid_bodies: " << dataManager->num_rigid_bodies << "\n"
    //     << "num_fluid_bodies: " << dataManager->num_fluid_bodies << "\n"
    //     << "host_data: " << "\n";
    // for (int i = 0; i < dataManager->shape_data.typ_rigid.size(); ++i) {
    //     GetLog() << "  typ_rigid[" << i << "]: "
    //     << dataManager->shape_data.typ_rigid[i] << "\n";
    // }
    ChMulticoreDataManager *dataManager = new ChMulticoreDataManager();
    dataManager->ArchiveIn(marchive);

    GetLog() << "num_rigid_bodies: " << dataManager->num_rigid_bodies << "\n";
    GetLog() << "node_container max vel: " << dataManager->node_container->max_velocity << "\n";
    GetLog() << "fea_container max vel: " << dataManager->fea_container->max_velocity << "\n";
    GetLog() << "fam_rigid: \n" << dataManager->shape_data.fam_rigid << "\n";
    GetLog() << "typ_rigid: \n" << dataManager->shape_data.typ_rigid << "\n";
    GetLog() << "ObA_rigid: \n" << dataManager->shape_data.ObA_rigid << "\n";
    GetLog() << "sphere_rigid: \n" << dataManager->shape_data.sphere_rigid << "\n";
    GetLog() << "rbox_like_rigid: \n" << dataManager->shape_data.rbox_like_rigid << "\n";
    GetLog() << "obj_data_R_global: \n" << dataManager->shape_data.obj_data_R_global << "\n";
    GetLog() << "pair_shapeIDs: \n" << dataManager->host_data.pair_shapeIDs << "\n";
    GetLog() << "tet_indices: \n" << dataManager->host_data.tet_indices << "\n";
    GetLog() << "shaft_active: \n" << dataManager->host_data.shaft_active << "\n";
    GetLog() << "cohesion: \n" << dataManager->host_data.cohesion << "\n";
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
