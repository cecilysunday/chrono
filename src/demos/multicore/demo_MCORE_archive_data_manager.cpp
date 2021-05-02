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
#include "chrono_multicore/constraints/ChConstraintRigidRigid.h"
#include "chrono_multicore/physics/ChSystemMulticore.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

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
using namespace chrono::collision;
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

    CompressedMatrix<real> cm;
    cm.clear(); cm.resize(4,5);
    cm(1,4) = 4; cm(2,3) = .5; cm(3,4) = -23;
    dataManager->host_data.Nshur = cm;

    DynamicVector<real> dv(5UL, 0);
    dv[1] = 2.4; dv[4] = -14.243434;
    dataManager->host_data.R_full = dv;

    dataManager->host_data.bin_num_contact.push_back(45);

    dataManager->system_descriptor = std::make_shared<ChSystemDescriptor>();
    dataManager->system_descriptor->SetMassFactor(2.701);

    dataManager->node_container->max_velocity = 56.2;
    dataManager->fea_container->max_velocity = -24.2;

    dataManager->rigid_rigid = new ChConstraintRigidRigid();
    dataManager->rigid_rigid->Setup(dataManager);

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
    // system_descriptor
    assert(dataManager->system_descriptor->GetMassFactor() == 2.701);
    // std::shared_ptr<Ch3DOFContainer>.real
    assert(dataManager->node_container->max_velocity == 56.2);
    assert(dataManager->fea_container->max_velocity == -24.2);
    // ChConstraintRigidRigid*
    assert(dataManager->rigid_rigid->offset == 3);

    delete dataManager;
}


// -----------------------------------------------------------------------------
// Create the falling spherical objects in a uniform rectangular grid.
// -----------------------------------------------------------------------------
void AddFallingBalls(ChSystemMulticore* sys) {
    // Common material
    auto ballMat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    ballMat->SetYoungModulus(2e6f);
    ballMat->SetFriction(0.4f);
    ballMat->SetRestitution(0.4f);
    ballMat->SetAdhesion(0);

    // Create the falling balls
    int ballId = 0;
    double mass = 1;
    double radius = 0.15;
    ChVector<> inertia = (2.0 / 5.0) * mass * radius * radius * ChVector<>(1, 1, 1);

    for (int ix = -2; ix <= 2; ix++) {
        for (int iy = -2; iy <= 2; iy++) {
            ChVector<> pos(0.4 * ix, 0.4 * iy, 1);

            auto ball = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelMulticore>());
            ball->SetIdentifier(ballId++);
            ball->SetMass(mass);
            ball->SetInertiaXX(inertia);
            ball->SetPos(pos);
            ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
            ball->SetBodyFixed(false);
            ball->SetCollide(true);

            ball->GetCollisionModel()->ClearModel();
            utils::AddSphereGeometry(ball.get(), ballMat, radius);
            ball->GetCollisionModel()->BuildModel();

            sys->AddBody(ball);
        }
    }
}

// -----------------------------------------------------------------------------
// Create a bin consisting of five boxes attached to the ground.
// -----------------------------------------------------------------------------
void AddContainer(ChSystemMulticoreSMC* sys) {
    // IDs for the two bodies
    int binId = -200;

    // Create a common material
    auto mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mat->SetYoungModulus(2e6f);
    mat->SetFriction(0.4f);
    mat->SetRestitution(0.4f);

    // Create the containing bin (4 x 4 x 1)
    auto bin = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelMulticore>());
    bin->SetIdentifier(binId);
    bin->SetMass(1);
    bin->SetPos(ChVector<>(0, 0, 0));
    bin->SetRot(Q_from_AngY(1 * CH_C_PI / 20));
    bin->SetCollide(true);
    bin->SetBodyFixed(true);

    ChVector<> hdim(2, 2, 0.5);
    double hthick = 0.1;

    bin->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(hdim.x(), hdim.y(), hthick), ChVector<>(0, 0, -hthick));
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(hthick, hdim.y(), hdim.z()),
                          ChVector<>(-hdim.x() - hthick, 0, hdim.z()));
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(hthick, hdim.y(), hdim.z()),
                          ChVector<>(hdim.x() + hthick, 0, hdim.z()));
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(hdim.x(), hthick, hdim.z()),
                          ChVector<>(0, -hdim.y() - hthick, hdim.z()));
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(hdim.x(), hthick, hdim.z()),
                          ChVector<>(0, hdim.y() + hthick, hdim.z()));
    bin->GetCollisionModel()->BuildModel();

    sys->AddBody(bin);
}

// Use a binary archive object to serialize (export) C++ objects into the binary file
int ExportSystem(const ChSystemMulticoreSMC &msystem, const std::string &path) {
	const std::string bin_name = path + "/state_export.bin";
	ChStreamOutBinaryFile mfileo(bin_name.c_str());

	ChArchiveOutBinary marchive(mfileo);
	marchive << CHNVP(msystem.data_manager);

	return 0;
}


// Use a binary archive object to serialize (import) C++ objects into the system
int ImportSystem(const ChSystemMulticoreSMC &msystem, const std::string &path) {
	const std::string bin_name = path + "/state_export.bin";
	ChStreamInBinaryFile mfileo(bin_name.c_str());

	ChArchiveInBinary marchive(mfileo);
	marchive >> CHNVP(msystem.data_manager);

	return 0;
}

// -----------------------------------------------------------------------------
// Create the system, specify simulation parameters, and run simulation loop.
// -----------------------------------------------------------------------------
int SetupSystem(ChSystemMulticoreSMC* msystem) {
    // System parameters
	double gravity = 9.81;

    uint max_iteration = 100;
    real tolerance = 1e-3;


    // Set number of threads
    msystem->SetNumThreads(8);

    // Set gravitational acceleration
    msystem->Set_G_acc(ChVector<>(0, 0, -gravity));

    // Set solver parameters
    msystem->GetSettings()->solver.max_iteration_bilateral = max_iteration;
    msystem->GetSettings()->solver.tolerance = tolerance;

    msystem->GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;
    msystem->GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

	// Create the container and spheres
	AddContainer(msystem);
    AddFallingBalls(msystem);

    return 0;
}

void RunSim(ChSystemMulticoreSMC *msystem)
{
	double time_step = 1e-3;
	// Run simulation
	double time_end = 0.1;
	int num_steps = (int)std::ceil(time_end / time_step);
	double time = 0;

	for (int i = 0; i < num_steps; i++) {
		msystem->DoStepDynamics(time_step);
		time += time_step;
	}
}

void my_serialization_example2(ChArchiveOut& marchive)
{
	ChSystemMulticoreSMC system;
	SetupSystem(&system);
	RunSim(&system);

	marchive << CHNVP(system.data_manager);
}

void my_deserialization_example2(ChArchiveIn& marchive)
{
	ChSystemMulticoreSMC system;
	SetupSystem(&system);

	marchive >> CHNVP(system.data_manager);
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

            my_serialization_example2(marchiveout);
        }

        {
            //std::string binfile = out_dir + "/foo_archive.dat";
            //ChStreamInBinaryFile mfilei(binfile.c_str());

            // Use a binary archive object to deserialize C++ objects from the binary file
            //ChArchiveInBinary marchivein(mfilei);

            //my_deserialization_example2(marchivein);
        }
        GetLog() << "Serialization test ended with success.\n\n";

    } catch (ChException myex) {
        GetLog() << "ERROR: " << myex.what() << "\n\n";
    }

    return 0;
}
