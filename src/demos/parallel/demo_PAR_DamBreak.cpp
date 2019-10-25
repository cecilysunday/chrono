// ============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Milad Rakhsha
// =============================================================================
// =============================================================================

//#include <blaze/math/DynamicVector.h>
#include <stdio.h>
#include <cmath>
#include <fstream>
#include <vector>
#include <time.h>
//#include <sys/time.h>  // for gettimeofday()
#include "chrono_parallel/physics/ChSystemParallel.h"
#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

#include "chrono/ChConfig.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/filesystem/resolver.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/physics/ChLinkLinActuator.h"
#include "chrono_parallel/physics/Ch3DOFContainer.h"

using namespace chrono;
using namespace chrono::collision;

enum ProblemType { SETTLING, VIBRATING };
ProblemType problem = SETTLING;

// -----------------------------------------------------------------------------
// Simulation settings
// -----------------------------------------------------------------------------
double mKernel_radius = 0.05;
double particle_dist = mKernel_radius * 0.5;
double Mass_Constant = 0.13;  // 0.138 for mKernel_radius*.5 and 0.613 for mKernel_radius*.9

double time_step = 1e-3;
int num_iteration = 50;
int threads = 8;
double out_fps = 50;

double time_movePlatform = 20;
double time_end = 5;

double tankFullFilledMass = 1000;

std::shared_ptr<ChFluidContainer> fluidSystem;
std::shared_ptr<ChBody> tank;
double width = 0.2;

ChVector<> hdim(5.3 / 2, width + 1.0 * particle_dist, 3.0 / 2);
ChVector<> fluidDim(1.0 + 0.5 * particle_dist, width + 1.0 * particle_dist, 0.5 + 0.5 * particle_dist);
ChVector<> tankInitPos(hdim.x() - fluidDim.x(), 0.5 * particle_dist, -0.5 + 0.12 * particle_dist);

// ChVector<> hdim(0.5, 0.1, 2);
// ChVector<> fluidDim(hdim.x(), hdim.y() + mKernel_radius / 2, hdim.z() / 4);
// ChVector<> tankInitPos = ChVector<>(0.0, 0.0, -hdim.z() / 4);

int tankId = 0;

// ground
std::shared_ptr<ChBody> ground;
double tank_ground_dist = hdim.z();
ChVector<> groundInitPos = tankInitPos - ChVector<>(0, 0, tank_ground_dist);
int groundID = -2;

// vibration property
double vibration_amp = .0;
double vibration_freq = 0.0001;

const std::string out_dir = "DamBreak";
const std::string demo_dir = out_dir + "/Paraview";
const std::string checkpoint_file_rigid = out_dir + "/settledRigid.dat";
const std::string checkpoint_file_fluid = out_dir + "/settledFluid.dat";
const std::string simSettings = out_dir + "/simSetting.txt";

// -----------------------------------------------------------------------------
// save csv data file
// -----------------------------------------------------------------------------
void WritePovray(ChSystemParallelNSC* msystem, int out_frame) {
    // Write rigid bodies
    //-------------------
    char filename1[100];
    sprintf(filename1, "%s/data_%04d.dat", demo_dir.c_str(), out_frame + 1);
    utils::WriteShapesPovray(msystem, filename1, false);

    // Write fluid bodies
    //-------------------
    char filename2[100];
    sprintf(filename2, "%s/fluid_%04d.csv", demo_dir.c_str(), out_frame + 1);

    const std::string& delim = "";
    const std::string& comma = ",";

    utils::CSV_writer csv(delim);
    custom_vector<real> den;
    custom_vector<real> pres;
    fluidSystem->GetFluidDensity(den);
    fluidSystem->GetFluidPressure(pres);

    std::cout << den.size() << std::endl;
    int numMarkers = msystem->data_manager->host_data.pos_3dof.size();
    csv << "t,x,y,z,vx,vy,vz,|U|,rho,p\n";
    for (int i = 0; i < numMarkers; i++) {
        real3 pos3 = msystem->data_manager->host_data.pos_3dof[i];
        real3 vel3 = msystem->data_manager->host_data.vel_3dof[i];
        real U = sqrt(vel3.x * vel3.x + vel3.y * vel3.y + vel3.z * vel3.z);
        csv << msystem->GetChTime() << comma << pos3.x << comma << pos3.y << comma << pos3.z << comma << vel3.x << comma
            << vel3.y << comma << vel3.z << comma << U << comma << den[i] << comma << pres[i] << std::endl;
    }

    csv.write_to_file(filename2);
}

// -----------------------------------------------------------------------------
// Write Checkpoint
// -----------------------------------------------------------------------------
void WriteCheckPointAll(ChSystemParallelNSC* msystem) {
    // Write Rigid bodies
    //	utils::WriteCheckpoint(msystem, checkpoint_file_rigid);

    // Write fluid bodies
    //-------------------
    const std::string& delim = " ";
    utils::CSV_writer csv(delim);
    int numMarkers = msystem->data_manager->host_data.pos_3dof.size();
    for (int i = 0; i < numMarkers; i++) {
        real3 pos3 = msystem->data_manager->host_data.pos_3dof[i];
        real3 vel3 = msystem->data_manager->host_data.vel_3dof[i];
        csv << pos3.x << pos3.y << pos3.z << vel3.x << vel3.y << vel3.z << std::endl;
    }
    csv.write_to_file(checkpoint_file_fluid);
}

// -----------------------------------------------------------------------------
// Read Checkpoint
// -----------------------------------------------------------------------------
void ReadCheckPointAll() {
    // Read Rigid bodies
    //    utils::ReadCheckpoint(msystem, checkpoint_file_rigid);

    // Read fluid bodies
    //-------------------
    std::vector<real3> pos_fluid;
    std::vector<real3> vel_fluid;

    std::ifstream ifile(checkpoint_file_fluid.c_str());
    std::string line;

    std::getline(ifile, line);
    std::istringstream iss0(line);

    while (std::getline(ifile, line)) {
        std::istringstream iss1(line);

        // Read body type, Id, flags
        real3 bpos, bpos_dt;
        iss1 >> bpos.x >> bpos.y >> bpos.z >> bpos_dt.x >> bpos_dt.y >> bpos_dt.z;
        pos_fluid.push_back(bpos);
        vel_fluid.push_back(bpos_dt);
    }

    fluidSystem->UpdatePosition(0);
    fluidSystem->AddBodies(pos_fluid, vel_fluid);
}

// -----------------------------------------------------------------------------
// Add fluid bucket
// -----------------------------------------------------------------------------
void AddFluidBox(double dist) {
    std::vector<real3> pos_fluid;
    std::vector<real3> vel_fluid;

    //    utils::HCPSampler<> sampler(dist);
    utils::GridSampler<> sampler(dist);

    utils::Generator::PointVector points =
        sampler.SampleBox(ChVector<>(0, 0.0, 0.5 * fluidSystem->kernel_radius) + 0.25 * fluidSystem->kernel_radius,
                          fluidDim - 0.5 * fluidSystem->kernel_radius);

    std::cout << "points size " << points.size() << std::endl;
    pos_fluid.resize(points.size());
    vel_fluid.resize(points.size());
    for (int i = 0; i < points.size(); i++) {
        pos_fluid[i] = real3(points[i].x(), points[i].y(), points[i].z());
        vel_fluid[i] = real3(0, 0, 0);
    }
    fluidSystem->UpdatePosition(0);
    fluidSystem->AddBodies(pos_fluid, vel_fluid);
}

// -----------------------------------------------------------------------------
// Initialize physical system and integration properties
// -----------------------------------------------------------------------------
void InitSystem(ChSystemParallelNSC& msystem) {
    real tolerance = 1e-2;

    omp_set_num_threads(threads);
    // Set number of threads.
    //    int max_threads = 2;//CHOMPfunctions::GetNumProcs();
    //    if (threads > max_threads)
    //        threads = max_threads;
    msystem.SetParallelThreadNumber(threads);
    CHOMPfunctions::SetNumThreads(threads);

    // Set gravitational acceleration
    double gravity = 9.81;
    msystem.Set_G_acc(ChVector<>(0, 0, -gravity));

    // Set solver parameters
    msystem.GetSettings()->solver.solver_mode = SolverMode::NORMAL;
    msystem.GetSettings()->solver.max_iteration_normal = num_iteration;
    msystem.GetSettings()->solver.max_iteration_sliding = 0;
    msystem.GetSettings()->solver.max_iteration_spinning = 0;
    msystem.GetSettings()->solver.max_iteration_bilateral = num_iteration;
    msystem.GetSettings()->solver.tolerance = tolerance;
    msystem.GetSettings()->solver.alpha = 0;
    msystem.GetSettings()->solver.use_full_inertia_tensor = false;
    msystem.GetSettings()->solver.contact_recovery_speed = 2;
    msystem.GetSettings()->solver.cache_step_length = true;

    msystem.ChangeSolverType(SolverType::APGDREF);
    msystem.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;
    msystem.GetSettings()->collision.bins_per_axis = vec3(2, 2, 2);
}
// -----------------------------------------------------------------------------
// Initialize fluid system and integration properties
// -----------------------------------------------------------------------------
void InitFluidSystem(ChSystemParallelNSC* sys) {
    fluidSystem = std::make_shared<ChFluidContainer>();

    fluidSystem->contact_cohesion = 0;
    fluidSystem->contact_mu = 0.0;
    fluidSystem->epsilon = 1e-8;
    fluidSystem->viscosity = 0.0;
    fluidSystem->enable_viscosity = false;
    fluidSystem->tau = time_step * 2;

    // msystem.GetSettings()->fluid.max_interactions = 30;
    fluidSystem->artificial_pressure = false;
    fluidSystem->artificial_pressure_k = .01;
    fluidSystem->collision_envelope = 0;  // fluidSystem->kernel_radius * .05;

    fluidSystem->artificial_pressure_dq = .2 * fluidSystem->kernel_radius;
    fluidSystem->artificial_pressure_n = 4;

    // Variable Settings
    // -----------------
    fluidSystem->rho = 1000;
    fluidSystem->kernel_radius = mKernel_radius;
    fluidSystem->contact_recovery_speed = 5;
    fluidSystem->max_velocity = 1000;
    sys->Add3DOFContainer(fluidSystem);
    sys->GetSettings()->collision.collision_envelope = (fluidSystem->kernel_radius * .05);
}

// -----------------------------------------------------------------------------
// Create a bin consisting of five boxes attached. The bin is constrained
// to by a prismatic joint
// -----------------------------------------------------------------------------
void AddTank(ChSystemParallelNSC* sys) {
    // Create a common material
    auto mat = std::make_shared<ChMaterialSurfaceNSC>();
    mat->SetFriction(0.4f);

    tank = utils::CreateBoxContainer(sys, tankId, mat, hdim + ChVector<>(0.0, 0.0, 0.0), particle_dist, tankInitPos,
                                     QUNIT, true, false, true, true);
    tank->SetMass(0.2 * tankFullFilledMass);
}

// -----------------------------------------------------------------------------
// Add prismatic joint between container and ground
// -----------------------------------------------------------------------------
void ConstrainTank(ChSystemParallelNSC* sys) {
    // Create ground
    auto mat = std::make_shared<ChMaterialSurfaceNSC>();
    mat->SetFriction(0.4f);

    auto ground = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>());
    ground->SetMaterialSurface(mat);
    ground->SetIdentifier(groundID);
    ground->SetMass(1);
    ground->SetPos(groundInitPos);
    ground->SetRot(QUNIT);
    ground->SetCollide(false);
    ground->SetBodyFixed(true);

    ground->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(ground.get(), ChVector<>(hdim.x(), hdim.y(), .05));
    ground->GetCollisionModel()->BuildModel();

    sys->AddBody(ground);

    // Add prismatic joint
    // ------------------
    tank->SetBodyFixed(false);
    auto platformPrismaticJoint = std::make_shared<ChLinkLockPrismatic>();
    platformPrismaticJoint->Initialize(tank, ground, ChCoordsys<>(tank->GetPos(), Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
    sys->AddLink(platformPrismaticJoint);
}

// -----------------------------------------------------------------------------
// Vibrate tank
// -----------------------------------------------------------------------------
void VibrateTank(double t) {
    double vibration_w = 2 * CH_C_PI * vibration_freq;  // 30 Hz vibration similar to Gravish 2012, PRL
    double x_bucket = vibration_amp * sin(vibration_w * t);
    double xDot_bucket = vibration_amp * vibration_w * cos(vibration_w * t);
    double xDDot_bucket = vibration_amp * vibration_w * vibration_w * -1 * sin(vibration_w * t);
    double X_CORRECTION = hdim.x() - hdim.x() / 2;
    ChVector<> tankNewPos = tankInitPos + ChVector<>(x_bucket + X_CORRECTION, 0, 0);
    tank->SetPos(tankNewPos);
    tank->SetPos_dt(ChVector<>(xDot_bucket, 0, 0));
    tank->SetPos_dtdt(ChVector<>(xDDot_bucket, 0, 0));
}

// -----------------------------------------------------------------------------
// Create the fluid in the shape of a sphere.
// -----------------------------------------------------------------------------
void AddFluid(ChSystemParallelNSC* sys) {
    double dist = particle_dist;
    real vol = pow(mKernel_radius, 3) * Mass_Constant;
    fluidSystem->mass = fluidSystem->rho * vol;

    if (problem == SETTLING) {
        AddFluidBox(dist);
    } else {
        ReadCheckPointAll();
    }

    std::ofstream settingsFile(simSettings.c_str(), std::ios::app);
    settingsFile << "Fluid system info: "
                 << "\n";
    settingsFile << "  number of markers: " << sys->data_manager->host_data.pos_3dof.size() << "\n"
                 << "  marker mass: " << fluidSystem->mass << "\n"
                 << "  total mass: " << fluidSystem->mass * sys->data_manager->host_data.pos_3dof.size() << "\n"
                 << "\n\n";
    settingsFile.flush();
    settingsFile.close();

    std::cout << "AddFluid size=" << fluidSystem->density.size() << std::endl;
}

// -----------------------------------------------------------------------------
// Find the height of the highest sphere in the granular mix that is
// within 'radius' distance of the 'ctr' point.
// We only look at bodies whith stricty positive
// identifiers (to exclude the containing bin).
// -----------------------------------------------------------------------------
double FindFluidHeight(ChSystemParallelNSC* sys, ChVector<> fluidWindowHdim) {
    double highest = -1000;
    for (size_t i = 0; i < sys->data_manager->host_data.pos_3dof.size(); ++i) {
        real3 pos3 = sys->data_manager->host_data.pos_3dof[i];
        real3 rDist3 = pos3 - real3(tankInitPos.x(), tankInitPos.y(), tankInitPos.z());
        if ((fabs(rDist3.x) < fluidWindowHdim.x() && fabs(rDist3.y) < fluidWindowHdim.y()) && pos3.z > highest) {
            highest = pos3.z;
        }
    }
    std::ofstream settingsFile(simSettings.c_str(), std::ios::app);
    settingsFile << "Fluid system info: "
                 << "\n";
    settingsFile << "  number of markers: " << sys->data_manager->host_data.pos_3dof.size() << "\n"
                 << "  marker mass: " << fluidSystem->mass << "\n"
                 << "\n\n";
    settingsFile.flush();
    settingsFile.close();

    double fluidH = highest - tankInitPos.z();

    const std::string fluidHeightName = out_dir + "/fluidHeight.txt";
    std::ofstream fluidHeight(fluidHeightName.c_str());
    fluidHeight << "Fluid Height: " << fluidH << "\n";
    fluidHeight.flush();
    fluidHeight.close();

    return fluidH;
}

// =============================================================================
// Set arguments from input
// -----------------------------------------------------------------------------
void SetArgumentsForMbdFromInput(int argc, char* argv[]) {
    int pType = 0;
    if (argc > 1) {
        const char* text = argv[1];
        threads = atoi(text);
    }
    if (argc > 2) {
        const char* text = argv[2];
        time_step = atof(text);
    }
    if (argc > 3) {
        const char* text = argv[3];
        pType = atoi(text);
    }
    if (argc > 4) {
        const char* text = argv[4];
        vibration_amp = atof(text);
    }
    if (argc > 5) {
        const char* text = argv[5];
        vibration_freq = atof(text);
    }
    if (argc > 6) {
        const char* text = argv[6];
        num_iteration = atoi(text);
    }

    if (pType == 0) {
        problem = SETTLING;
    } else if (pType == 1) {
        problem = VIBRATING;
    } else {
        std::runtime_error("Error! problem type can be either 0 (settling) or 1 (vibrating) \n");
    }

    std::ofstream settingsFile(simSettings.c_str());
    settingsFile << "General Settings, set from input: "
                 << "\n";
    settingsFile << "  num threads: " << threads << "\n"
                 << "  time_step: " << time_step << "\n"
                 << "  pType (0: SETTLING, 1: VIBRATING): " << pType << "\n"
                 << "  vibration_amp: " << vibration_amp << "\n"
                 << "  vibration_freq: " << vibration_freq << "\n"
                 << "  num_iteration: " << num_iteration << "\n"
                 << "\n"
                 << "\n\n";
    settingsFile.flush();
    settingsFile.close();
}

// -----------------------------------------------------------------------------
// Create the system, specify simulation parameters, and run simulation loop.
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    printf("Tank pos: %f,%f,%f\n", tankInitPos.x(), tankInitPos.y(), tankInitPos.z());
    // Create output directories.
    // --------------------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(demo_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Get problem parameters from arguments
    // -------------------------------------
    SetArgumentsForMbdFromInput(argc, argv);

    // Simulation parameters
    // ---------------------
    bool povray_output = true;

    // Number of steps
    int num_steps = std::ceil(time_end / time_step);
    int out_steps = std::ceil((1.0 / time_step) / out_fps);

    // Create system
    // -------------

    ChSystemParallelNSC msystem;
    InitSystem(msystem);
    InitFluidSystem(&msystem);
    AddFluid(&msystem);

    AddTank(&msystem);
    tank->SetBodyFixed(true);

    // Perform the simulation
    // ----------------------

    double time = 0;
    int sim_frame = 0;
    int out_frame = 0;
    int next_out_frame = 0;

    bool moving = false;
    // Run simulation for specified time

    const std::string rmCmd = (std::string("rm ") + demo_dir + std::string("/*"));
    system(rmCmd.c_str());

#if 0
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "ballsDVI", &msystem);
    gl_window.SetCamera(ChVector<>(0, -10, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
    gl_window.Pause();
    // Uncomment the following two lines for the OpenGL manager to automatically
    // run the simulation in an infinite loop.
    // gl_window.StartDrawLoop(time_step);
    // return 0;

    while (true) {
        if (gl_window.Active()) {
            gl_window.DoStepDynamics(time_step);
            gl_window.Render();
            //            msystem.CalculateContactForces();
            //            real3 frc = msystem.GetBodyContactForce(0);
            //            std::cout << frc.x << "  " << frc.y << "  " << frc.z << std::endl;
        }
    }
#else

    // time_t begin = time(NULL);
    //struct timeval start, end;

    //gettimeofday(&start, NULL);

    const std::string tankPosForce = out_dir + "/posForce.csv";
    std::ofstream outp(tankPosForce.c_str());
    while (time < time_end) {
        fluidSystem->CalculateContactForces();
        real3 force = fluidSystem->GetBodyContactForce(tank->GetId());
        outp << time << ", " << tank->GetPos().x() << ", " << tank->GetPos().y() << ", " << tank->GetPos().z() << ", "
             << force.x << ", " << force.y << ", " << force.z << std::endl;
        if ((time > time_movePlatform || problem == VIBRATING) && !moving) {
            moving = true;
            ConstrainTank(&msystem);
            (void)FindFluidHeight(&msystem, 0.5 * hdim);
        }

        if (moving) {
            double t = (problem == VIBRATING) ? time : time - time_movePlatform;
            VibrateTank(t);
        }

        msystem.DoStepDynamics(time_step);

        printf("time %f time_end %f \n", time, time_end);
        // If enabled, output data for PovRay postprocessing.
        if (sim_frame == next_out_frame) {
            if (povray_output) {
                WritePovray(&msystem, out_frame);
            }

            // Create a checkpoint from the current state.
            if (problem == SETTLING && time < time_movePlatform) {
                WriteCheckPointAll(&msystem);
            }
            out_frame++;
            next_out_frame += out_steps;
        }
        time += time_step;
        sim_frame++;
    }
    // time_t end = time(NULL);
    //gettimeofday(&end, NULL);

    //long seconds = (end.tv_sec - start.tv_sec);

    //printf("Finished in %d\n", seconds);
    // Create a checkpoint from the last state
    std::cout << "  done simulating " << msystem.Get_bodylist().size() << " bodies." << std::endl;

#endif
    return 0;
}
