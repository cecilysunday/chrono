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
// Authors: Radu Serban
// =============================================================================
//
// Implementation of a single-tire test rig.
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/test_rig/MMXTireTestRig.h"

#include "chrono/assets/ChAssetLevel.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono_vehicle/terrain/MMXTerrain.h"

namespace chrono {
namespace vehicle {

MMXTireTestRig::MMXTireTestRig(std::shared_ptr<ChWheel> wheel, std::shared_ptr<ChTire> tire, ChSystem* system)
    : m_system(system),
      m_wheel(wheel),
      m_tire(tire),
      m_camber_angle(0),
      m_normal_load(0),
      m_applied_load(0),
      m_total_mass(0),
      m_time_delay(0),
      m_ls_actuated(false),
      m_rs_actuated(false),
      m_terrain_type(TerrainType::NONE),
      m_rig_voffset(0),
      m_rig_hoffset(0),
      m_tire_step(1e-3),
      m_tire_vis(VisualizationType::PRIMITIVES) {
	 
	  // Default motion function for slip angle control
      m_sa_fun = chrono_types::make_shared<ChFunction_Const>(0);
      
	  // Default tire-terrain collision method
      m_tire->SetCollisionType(ChTire::CollisionType::SINGLE_POINT);
}

// -----------------------------------------------------------------------------

void MMXTireTestRig::SetLongSpeedFunction(std::shared_ptr<ChFunction> funct) {
    m_ls_fun = funct;
    m_ls_actuated = true;
}

void MMXTireTestRig::SetAngSpeedFunction(std::shared_ptr<ChFunction> funct) {
    m_rs_fun = funct;
    m_rs_actuated = true;
}

void MMXTireTestRig::SetTireCollisionType(ChTire::CollisionType coll_type) {
    m_tire->SetCollisionType(coll_type);
}

// -----------------------------------------------------------------------------

void MMXTireTestRig::SetTerrainMMX(std::shared_ptr<ChMaterialSurfaceSMC> mat_g,
                                   std::shared_ptr<ChMaterialSurfaceSMC> mat_w,
                                   double length,
								   double width,
								   double height,
								   double radius,
                                   double density) {

    m_terrain_type = TerrainType::MMX;

	m_params_mmx.mat_g = mat_g;
    m_params_mmx.mat_w = mat_w;
    m_params_mmx.length = length;
    m_params_mmx.width = width;
    m_params_mmx.height = height;
    m_params_mmx.radius = radius;
    m_params_mmx.density = density;
}

// -----------------------------------------------------------------------------

void MMXTireTestRig::Initialize() {
    CreateMechanism();

    if (m_ls_actuated)
        m_lin_motor->SetSpeedFunction(m_ls_fun);

    if (m_rs_actuated)
        m_rot_motor->SetSpeedFunction(m_rs_fun);

    m_slip_lock->SetMotion_ang(m_sa_fun);

    CreateTerrain();
}

void MMXTireTestRig::Reinitialize() {
    if (m_ls_actuated)
        m_lin_motor->SetSpeedFunction(m_ls_fun);

    if (m_rs_actuated)
        m_rot_motor->SetSpeedFunction(m_rs_fun);

    m_slip_lock->SetMotion_ang(m_sa_fun);
}

// -----------------------------------------------------------------------------

class BaseFunction {
  protected:
    BaseFunction(double speed) : m_speed(speed) {}
    double calc(double t) const {
        double delay = 0.25;
        double ramp = 0.5;
        if (t <= delay)
            return 0;
        double tt = t - delay;
        if (tt >= ramp)
            return m_speed;
        return m_speed * tt / ramp;
    }
    double m_speed;
};

class LinSpeedFunction : public BaseFunction, public ChFunction {
  public:
    LinSpeedFunction(double speed) : BaseFunction(speed) {}
    virtual double Get_y(double t) const override { return calc(t); }
    virtual LinSpeedFunction* Clone() const override { return new LinSpeedFunction(*this); }
};

class RotSpeedFunction : public BaseFunction, public ChFunction {
  public:
    RotSpeedFunction(double slip, double speed, double radius) : BaseFunction(speed), m_slip(slip), m_radius(radius) {}
    virtual double Get_y(double t) const override {
        double v = calc(t);
        return (1 + m_slip) * v / m_radius;
    }
    virtual RotSpeedFunction* Clone() const override { return new RotSpeedFunction(*this); }

    double m_slip;
    double m_radius;
};

void MMXTireTestRig::Initialize(double long_slip, double base_speed) {
    m_ls_actuated = true;
    m_rs_actuated = true;

    CreateMechanism();

    m_ls_fun = chrono_types::make_shared<LinSpeedFunction>(base_speed);
    m_rs_fun = chrono_types::make_shared<RotSpeedFunction>(long_slip, base_speed, m_tire->GetRadius());

    m_lin_motor->SetSpeedFunction(m_ls_fun);
    m_rot_motor->SetSpeedFunction(m_rs_fun);
    m_slip_lock->SetMotion_ang(m_sa_fun);

    CreateTerrain();
}

// -----------------------------------------------------------------------------

void MMXTireTestRig::Advance(double step) {
    double time = m_system->GetChTime();

    // Apply load on chassis body
    double external_force = m_total_mass * m_system->Get_G_acc().Length();
    if (time > m_time_delay)
        external_force = m_applied_load;

    m_chassis_body->Empty_forces_accumulators();
    m_chassis_body->Accumulate_force(ChVector<>(0, 0, external_force), ChVector<>(0, 0, 0), true);

    // Synchronize subsystems
    m_terrain->Synchronize(time);
    m_tire->Synchronize(time, *m_terrain.get());
    m_spindle_body->Empty_forces_accumulators();
    m_wheel->Synchronize();

    // Advance state
    m_terrain->Advance(step);
    m_tire->Advance(step);
    m_system->DoStepDynamics(step);
}

// -----------------------------------------------------------------------------

void MMXTireTestRig::CreateMechanism() {
    // Create rig bodies
    const double dim = m_tire->GetRadius() / 10.0;
    const double max_length = m_params_mmx.length;

    m_ground_body = std::shared_ptr<ChBody>(m_system->NewBody());
    m_system->AddBody(m_ground_body);
    m_ground_body->SetName("rig_ground");
    m_ground_body->SetIdentifier(-1);
    m_ground_body->SetBodyFixed(true);
    {
        auto box = chrono_types::make_shared<ChBoxShape>();
        box->GetBoxGeometry().SetLengths(ChVector<>(max_length, dim / 3, dim / 3));
        m_ground_body->AddAsset(box);
    }

    m_carrier_body = std::shared_ptr<ChBody>(m_system->NewBody());
    m_system->AddBody(m_carrier_body);
    m_carrier_body->SetName("rig_carrier");
    m_carrier_body->SetIdentifier(-2);
    m_carrier_body->SetPos(ChVector<>(0, 0, 0));
    m_carrier_body->SetMass(m_wheel->GetMass());
    m_carrier_body->SetInertiaXX(m_wheel->GetInertia());
    {
        auto cyl = chrono_types::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().rad = dim / 2;
        cyl->GetCylinderGeometry().p1 = ChVector<>(+2 * dim, 0, 0);
        cyl->GetCylinderGeometry().p2 = ChVector<>(-2 * dim, 0, 0);
        m_carrier_body->AddAsset(cyl);

        auto box = chrono_types::make_shared<ChBoxShape>();
        box->GetBoxGeometry().SetLengths(ChVector<>(dim / 3, dim / 3, 10 * dim));
        box->Pos = ChVector<>(0, 0, -5 * dim);
        m_carrier_body->AddAsset(box);

        m_carrier_body->AddAsset(chrono_types::make_shared<ChColorAsset>(0.8f, 0.2f, 0.2f));
    }

    m_chassis_body = std::shared_ptr<ChBody>(m_system->NewBody());
    m_system->AddBody(m_chassis_body);
    m_chassis_body->SetName("rig_chassis");
    m_chassis_body->SetIdentifier(-3);
    m_chassis_body->SetPos(ChVector<>(0, 0, 0));
    m_chassis_body->SetMass(m_wheel->GetMass());
    m_chassis_body->SetInertiaXX(m_wheel->GetInertia());
    {
        auto sphere = chrono_types::make_shared<ChSphereShape>();
        sphere->GetSphereGeometry().rad = dim;
        m_chassis_body->AddAsset(sphere);

        auto cyl = chrono_types::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().rad = dim / 2;
        cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, 0);
        cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, -2 * dim);
        m_chassis_body->AddAsset(cyl);

        m_chassis_body->AddAsset(chrono_types::make_shared<ChColorAsset>(0.2f, 0.8f, 0.2f));
    }

    m_slip_body = std::shared_ptr<ChBody>(m_system->NewBody());
    m_system->AddBody(m_slip_body);
    m_slip_body->SetName("rig_slip");
    m_slip_body->SetIdentifier(-4);
    m_slip_body->SetPos(ChVector<>(0, 0, -4 * dim));
    m_slip_body->SetMass(m_wheel->GetMass());
    m_slip_body->SetInertiaXX(m_wheel->GetInertia());
    {
        auto box = chrono_types::make_shared<ChBoxShape>();
        box->GetBoxGeometry().SetLengths(ChVector<>(4 * dim, dim, 4 * dim));
        m_slip_body->AddAsset(box);

        m_slip_body->AddAsset(chrono_types::make_shared<ChColorAsset>(0.2f, 0.2f, 0.8f));
    }

    m_spindle_body = std::shared_ptr<ChBody>(m_system->NewBody());
    ChQuaternion<> qc;
    qc.Q_from_AngX(-m_camber_angle);
    m_system->AddBody(m_spindle_body);
    m_spindle_body->SetName("rig_spindle");
    m_spindle_body->SetIdentifier(-5);
    m_spindle_body->SetMass(1);
    m_spindle_body->SetInertiaXX(ChVector<>(0.01, 0.01, 0.01));
    m_spindle_body->SetPos(ChVector<>(0, 3 * dim, -4 * dim));
    m_spindle_body->SetRot(qc);
    {
        auto cyl = chrono_types::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().rad = dim / 2;
        cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, 0);
        cyl->GetCylinderGeometry().p2 = ChVector<>(0, -3 * dim, 0);
        m_spindle_body->AddAsset(cyl);
    }

    // Create joints and motors
    if (m_ls_actuated) {
        m_lin_motor = chrono_types::make_shared<ChLinkMotorLinearSpeed>();
        m_system->AddLink(m_lin_motor);
        m_lin_motor->Initialize(m_carrier_body, m_ground_body, ChFrame<>(ChVector<>(0, 0, 0), QUNIT));
    } else {
        ChQuaternion<> z2x;
        z2x.Q_from_AngY(CH_C_PI_2);
        auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
        m_system->AddLink(prismatic);
        prismatic->Initialize(m_carrier_body, m_ground_body, ChCoordsys<>(VNULL, z2x));
    }

    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    m_system->AddLink(prismatic);
    prismatic->Initialize(m_carrier_body, m_chassis_body, ChCoordsys<>(VNULL, QUNIT));

    m_slip_lock = chrono_types::make_shared<ChLinkLockLock>();
    m_system->AddLink(m_slip_lock);
    m_slip_lock->Initialize(m_chassis_body, m_slip_body, ChCoordsys<>(VNULL, QUNIT));
    m_slip_lock->SetMotion_axis(ChVector<>(0, 0, 1));

    ChQuaternion<> z2y;
    z2y.Q_from_AngAxis(-CH_C_PI / 2 - m_camber_angle, ChVector<>(1, 0, 0));
    if (m_rs_actuated) {
        m_rot_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        m_system->AddLink(m_rot_motor);
        m_rot_motor->Initialize(m_spindle_body, m_slip_body, ChFrame<>(ChVector<>(0, 3 * dim, -4 * dim), z2y));
    } else {
        auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
        m_system->AddLink(revolute);
        revolute->Initialize(m_spindle_body, m_slip_body, ChCoordsys<>(ChVector<>(0, 3 * dim, -4 * dim), z2y));
    }

    // Calculate required body force on chassis to enforce given normal load
    m_total_mass = m_chassis_body->GetMass() + m_slip_body->GetMass() + m_spindle_body->GetMass() + m_wheel->GetMass() +
                   m_tire->GetMass();
    m_applied_load = m_total_mass * m_system->Get_G_acc().Length() - m_normal_load;

    // Initialize subsystems
    m_wheel->Initialize(m_spindle_body, LEFT);
    m_wheel->SetVisualizationType(VisualizationType::NONE);
    m_wheel->SetTire(m_tire);
    m_tire->SetStepsize(m_tire_step);
    m_tire->Initialize(m_wheel);
    m_tire->SetVisualizationType(m_tire_vis);

    // Set the rig offset based on wheel center
    m_rig_hoffset = 3 * dim; //+ m_tire->GetWidth() / 2.0;
    m_rig_voffset = 4 * dim + m_tire->GetRadius();
}

// -----------------------------------------------------------------------------

void MMXTireTestRig::CreateTerrain() {
    switch (m_terrain_type) {
        case TerrainType::MMX:
            CreateTerrainMMX();
            break;
        default:
            break;
    }
}

void MMXTireTestRig::CreateTerrainMMX() {

	// 3) Set wall and particle material properties


    double terrain_voffset = m_params_mmx.height + 2.0 * m_params_mmx.radius;
    ChVector<> location = ChVector<>(0, m_rig_hoffset, -m_rig_voffset - terrain_voffset);

    auto terrain = chrono_types::make_shared<vehicle::MMXTerrain>(m_system);
    terrain->SetStartIdentifier(0);
	
	terrain->SetContactMaterial(m_params_mmx.mat_g);
    
	terrain->EnableVisualization(false);

    terrain->Initialize(location, m_params_mmx.length, m_params_mmx.width, m_params_mmx.height, m_params_mmx.radius,
                        m_params_mmx.density, m_params_mmx.pinfo);

    m_terrain = terrain;
}

// -----------------------------------------------------------------------------

TerrainForce MMXTireTestRig::ReportTireForce() const {
    return m_tire->ReportTireForce(m_terrain.get());
}

// -----------------------------------------------------------------------------

}  // end namespace vehicle
}  // end namespace chrono
