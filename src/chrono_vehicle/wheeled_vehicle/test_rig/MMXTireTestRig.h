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
// Authors: Radu Serban, Cecily Sunday
// =============================================================================
//
// Implementation of a single-tire test rig for MMX rover wheels
// - Accepts an arbitrary Chrono::Vehicle tire object (associated ChWheel object)
// - Works with MMX terrain
// - Allows variation of longitudinal speed, wheel angular speed, and wheel slip
//   angle as functions of time
// - Allows specification of camber angle (kept constant through the simulation)
//
// =============================================================================

#ifndef MMX_TIRE_TEST_RIG_H
#define MMX_TIRE_TEST_RIG_H

#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_test_rig
/// @{

/// Definition of a single-tire test rig.
class CH_VEHICLE_API MMXTireTestRig {
  public:
    /// Construct a tire test rig within the specified system.
    MMXTireTestRig(std::shared_ptr<ChWheel> wheel,  ///< wheel subsystem
                   std::shared_ptr<ChTire> tire,    ///< tire subsystem
                   ChSystem* system                 ///< containing mechanical system
    );

	MMXTireTestRig(ChSystem* system);

	// If the default constructor is used, set the system type
    void SetSystem(ChSystem* system) { m_system = system; }

	// If the default constructor is used, set the system type
    void SetWheel(std::shared_ptr<ChWheel> wheel) { m_wheel = wheel; }

	// If the default constructor is used, set the tire type
	void SetTire(std::shared_ptr<ChTire> tire) { m_tire = tire; }

    /// Set desired normal load (default: 0 N).
    void SetNormalLoad(double load) { m_normal_load = load; }

    /// Set camber angle (default: 0 rad).
    void SetCamberAngle(double camber) { m_camber_angle = camber; }

    /// Specify rig carrier longitudinal speed as function of time (default: none).
    /// If a function is not specified, the carrier is not actuated.
    void SetLongSpeedFunction(std::shared_ptr<ChFunction> funct);

    /// Specify wheel angular speed as function of time (default: none).
    /// If a function is not specified, the wheel is not actuated.
    void SetAngSpeedFunction(std::shared_ptr<ChFunction> funct);

    /// Set the time step for advancing tire dynamics (default: 1e-3 s).
    void SetTireStepsize(double step) { m_tire_step = step; }

    /// Set visualization type for the tire (default: PRIMITIVES).
    void SetTireVisualizationType(VisualizationType vis) { m_tire_vis = vis; }

    /// Import vectors containing inital size and location properties of the particles in the terrain
    void SetTerrainParticles(const std::vector<std::pair<ChVector<>, double>>& pinfo) { m_params_mmx.pinfo = pinfo; }

	/// Add an additional offset between the top of the top-center of the terrain and the bottom-center of the wheel
    void SetTerrainOffset(const ChVector<> offset) { m_terrain_offset = offset; }

    /// Enable use of MMX terrain.
    /// The terrain subsystem consists of identical spherical particles initialized in layers.
    /// A moving-patch option is used, with the patch dimensions set based on the tire dimensions.
    void SetTerrainMMX(std::shared_ptr<ChMaterialSurfaceSMC> sphere_mat,
                       std::shared_ptr<ChMaterialSurfaceSMC> ground_mat,
                       double length,
                       double width,
                       double height,
                       double radius,
                       double density);

    /// Initialize the rig system. This version uses all motion functions as specified by the user. It is the user's
    /// responsibility to set these up for a meaningful test.
    void InitializeRig();

    ///  Update the rig system if motor functions have changed
    void InitializeMotors();

	/// Apply an external load to the system
    void InitializeLoad() { m_load_chassis = true; }

    /// Advance system state by the specified time step.
    void Advance(double step);

    /// Get total rig mass.
    double GetTotalMass() const { return m_total_mass; }

    /// Get applied load on rig (to enforce specified normal load, taking into account the masses of all components).
    double GetAppliedLoad() const { return m_applied_load; }

    /// Get a handle to the underlying terrain subsystem.
    std::shared_ptr<ChTerrain> GetTerrain() const { return m_terrain; }

	/// Get a handle to the underlying tire subsystem.
    std::shared_ptr<ChTire> GetTire() const { return m_tire; }

    /// Get current carrier body position.
    const ChVector<>& GetPos() const { return m_carrier_body->GetPos(); }

    /// Get the current tire forces
    TerrainForce ReportTireForce() const;

  private:
    enum class TerrainType { MMX, NONE };

    struct TerrainParamsMMX {
        std::shared_ptr<ChMaterialSurfaceSMC> sphere_mat;
        std::shared_ptr<ChMaterialSurfaceSMC> ground_mat;
        double length;
        double width;
        double height;
        double radius;
        double density;
        std::vector<std::pair<ChVector<>, double>> pinfo = std::vector<std::pair<ChVector<>, double>>();
    };

    void CreateMechanism();

    void CreateTerrain();
    void CreateTerrainMMX();

    ChSystem* m_system;						 ///< pointer to the Chrono system

    std::shared_ptr<ChTerrain> m_terrain;    ///< handle to underlying terrain subsystem
    std::shared_ptr<ChWheel> m_wheel;        ///< handle to wheel subsystem
    std::shared_ptr<ChTire> m_tire;          ///< handle to tire subsystem
    VisualizationType m_tire_vis;            ///< visualization type for tire subsystem
    
	double m_tire_step;						 ///< step size for tire integration
    double m_camber_angle;					 ///< camber angle
    double m_normal_load;					 ///< desired normal load
    double m_applied_load;					 ///< applied load on chassis body
    double m_total_mass;					 ///< total sprung mass

    TerrainType m_terrain_type;				 ///< terrain type
    TerrainParamsMMX m_params_mmx;			 ///< granular terrain parameters
                                            
    double m_rig_hoffset;					 ///< horizontal offset from the base frame to the wheel face
    double m_rig_voffset;					 ///< vertical offset from the base frame to the bottom of the wheel

	ChVector<> m_terrain_offset;			 ///< additional user-defined terrain offset from the bottom-center of the wheel

    std::shared_ptr<ChBody> m_ground_body;   ///< pointer to the ground body
    std::shared_ptr<ChBody> m_carrier_body;  ///< pointer to the rig carrier body
    std::shared_ptr<ChBody> m_chassis_body;  ///< pointer to the "chassis" body which carries normal load
    std::shared_ptr<ChBody> m_slip_body;     ///< pointer to the intermediate body for controlling slip angle
    std::shared_ptr<ChBody> m_spindle_body;  ///< pointe to the wheel spindle body

	bool m_load_chassis;					 ///< is the external load applied to the system?
    bool m_ls_actuated;                      ///< is linear spped actuated?
    bool m_rs_actuated;                      ///< is angular speed actuated?
    
	std::shared_ptr<ChFunction> m_ls_fun;    ///< longitudinal speed function of time
    std::shared_ptr<ChFunction> m_rs_fun;    ///< angular speed function of time

    std::shared_ptr<ChLinkMotorLinearSpeed> m_lin_motor;    ///< pointer to the carrier actuator
    std::shared_ptr<ChLinkMotorRotationSpeed> m_rot_motor;  ///< pointer to the wheel actuator
    std::shared_ptr<ChLinkLockLock> m_slip_lock;            ///< pointer to the slip angle actuator
};

/// @} vehicle_wheeled_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
