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
// Granular terrain model.
// This class implements a rectangular patch of granular terrain.
// Optionally, a moving patch feature can be enable so that the patch is
// relocated (currently only in the positive X direction) based on the position
// of a user-specified body.
// Boundary conditions (model of a container bin) are imposed through a custom
// collision detection object.
//
// Reference frame is ISO (X forward, Y left, Z up).
// All units SI.
//
// =============================================================================

#ifndef MMX_TERRAIN_H
#define MMX_TERRAIN_H

#include "chrono/assets/ChColorAsset.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChMaterialSurfaceNSC.h"
#include "chrono/physics/ChMaterialSurfaceSMC.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTerrain.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_terrain
/// @{

/// Granular terrain model.
/// This class implements a rectangular patch of granular terrain with spherical particles.
/// Boundary conditions (model of a container bin) are imposed through a custom collision
/// detection object.
class CH_VEHICLE_API MMXTerrain : public ChTerrain {
  public:
    /// Construct a default GranularTerrain.
    /// The user is responsible for calling various Set methods before Initialize.
    MMXTerrain(ChSystem* system  ///< [in] pointer to the containing multibody system
                    );

    ~MMXTerrain() {}

    /// Set contact material (must be consistent with the containing system).
    void SetContactMaterial(std::shared_ptr<ChMaterialSurface> material) { m_material = material; }

    /// Get the current contact material.
    std::shared_ptr<ChMaterialSurface> GetContactMaterial() const { return m_material; }

    /// Set start value for body identifiers of generated particles (default: 1000000).
    /// It is assumed that all bodies with a larger identifier are granular material particles.
    void SetStartIdentifier(int id) { m_start_id = id; }

    /// Enable/disable visualization of boundaries (default: false).
    void EnableVisualization(bool val) { m_vis_enabled = val; }

    /// Set boundary visualization color.
    void SetGroundColor(ChColor color) { m_ground_color->SetColor(color); }


	void SetSphereColor(ChColor color) { m_sphere_color->SetColor(color); }

    /// Return a handle to the ground body.
    std::shared_ptr<ChBody> GetGroundBody() { return m_ground; }

    /// Initialize the granular terrain system.
    /// The granular material is created in successive layers within the specified volume,
    /// using the specified generator, until the number of particles exceeds the specified
    /// minimum value (see SetMinNumParticles).
    /// The initial particle locations are obtained with Poisson Disk sampling, using the
    /// given minimum separation distance.
    void Initialize(const ChVector<>& center,	
                    double length,
                    double width,
                    double height,
                    double radius,
                    double density,
                    const std::vector<std::pair<ChVector<>, double>>& pinfo
                    );

    /// Update the state of the terrain system at the specified time.
    virtual void Synchronize(double time) override;

    /// Get the number of particles.
    unsigned int GetNumParticles() const { return m_num_particles; }

    /// Get the terrain height below the specified location.
    /// This function returns the highest point over all granular particles.
    virtual double GetHeight(const ChVector<>& loc) const override;

    /// Get the terrain normal at the point below the specified location.
    virtual chrono::ChVector<> GetNormal(const ChVector<>& loc) const override;

    /// Get the terrain coefficient of friction at the point below the specified location.
    /// This coefficient of friction value may be used by certain tire models to modify
    /// the tire characteristics, but it will have no effect on the interaction of the terrain
    /// with other objects (including tire models that do not explicitly use it).
    /// For GranularTerrain, this function defers to the user-provided functor object of type
    /// ChTerrain::FrictionFunctor, if one was specified.
    /// Otherwise, it returns the constant value specified through SetContactFrictionCoefficient.
    virtual float GetCoefficientFriction(const ChVector<>& loc) const override;

  private:
    unsigned int m_num_particles;       ///< actual number of particles
    int m_start_id;                     ///< start body identifier for particles
    double m_radius;                    ///< particle radius

    // Patch dimensions
    double m_length;					///< length (X direction) of granular patch
    double m_width;						///< width (Y direction) of granular patch
    double m_height;					///< width (z direction) of granular patch

    // Boundary locations
    double m_front;						///< front (positive X) boundary location
    double m_rear;						///< rear (negative X) boundary location
    double m_left;						///< left (positive Y) boundary location
    double m_right;						///< right (negative Y) boundary location
    double m_bottom;					///< bottom boundary location

    bool m_vis_enabled;                     ///< boundary visualization enabled?
    
	std::shared_ptr<ChBody> m_ground;       ///< ground body
    
	std::shared_ptr<ChColorAsset> m_ground_color;  ///< color of boundary visualization asset
    std::shared_ptr<ChColorAsset> m_sphere_color; 

    std::shared_ptr<ChMaterialSurface> m_material; ///< contact material properties    

    friend class BoundaryContactMMX;
};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
