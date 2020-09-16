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
// MMX terrain model
//
// This class implements a patch of rectangular terrain based on a list of
// input particle positions and sizes. Boundary conditions (model of a container
// bin) are imposed through a custom collision detection object.
//
// Reference frame is ISO (X forward, Y left, Z up)
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

/// MMX terrain model.
/// This class implements a rectangular patch of granular terrain with spherical particles.
/// Boundary conditions (model of a container bin) are imposed through a custom collision
/// detection object.
class CH_VEHICLE_API MMXTerrain : public ChTerrain {
  public:
    /// Construct a default MMXTerrain.
    /// The user is responsible for calling various Set methods before Initialize.
    MMXTerrain(ChSystem* system  ///< [in] pointer to the containing multibody system
                    );

    ~MMXTerrain() {}

    /// Set the contact material for the boundary walls (must be consistent with the containing system).
    void SetGroundContactMaterial(std::shared_ptr<ChMaterialSurface> material) { m_ground_material = material; }

	/// Set the contact material for the terrain particles(must be consistent with the containing system).
    void SetSphereContactMaterial(std::shared_ptr<ChMaterialSurface> material) { m_sphere_material = material; }

    /// Get the current contact material for the boundary walls
    std::shared_ptr<ChMaterialSurface> GetGroundContactMaterial() const { return m_ground_material; }

	/// Get the current contact material for the terrain particles
    std::shared_ptr<ChMaterialSurface> GetSphereContactMaterial() const { return m_sphere_material; }

    /// Set start value for body identifiers of generated particles (default: 0).
    /// It is assumed that all bodies with a positive identifier are granular material particles.
    void SetStartIdentifier(int id) { m_start_id = id; }

    /// Set visualization color for the rough surface boundary
    void SetGroundColor(ChColor color) { m_ground_color->SetColor(color); }

	/// Set visualization color for the terrain grains
	void SetSphereColor(ChColor color) { m_sphere_color->SetColor(color); }

    /// Return a handle to the ground body
    std::shared_ptr<ChBody> GetGroundBody() { return m_ground; }

    /// Initialize the MMX terrain system.
    /// The granular material is created accoring to an input array of particle positions and sizes
    void Initialize(const ChVector<>& center,	
                    double length,
                    double width,
                    double height,
                    double radius,
                    double density,
                    const std::vector<std::pair<ChVector<>, double>>& pinfo
                    );

    /// Update the state of the terrain system at the specified time
    virtual void Synchronize(double time) override;

    /// Get the number of particles.
    unsigned int GetNumParticles() const { return m_num_particles; }

    /// Get the terrain height below the specified location.
    /// This function returns the highest point over all granular particles.
    virtual double GetHeight(const ChVector<>& loc) const override;

    /// Get the terrain normal at the point below the specified location
    virtual chrono::ChVector<> GetNormal(const ChVector<>& loc) const override;

    /// Get the terrain coefficient of friction at the point below the specified location.
    /// This coefficient of friction value may be used by certain tire models to modify
    /// the tire characteristics, but it will have no effect on the interaction of the terrain
    /// with other objects (including tire models that do not explicitly use it).
    /// For MMXTerrain, this function defers to the user-provided functor object of type
    /// ChTerrain::FrictionFunctor, if one was specified.
    /// Otherwise, it returns the constant value specified through SetContactFrictionCoefficient.
    virtual float GetCoefficientFriction(const ChVector<>& loc) const override;

  private:
    unsigned int m_num_particles;       ///< actual number of particles
    int m_start_id;                     ///< start body identifier for particles
    
	double m_radius;                    ///< particle radius
    double m_length;					///< length (X direction) of granular patch
    double m_width;						///< width (Y direction) of granular patch
    double m_height;					///< width (z direction) of granular patch
    double m_front;						///< front (positive X) boundary location
    double m_rear;						///< rear (negative X) boundary location
    double m_left;						///< left (positive Y) boundary location
    double m_right;						///< right (negative Y) boundary location
    double m_bottom;					///< bottom boundary location
    
	std::shared_ptr<ChBody> m_ground;						///< pointer to the ground body
    
	std::shared_ptr<ChColorAsset> m_ground_color;			///< color of the rough surface boundary particles
    std::shared_ptr<ChColorAsset> m_sphere_color;			///< color of the granular terrain particles 

	std::shared_ptr<ChMaterialSurface> m_ground_material;	///< contact material properties for the boundary walls  
    std::shared_ptr<ChMaterialSurface> m_sphere_material;	///< contact material properties for the terrain particles    

    friend class BoundaryContactMMX;
};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
