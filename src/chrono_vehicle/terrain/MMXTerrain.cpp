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
//
// TODO: DO THESE COMMENTS STILL APPLY??
//   - Re-enable collision envelope for lateral boundaries.
//   - Currently disabled due to a bug in Chrono::Parallel where cohesion forces
//     are applied even when the penetration depth is positive!
//   - As a result, if envelope is considered, particles stick to the lateral
//     boundaries...
//   - For now, also make sure the envelope is not too large:  it's still used
//     for the bottom boundary and, if present, for the collision with the
//     ground-fixed spheres
//
//   - Import both wall and grain material properties
//   - Update the 'find height' function
//
// =============================================================================

#include <cstdio>
#include <cmath>

#include "chrono/core/ChLog.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsGenerators.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChPart.h"
#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/terrain/MMXTerrain.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Default constructor
// -----------------------------------------------------------------------------
MMXTerrain::MMXTerrain(ChSystem* system) : m_start_id(0), m_num_particles(0) {
    
	  // Create the ground body and add it to the system
      m_ground = std::shared_ptr<ChBody>(system->NewBody());
      m_ground->SetName("ground");
      m_ground->SetPos(ChVector<>(0, 0, 0));
      m_ground->SetBodyFixed(true);
      m_ground->SetCollide(false);
      system->AddBody(m_ground);

      // Set the default parameters for contact material
      MaterialInfo minfo;
      minfo.mu = 0.9f;
      minfo.cr = 0.0f;
      minfo.Y = 2e5f;
      m_ground_material = minfo.CreateMaterial(system->GetContactMethod());
      m_sphere_material = minfo.CreateMaterial(system->GetContactMethod());

      // Create the default color assets
      m_ground_color = chrono_types::make_shared<ChColorAsset>();
      m_ground_color->SetColor(ChColor(0.65f, 0.44f, 0.39f));

	  m_sphere_color = chrono_types::make_shared<ChColorAsset>();
      m_sphere_color->SetColor(ChColor(0.0f, 0.28f, 0.67f));
}

// -----------------------------------------------------------------------------
// Custom collision callback
// -----------------------------------------------------------------------------
class BoundaryContactMMX : public ChSystem::CustomCollisionCallback {
  public:
    BoundaryContactMMX(MMXTerrain* terrain) : m_terrain(terrain), m_radius(terrain->m_radius) {}
    virtual void OnCustomCollision(ChSystem* system) override;

  private:
    void CheckBottom(ChBody* body, const ChVector<>& center, const double& radius);
    void CheckLeft(ChBody* body, const ChVector<>& center, const double& radius);
    void CheckRight(ChBody* body, const ChVector<>& center, const double& radius);
    void CheckFront(ChBody* body, const ChVector<>& center, const double& radius);
    void CheckRear(ChBody* body, const ChVector<>& center, const double& radius);

    MMXTerrain* m_terrain;
    double m_radius;
};

void BoundaryContactMMX::OnCustomCollision(ChSystem* system) {
    auto bodylist = system->Get_bodylist();
    for (auto body : bodylist) {
		if (body->GetIdentifier() >= m_terrain->m_start_id) {
            auto center = body->GetPos();
            auto radius = body->GetCollisionModel()->GetShapeDimensions(0).at(0);
			
			CheckBottom(body.get(), center, radius);
            CheckLeft(body.get(), center, radius);
            CheckRight(body.get(), center, radius);
            CheckFront(body.get(), center, radius);
            CheckRear(body.get(), center, radius);
        }
    }
}

// Check contact between granular material and bottom boundary.
void BoundaryContactMMX::CheckBottom(ChBody* body, const ChVector<>& center, const double& radius) {
    double dist = center.z() - m_terrain->m_bottom;

    if (dist > radius)
        return;

    collision::ChCollisionInfo contact;
    contact.modelA = m_terrain->m_ground->GetCollisionModel().get();
    contact.modelB = body->GetCollisionModel().get();
    contact.shapeA = nullptr;
    contact.shapeB = nullptr;
    contact.vN = ChVector<>(0, 0, 1);
    contact.vpA = ChVector<>(center.x(), center.y(), m_terrain->m_bottom);
    contact.vpB = ChVector<>(center.x(), center.y(), center.z() - radius);
    contact.distance = dist - radius;
    contact.eff_radius = radius;

    body->GetSystem()->GetContactContainer()->AddContact(contact, m_terrain->m_ground_material, m_terrain->m_sphere_material);
}

// Check contact between granular material and left boundary.
void BoundaryContactMMX::CheckLeft(ChBody* body, const ChVector<>& center, const double& radius) {
    double dist = m_terrain->m_left - center.y();

    if (dist > radius)
        return;

    collision::ChCollisionInfo contact;
    contact.modelA = m_terrain->m_ground->GetCollisionModel().get();
    contact.modelB = body->GetCollisionModel().get();
    contact.shapeA = nullptr;
    contact.shapeB = nullptr;
    contact.vN = ChVector<>(0, -1, 0);
    contact.vpA = ChVector<>(center.x(), m_terrain->m_left, center.z());
    contact.vpB = ChVector<>(center.x(), center.y() + radius, center.z());
    contact.distance = dist - radius;
    contact.eff_radius = radius;

    body->GetSystem()->GetContactContainer()->AddContact(contact, m_terrain->m_ground_material, m_terrain->m_sphere_material);
}

// Check contact between granular material and right boundary.
void BoundaryContactMMX::CheckRight(ChBody* body, const ChVector<>& center, const double& radius) {
    double dist = center.y() - m_terrain->m_right;

    if (dist > radius)
        return;

    collision::ChCollisionInfo contact;
    contact.modelA = m_terrain->m_ground->GetCollisionModel().get();
    contact.modelB = body->GetCollisionModel().get();
    contact.shapeA = nullptr;
    contact.shapeB = nullptr;
    contact.vN = ChVector<>(0, 1, 0);
    contact.vpA = ChVector<>(center.x(), m_terrain->m_right, center.z());
    contact.vpB = ChVector<>(center.x(), center.y() - radius, center.z());
    contact.distance = dist - radius;
    contact.eff_radius = radius;

    body->GetSystem()->GetContactContainer()->AddContact(contact, m_terrain->m_ground_material, m_terrain->m_sphere_material);
}

// Check contact between granular material and front boundary.
void BoundaryContactMMX::CheckFront(ChBody* body, const ChVector<>& center, const double& radius) {
    double dist = m_terrain->m_front - center.x();

    if (dist > radius)
        return;

    collision::ChCollisionInfo contact;
    contact.modelA = m_terrain->m_ground->GetCollisionModel().get();
    contact.modelB = body->GetCollisionModel().get();
    contact.shapeA = nullptr;
    contact.shapeB = nullptr;
    contact.vN = ChVector<>(-1, 0, 0);
    contact.vpA = ChVector<>(m_terrain->m_front, center.y(), center.z());
    contact.vpB = ChVector<>(center.x() + radius, center.y(), center.z());
    contact.distance = dist - radius;
    contact.eff_radius = radius;

    body->GetSystem()->GetContactContainer()->AddContact(contact, m_terrain->m_ground_material, m_terrain->m_sphere_material);
}

// Check contact between granular material and rear boundary.
void BoundaryContactMMX::CheckRear(ChBody* body, const ChVector<>& center, const double& radius) {
    double dist = center.x() - m_terrain->m_rear;

    if (dist > radius)
        return;

    collision::ChCollisionInfo contact;
    contact.modelA = m_terrain->m_ground->GetCollisionModel().get();
    contact.modelB = body->GetCollisionModel().get();
    contact.shapeA = nullptr;
    contact.shapeB = nullptr;
    contact.vN = ChVector<>(1, 0, 0);
    contact.vpA = ChVector<>(m_terrain->m_rear, center.y(), center.z());
    contact.vpB = ChVector<>(center.x() - radius, center.y(), center.z());
    contact.distance = dist - radius;
    contact.eff_radius = radius;

    body->GetSystem()->GetContactContainer()->AddContact(contact, m_terrain->m_ground_material, m_terrain->m_sphere_material);
}

// -----------------------------------------------------------------------------
// Initialize the MMW terrain container
// -----------------------------------------------------------------------------
void MMXTerrain::Initialize(const ChVector<>& center,
                            double length,
                            double width,
                            double height,
                            double radius,
                            double density,
                            const std::vector<std::pair<ChVector<>, double>>& pinfo) {
    
	// Set the container and grain dimensions
    m_length = length;
    m_width = width;
    m_height = height;
    m_radius = radius;

    // Set the container boundary locations
    m_front = center.x() + length / 2.0;
    m_rear = center.x() - length / 2.0;
    m_left = center.y() + width / 2.0;
    m_right = center.y() - width / 2.0;
    m_bottom = center.z();

    // Create an invisible ground body at the base of the terrain patch. This is simply used as a reference body
    int g_id = -1 * m_ground->GetSystem()->Get_bodylist().size();
	m_ground->SetIdentifier(g_id);
    m_ground->SetPos(center);

	// Add the particles to the system. If initial prarticle positions are not provided, create a rough surface
    if (pinfo.size() == 0) {
		// Define spacing parameters needed to create the rough surface
        double marg = radius * 1.01;
        double sft_x = marg;
        double sft_y = 2.0 * marg * sin(CH_C_PI / 3.0);
        double numx = ceil(length / (marg * 2.0));
        double numy = ceil(width / sft_y);

        // Add roughness to the base of the box because collision detection does not support cylinder - box contacts
        ChVector<> pos_ref = ChVector<>(m_rear + marg, m_right + marg, m_bottom + marg);
        for (int iy = 0; iy < numy; ++iy) {
            double posx = sft_x * (iy % 2);
            double posy = sft_y * iy;

            ChVector<> pos_next = pos_ref + ChVector<>(posx, posy, 0);
            if (pos_next.y() <= m_left - marg && pos_next.y() >= m_right + marg) {
				for (int ix = 0; ix < numx; ++ix) {
					if (pos_next.x() <= m_front - marg && pos_next.x() >= m_rear + marg) {
						double mass = density * (4.0 / 3.0) * CH_C_PI * radius * radius * radius;
						ChVector<> inertia = 0.4 * mass * radius * radius * ChVector<>(1, 1, 1);

						ChBody* sphere = m_ground->GetSystem()->NewBody();
						sphere->SetIdentifier(--g_id);
						sphere->SetMass(mass);
						sphere->SetPos(pos_next);
						sphere->SetInertiaXX(inertia);
						sphere->SetBodyFixed(true);
						sphere->SetCollide(true);
                        sphere->AddAsset(m_ground_color);
						sphere->GetCollisionModel()->ClearModel();
						utils::AddSphereGeometry(sphere, m_sphere_material, radius);
						sphere->GetCollisionModel()->BuildModel();

						std::shared_ptr<ChBody> spherePtr(sphere);
						m_ground->GetSystem()->AddBody(spherePtr);
					}
					pos_next += ChVector<>(2.0 * sft_x, 0, 0);
				}
			}
        }
    } else {
		// Fill the container with particles using and input array of particle positions and sizes
        int p_id = m_start_id;
        for (size_t i = 0; i < pinfo.size(); i++) {
			double rad = pinfo.at(i).second;
            double mass = density * (4.0 / 3.0) * CH_C_PI * rad * rad * rad;
            ChVector<> pos = center + pinfo.at(i).first;
            ChVector<> inertia = 0.4 * mass * rad * rad * ChVector<>(1, 1, 1);

            ChBody* sphere = m_ground->GetSystem()->NewBody();
            sphere->SetIdentifier(p_id++);
            sphere->SetMass(mass);
            sphere->SetPos(pos);
            sphere->SetInertiaXX(inertia);
            sphere->SetBodyFixed(false);
            sphere->SetCollide(true);
            sphere->AddAsset(m_sphere_color);
            sphere->GetCollisionModel()->ClearModel();
            utils::AddSphereGeometry(sphere, m_sphere_material, rad);
            sphere->GetCollisionModel()->BuildModel();

            std::shared_ptr<ChBody> spherePtr(sphere);
            m_ground->GetSystem()->AddBody(spherePtr);

			m_num_particles++;
        }
    }

    // Register the custom collision callback for boundary conditions.
    auto cb = chrono_types::make_shared<BoundaryContactMMX>(this);
    m_ground->GetSystem()->RegisterCustomCollisionCallback(cb);
}

void MMXTerrain::Synchronize(double time) {
    return;
}

// FIX THIS
double MMXTerrain::GetHeight(const ChVector<>& loc) const {
    /*double highest = m_bottom;
    for (auto body : m_ground->GetSystem()->Get_bodylist()) {
        double height = ChWorldFrame::Height(body->GetPos());
        if (body->GetIdentifier() > m_start_id && body->GetPos().z() > highest)
            highest = body->GetPos().z();
    }
    return highest + m_radius;*/
    return m_height;
}

ChVector<> MMXTerrain::GetNormal(const ChVector<>& loc) const {
    return ChWorldFrame::Vertical();
}

float MMXTerrain::GetCoefficientFriction(const ChVector<>& loc) const {
    if (m_friction_fun)
        return (*m_friction_fun)(loc);

    return m_sphere_material->GetSfriction();
}

}  // end namespace vehicle
}  // end namespace chrono
