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
//
// TODO:
//   - re-enable collision envelope for lateral boundaries.
//   - currently disabled due to a bug in Chrono::Parallel where cohesion forces
//     are applied even when the penetration depth is positive!
//   - as a result, if envelope is considered, particles stick to the lateral
//     boundaries...
//   - for now, also make sure the envelope is not too large:  it's still used
//     for the bottom boundary and, if present, for the collision with the
//     ground-fixed spheres
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
// Constants
// -----------------------------------------------------------------------------

// Safety inflation factor for inter-particle initial separation.
const double safety_factor = 1.001;

// Offset particles from bottom boundary by offset_factor * radius.
// Note: this should be at least 3, to accommodate the case of rough surface.
const double offset_factor = 3;

// -----------------------------------------------------------------------------
// Default constructor.
// -----------------------------------------------------------------------------
MMXTerrain::MMXTerrain(ChSystem* system)
    : m_start_id(0),
      m_min_num_particles(0),
      m_num_particles(0),
      m_rough_surface(false),
      m_vis_enabled(false),
      // m_moving_patch(false),
      // m_moved(false),
      m_envelope(0) {
    
	  // Create the ground body and add it to the system.
      m_ground = std::shared_ptr<ChBody>(system->NewBody());
      m_ground->SetName("ground");
      m_ground->SetPos(ChVector<>(0, 0, 0));
      m_ground->SetBodyFixed(true);
      m_ground->SetCollide(false);
      system->AddBody(m_ground);

      // Set default parameters for contact material
      MaterialInfo minfo;
      minfo.mu = 0.9f;
      minfo.cr = 0.0f;
      minfo.Y = 2e5f;
      m_material = minfo.CreateMaterial(system->GetContactMethod());

      // Create the default color asset
      m_color = chrono_types::make_shared<ChColorAsset>();
      m_color->SetColor(ChColor(1, 1, 1));
      m_ground->AddAsset(m_color);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
//void MMXTerrain::EnableRoughSurface() {
//	m_center
//    m_rough_surface = true;
//}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
/* void MMXTerrain::EnableMovingPatch(std::shared_ptr<ChBody> body,
                                   double buffer_distance,
                                   double shift_distance,
                                   const ChVector<>& init_vel) {
    m_body = body;
    m_buffer_distance = buffer_distance;
    m_shift_distance = shift_distance;
    m_init_part_vel = init_vel;

    // Enable moving patch
    m_moving_patch = true;
}*/ 

// -----------------------------------------------------------------------------
// Custom collision callback
// -----------------------------------------------------------------------------
/* class BoundaryContactMMX : public ChSystem::CustomCollisionCallback {
  public:
    BoundaryContactMMX(MMXTerrain* terrain) : m_terrain(terrain), m_radius(terrain->m_radius) {}
    virtual void OnCustomCollision(ChSystem* system) override;

  private:
    void CheckBottom(ChBody* body, const ChVector<>& center);
    void CheckLeft(ChBody* body, const ChVector<>& center);
    void CheckRight(ChBody* body, const ChVector<>& center);
    void CheckFront(ChBody* body, const ChVector<>& center);
    void CheckRear(ChBody* body, const ChVector<>& center);

    MMXTerrain* m_terrain;
    double m_radius;
};

void BoundaryContactMMX::OnCustomCollision(ChSystem* system) {
    auto bodylist = system->Get_bodylist();
    for (auto body : bodylist) {
        auto center = body->GetPos();
        if (body->GetIdentifier() > m_terrain->m_start_id) {
            CheckLeft(body.get(), center);
            CheckRight(body.get(), center);
            CheckFront(body.get(), center);
            CheckRear(body.get(), center);
            // if (!m_terrain->m_rough_surface)
            CheckBottom(body.get(), center);
        }
    }
}

// Check contact between granular material and bottom boundary.
void BoundaryContactMMX::CheckBottom(ChBody* body, const ChVector<>& center) {
    double dist = center.z() - m_terrain->m_bottom;

    if (dist > m_radius + 2 * m_terrain->m_envelope)
        return;

    collision::ChCollisionInfo contact;
    contact.modelA = m_terrain->m_ground->GetCollisionModel().get();
    contact.modelB = body->GetCollisionModel().get();
    contact.shapeA = nullptr;
    contact.shapeB = nullptr;
    contact.vN = ChVector<>(0, 0, 1);
    contact.vpA = ChVector<>(center.x(), center.y(), m_terrain->m_bottom);
    contact.vpB = ChVector<>(center.x(), center.y(), center.z() - m_radius);
    contact.distance = dist - m_radius;
    contact.eff_radius = m_radius;

    body->GetSystem()->GetContactContainer()->AddContact(contact, m_terrain->m_material, m_terrain->m_material);
}

// Check contact between granular material and left boundary.
void BoundaryContactMMX::CheckLeft(ChBody* body, const ChVector<>& center) {
    double dist = m_terrain->m_left - center.y();

    ////if (dist > m_radius + 2 * m_terrain->m_envelope)
    if (dist > m_radius)
        return;

    collision::ChCollisionInfo contact;
    contact.modelA = m_terrain->m_ground->GetCollisionModel().get();
    contact.modelB = body->GetCollisionModel().get();
    contact.shapeA = nullptr;
    contact.shapeB = nullptr;
    contact.vN = ChVector<>(0, -1, 0);
    contact.vpA = ChVector<>(center.x(), m_terrain->m_left, center.z());
    contact.vpB = ChVector<>(center.x(), center.y() + m_radius, center.z());
    contact.distance = dist - m_radius;
    contact.eff_radius = m_radius;

    body->GetSystem()->GetContactContainer()->AddContact(contact, m_terrain->m_material, m_terrain->m_material);
}

// Check contact between granular material and right boundary.
void BoundaryContactMMX::CheckRight(ChBody* body, const ChVector<>& center) {
    double dist = center.y() - m_terrain->m_right;

    ////if (dist > m_radius + 2 * m_terrain->m_envelope)
    if (dist > m_radius)
        return;

    collision::ChCollisionInfo contact;
    contact.modelA = m_terrain->m_ground->GetCollisionModel().get();
    contact.modelB = body->GetCollisionModel().get();
    contact.shapeA = nullptr;
    contact.shapeB = nullptr;
    contact.vN = ChVector<>(0, 1, 0);
    contact.vpA = ChVector<>(center.x(), m_terrain->m_right, center.z());
    contact.vpB = ChVector<>(center.x(), center.y() - m_radius, center.z());
    contact.distance = dist - m_radius;
    contact.eff_radius = m_radius;

    body->GetSystem()->GetContactContainer()->AddContact(contact, m_terrain->m_material, m_terrain->m_material);
}

// Check contact between granular material and front boundary.
void BoundaryContactMMX::CheckFront(ChBody* body, const ChVector<>& center) {
    double dist = m_terrain->m_front - center.x();

    ////if (dist > m_radius + 2 * m_terrain->m_envelope)
    if (dist > m_radius)
        return;

    collision::ChCollisionInfo contact;
    contact.modelA = m_terrain->m_ground->GetCollisionModel().get();
    contact.modelB = body->GetCollisionModel().get();
    contact.shapeA = nullptr;
    contact.shapeB = nullptr;
    contact.vN = ChVector<>(-1, 0, 0);
    contact.vpA = ChVector<>(m_terrain->m_front, center.y(), center.z());
    contact.vpB = ChVector<>(center.x() + m_radius, center.y(), center.z());
    contact.distance = dist - m_radius;
    contact.eff_radius = m_radius;

    body->GetSystem()->GetContactContainer()->AddContact(contact, m_terrain->m_material, m_terrain->m_material);
}

// Check contact between granular material and rear boundary.
void BoundaryContactMMX::CheckRear(ChBody* body, const ChVector<>& center) {
    double dist = center.x() - m_terrain->m_rear;

    ////if (dist > m_radius + 2 * m_terrain->m_envelope)
    if (dist > m_radius)
        return;

    collision::ChCollisionInfo contact;
    contact.modelA = m_terrain->m_ground->GetCollisionModel().get();
    contact.modelB = body->GetCollisionModel().get();
    contact.shapeA = nullptr;
    contact.shapeB = nullptr;
    contact.vN = ChVector<>(1, 0, 0);
    contact.vpA = ChVector<>(m_terrain->m_rear, center.y(), center.z());
    contact.vpB = ChVector<>(center.x() - m_radius, center.y(), center.z());
    contact.distance = dist - m_radius;
    contact.eff_radius = m_radius;

    body->GetSystem()->GetContactContainer()->AddContact(contact, m_terrain->m_material, m_terrain->m_material);
} */

// -----------------------------------------------------------------------------
// Initialize the granular terrain patch
// -----------------------------------------------------------------------------
void MMXTerrain::Initialize(ChVector<>& center,
                            double length,
                            double width,
                            double thickness,
                            unsigned int num_layers,
                            double radius,
                            double density,
                            const ChVector<>& init_vel) {
    m_length = length;
    m_width = width;
    m_radius = radius;
    m_thickness = thickness;

	// Update center position based on surface roughness
    if (m_rough_surface) {
        center = center - ChVector<>(0, 0, 2.0 * radius);
    }

    // Set boundary locations
    m_front = center.x() + length / 2;
    m_rear = center.x() - length / 2;
    m_left = center.y() + width / 2;
    m_right = center.y() - width / 2;
    m_bottom = center.z();

    // Move the ground body at patch location.
    // int ground_id = m_ground->GetSystem()->Get_bodylist().size() * -1;
    // m_ground->SetPos(center);

    // Create a box around the terrain area. If enabled, create visualization assets for the boundaries.
    double tempth = thickness * 0.99;
    double height = 4.0 * radius * num_layers;

    ChVector<> sbase = ChVector<>(length, width, tempth) / 2.0;
    ChVector<> srght = ChVector<>(tempth, width, height + 2.0 * thickness) / 2.0;
    ChVector<> sback = ChVector<>(length + 2.0 * thickness, tempth, height + 2.0 * thickness) / 2.0;

    ChVector<> pbase = center - ChVector<>(0, 0, thickness) / 2.0;
    ChVector<> prght = center + ChVector<>(length + thickness, 0, height) / 2.0;
    ChVector<> pleft = center - ChVector<>(length + thickness, 0, -height) / 2.0;
    ChVector<> pfrnt = center + ChVector<>(0, width + thickness, height) / 2.0;
    ChVector<> pback = center - ChVector<>(0, width + thickness, -height) / 2.0;

    //auto base = m_ground->GetSystem()->NewBody();
    int ground_id = m_ground->GetSystem()->Get_bodylist().size() * -1;
    m_ground->SetIdentifier(ground_id);
    m_ground->SetMass(1.0E3);  // UPDATE
    m_ground->SetPos(pbase);
    m_ground->SetBodyFixed(true);
    m_ground->SetCollide(true);

    m_ground->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(m_ground.get(), m_material, sbase, VNULL, QUNIT, m_vis_enabled);  // UPDATE FOR TWO MATERIAL INPUTS
    m_ground->GetCollisionModel()->BuildModel();

    auto right = m_ground->GetSystem()->NewBody();
    right->SetIdentifier(--ground_id);
    right->SetMass(1.0E3);  // UPDATE
    right->SetPos(prght);
    right->SetBodyFixed(true);
    right->SetCollide(true);

    right->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(right, m_material, srght, VNULL, QUNIT, m_vis_enabled);  // UPDATE FOR TWO MATERIAL INPUTS
    right->GetCollisionModel()->BuildModel();

    auto left = m_ground->GetSystem()->NewBody();
    left->SetIdentifier(--ground_id);
    left->SetMass(1.0E3);  // UPDATE
    left->SetPos(pleft);
    left->SetBodyFixed(true);
    left->SetCollide(true);

    left->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(left, m_material, srght, VNULL, QUNIT, m_vis_enabled);  // UPDATE FOR TWO MATERIAL INPUTS
    left->GetCollisionModel()->BuildModel();

    auto back = m_ground->GetSystem()->NewBody();
    back->SetIdentifier(--ground_id);
    back->SetMass(1.0E3);  // UPDATE
    back->SetPos(pback);
    back->SetBodyFixed(true);
    back->SetCollide(true);

    back->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(back, m_material, sback, VNULL, QUNIT, m_vis_enabled);  // UPDATE FOR TWO MATERIAL INPUTS
    back->GetCollisionModel()->BuildModel();

    auto front = m_ground->GetSystem()->NewBody();
    front->SetIdentifier(--ground_id);
    front->SetMass(1.0E3);  // UPDATE
    front->SetPos(pfrnt);
    front->SetBodyFixed(true);
    front->SetCollide(true);

    front->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(front, m_material, sback, VNULL, QUNIT, false);  // UPDATE FOR TWO MATERIAL INPUTS
    front->GetCollisionModel()->BuildModel();

    // std::shared_ptr<ChBody> basePtr(base);
    std::shared_ptr<ChBody> rghtPtr(right);
    std::shared_ptr<ChBody> leftPtr(left);
    std::shared_ptr<ChBody> backPtr(back);
    std::shared_ptr<ChBody> frntPtr(front);

    // m_ground->GetSystem()->AddBody(basePtr);
    m_ground->GetSystem()->AddBody(rghtPtr);
    m_ground->GetSystem()->AddBody(leftPtr);
    m_ground->GetSystem()->AddBody(backPtr);
    m_ground->GetSystem()->AddBody(frntPtr);

    // Create particles fixed to ground.
    if (m_rough_surface) {

        // Re-name and define certain size parameters
        double marg = radius * 1.01;
        double sft_yx = marg;
        double sft_yy = 2.0 * marg * sin(CH_C_PI / 3.0);
        double numx = ceil(length / (marg * 2.0));
        double numy = ceil(width / sft_yy);

        // Add roughness to the base of the box because collision detection does not support cylinder - box contacts
        ChVector<> pos_ref = center + ChVector<>(-length / 2.0 + marg, -width / 2.0 + marg, radius);
        for (int iy = 0; iy < numy; ++iy) {
            double posx = sft_yx * (iy % 2);
            double posy = sft_yy * iy;

            ChVector<> pos_next = pos_ref + ChVector<>(posx, posy, 0);
            for (int ix = 0; ix < numx; ++ix) {
                if (pos_next.x() <= center.x() + length / 2.0 - marg && 
					pos_next.x() >= center.x() - length / 2.0 + marg) { 
                    if (pos_next.y() <= center.y() + width / 2.0 - marg &&
                        pos_next.y() >= center.y() - width / 2.0 + marg) {
                        
						double mass = density * (4.0 / 3.0) * CH_C_PI * radius * radius * radius;
                        ChVector<> inertia = 0.4 * mass * radius * radius * ChVector<>(1, 1, 1);

                        ChBody* sphere = m_ground->GetSystem()->NewBody();
                        sphere->SetIdentifier(--ground_id);
                        sphere->SetMass(mass);
                        sphere->SetPos(pos_next);
                        sphere->SetInertiaXX(inertia);
                        sphere->SetBodyFixed(true);
                        sphere->SetCollide(true);
                        sphere->GetCollisionModel()->ClearModel();
                        utils::AddSphereGeometry(sphere, m_material, radius);
                        sphere->GetCollisionModel()->BuildModel();

						sphere->AddAsset(chrono_types::make_shared<ChColorAsset>(0.65f, 0.44f, 0.39f));

                        std::shared_ptr<ChBody> spherePtr(sphere);
                        m_ground->GetSystem()->AddBody(spherePtr);
                    }
                }
                pos_next += ChVector<>(2.0 * sft_yx, 0, 0);
            }
        }
    }

    // Set envelope to default value (5% of particle radius for NSC if not user specified and always 0 for SMC)
    switch (m_ground->GetSystem()->GetContactMethod()) {
        case ChContactMethod::NSC: {
            if (m_envelope < 0)
                m_envelope = 0.05 * radius;
            break;
        }
        case ChContactMethod::SMC: {
            m_envelope = 0; 
            break;
        }
    }

    // Set the ground body identifier.
    // m_ground->SetIdentifier(m_start_id);

    // Create a particle generator and a mixture entirely made out of spheres.
    // Set the starting value for particle body identifiers.
    /*utils::Generator generator(m_ground->GetSystem());
    generator.setBodyIdentifier(m_start_id + 1);
    std::shared_ptr<utils::MixtureIngredient> m1 = generator.AddMixtureIngredient(utils::MixtureType::SPHERE, 1.0);
    m1->setDefaultMaterial(m_material);
    m1->setDefaultDensity(density);
    m1->setDefaultSize(radius);

    // Create particles, in layers, until exceeding the specified number.
    double r = safety_factor * radius;
    unsigned int layer = 0;
    ChVector<> layer_hdims(length / 2 - r, width / 2 - r, 0);
    ChVector<> layer_center = center;
    layer_center.z() += offset_factor * r;

    while (layer < num_layers || m_num_particles < m_min_num_particles) {
        if (m_verbose)
            GetLog() << "Create layer at height: " << layer_center.z() << "\n";
        generator.createObjectsBox(utils::SamplingType::POISSON_DISK, 2 * r, layer_center, layer_hdims, init_vel);
        layer_center.z() += 2 * r;
        m_num_particles = generator.getTotalNumBodies();
        layer++;
    }*/

    // Register the custom collision callback for boundary conditions.
    // auto cb = chrono_types::make_shared<BoundaryContactMMX>(this);
    // m_ground->GetSystem()->RegisterCustomCollisionCallback(cb);

	// Construct a particle cloud inside of the box limits
    // TO DO: distribution on particle diameter
    // TO DO: particle initial velocity / particle mixing
    //double ag = radius * 0.1 * 10.0; // FIX ME
    //double marg = radius * (1.0 + ag);
    double marg = radius * 1.01;
    double sft_yx = marg;
    double sft_yy = 2.0 * marg * sin(CH_C_PI / 3.0);
    double sft_yz = marg * tan(CH_C_PI / 6.0);
    double sft_zz = marg * std::sqrt(4.0 - (1.0 / std::pow(cos(CH_C_PI / 6.0), 2.0)));
    double numx = ceil(length / (marg * 2.0));
    double numy = ceil(width / sft_yy);
    double numz = num_layers;

    ChVector<> pos_ref = center + ChVector<>(-length / 2.0 + marg, -width / 2.0 + marg, 4.0 * radius);
    for (int iz = 0; iz < numz; ++iz) {
		for (int iy = 0; iy < numy; ++iy) {
            double posx = sft_yx * (iy % 2) - sft_yx * (iz % 3); //FIX ME
            double posy = sft_yy * iy + sft_yz * (iz % 3);
            double posz = sft_zz * iz;

			ChVector<> pos_next = pos_ref + ChVector<>(posx, posy, posz);
			for (int ix = 0; ix < numx; ++ix) {
				if (pos_next.x() <= center.x() + length / 2.0 - marg && 
					pos_next.x() >= center.x() - length / 2.0 + marg) {
					if (pos_next.y() <= center.y() + width / 2.0 - marg && 
						pos_next.y() >= center.y() - width / 2.0 + marg) {
						double mass = density * (4.0 / 3.0) * CH_C_PI * radius * radius * radius;
						ChVector<> inertia = 0.4 * mass * radius * radius * ChVector<>(1, 1, 1);

						ChBody* sphere = m_ground->GetSystem()->NewBody();
						sphere->SetIdentifier(m_start_id++);
						sphere->SetMass(mass);
						sphere->SetPos(pos_next); 
						sphere->SetInertiaXX(inertia);
						sphere->SetBodyFixed(true);
						sphere->SetCollide(true);
						sphere->GetCollisionModel()->ClearModel();
						utils::AddSphereGeometry(sphere, m_material, radius);
						sphere->GetCollisionModel()->BuildModel();

						std::shared_ptr<ChBody> spherePtr(sphere);
						m_ground->GetSystem()->AddBody(spherePtr);
					}
				}
				pos_next += ChVector<>(2.0 * sft_yx, 0, 0);
			}
		}
	}
}

void MMXTerrain::Synchronize(double time) {
    return;
}

double MMXTerrain::GetHeight(const ChVector<>& loc) const {
    double highest = m_bottom;
    for (auto body : m_ground->GetSystem()->Get_bodylist()) {
        double height = ChWorldFrame::Height(body->GetPos());
        if (body->GetIdentifier() > m_start_id && body->GetPos().z() > highest)
            highest = body->GetPos().z();
    }
    return highest + m_radius;
}

ChVector<> MMXTerrain::GetNormal(const ChVector<>& loc) const {
    return ChWorldFrame::Vertical();
}

float MMXTerrain::GetCoefficientFriction(const ChVector<>& loc) const {
    if (m_friction_fun)
        return (*m_friction_fun)(loc);

    return m_material->GetSfriction();
}

}  // end namespace vehicle
}  // end namespace chrono
