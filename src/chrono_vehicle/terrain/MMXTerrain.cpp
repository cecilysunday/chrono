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
// Default constructor
// -----------------------------------------------------------------------------
MMXTerrain::MMXTerrain(ChSystem* system)
    : m_start_id(0),
      m_min_num_particles(0),
      m_num_particles(0),
      m_rough_surface(false),
      m_vis_enabled(false) {
    
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
// Custom collision callback
// -----------------------------------------------------------------------------
class BoundaryContactMMX : public ChSystem::CustomCollisionCallback {
  public:
    BoundaryContactMMX(MMXTerrain* terrain) : m_terrain(terrain), m_radius(terrain->m_radius) {}
    virtual void OnCustomCollision(ChSystem* system) override;

  private:
    void CheckBottom(ChBody* body, const ChVector<>& center);
    //void CheckLeft(ChBody* body, const ChVector<>& center);
    //void CheckRight(ChBody* body, const ChVector<>& center);
    //void CheckFront(ChBody* body, const ChVector<>& center);
    //void CheckRear(ChBody* body, const ChVector<>& center);

    MMXTerrain* m_terrain;
    double m_radius;
};

void BoundaryContactMMX::OnCustomCollision(ChSystem* system) {
    auto bodylist = system->Get_bodylist();
    for (auto body : bodylist) {
        auto center = body->GetPos();
        //if (body->GetIdentifier() > m_terrain->m_start_id) {
            // CheckLeft(body.get(), center);
            // CheckRight(body.get(), center);
            // CheckFront(body.get(), center);
            // CheckRear(body.get(), center);
            CheckBottom(body.get(), center);
        //}
    }
}

// Check contact between granular material and bottom boundary.
void BoundaryContactMMX::CheckBottom(ChBody* body, const ChVector<>& center) {
    double dist = center.z() - m_terrain->m_bottom;

    if (dist > m_radius)
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
/*void BoundaryContactMMX::CheckLeft(ChBody* body, const ChVector<>& center) {
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
}*/

// -----------------------------------------------------------------------------
// Initialize the MMW terrain container
// -----------------------------------------------------------------------------
void MMXTerrain::Initialize(ChVector<>& center,
                            double length,
                            double width,
                            double height,
                            double radius,
                            double density,
                            const ChVector<>& init_vel) {

	// Set the container and grain dimensions
    m_length = length;
    m_width = width;
    m_height = height;
    m_radius = radius;

    // Set boundary locations
    m_front = center.x() + length / 2;
    m_rear = center.x() - length / 2;
    m_left = center.y() + width / 2;
    m_right = center.y() - width / 2;
    m_bottom = center.z();

    /*// Create a box around the terrain area. If enabled, create visualization assets for the boundaries.
	// FIX THESE
    double cthickness = 10.0;
    double cmass = 1.0E3;
    double tempt = cthickness * 0.99;
    double temph = 2.0 * height;

    ChVector<> sbase = ChVector<>(length, width, tempt) / 2.0;
    ChVector<> srght = ChVector<>(tempt, width, temph + 2.0 * cthickness) / 2.0;
    ChVector<> sback = ChVector<>(length + 2.0 * cthickness, tempt, temph + 2.0 * cthickness) / 2.0;

    ChVector<> pbase = center - ChVector<>(0, 0, cthickness) / 2.0;
    ChVector<> prght = center + ChVector<>(length + cthickness, 0, temph) / 2.0;
    ChVector<> pleft = center - ChVector<>(length + cthickness, 0, -temph) / 2.0;
    ChVector<> pfrnt = center + ChVector<>(0, width + cthickness, temph) / 2.0;
    ChVector<> pback = center - ChVector<>(0, width + cthickness, -temph) / 2.0;

    //auto base = m_ground->GetSystem()->NewBody();
    int ground_id = m_ground->GetSystem()->Get_bodylist().size() * -1;
    /*m_ground->SetIdentifier(ground_id);
    m_ground->SetMass(cmass);
    m_ground->SetPos(pbase);
    m_ground->SetBodyFixed(true);
    m_ground->SetCollide(true);

    m_ground->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(m_ground.get(), m_material, sbase, VNULL, QUNIT, m_vis_enabled);  // UPDATE FOR TWO MATERIAL INPUTS
    m_ground->GetCollisionModel()->BuildModel();

    auto right = m_ground->GetSystem()->NewBody();
    right->SetIdentifier(--ground_id);
    right->SetMass(cmass);
    right->SetPos(prght);
    right->SetBodyFixed(true);
    right->SetCollide(true);

    right->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(right, m_material, srght, VNULL, QUNIT, m_vis_enabled);  // UPDATE FOR TWO MATERIAL INPUTS
    right->GetCollisionModel()->BuildModel();

    auto left = m_ground->GetSystem()->NewBody();
    left->SetIdentifier(--ground_id);
    left->SetMass(cmass);
    left->SetPos(pleft);
    left->SetBodyFixed(true);
    left->SetCollide(true);

    left->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(left, m_material, srght, VNULL, QUNIT, m_vis_enabled);  // UPDATE FOR TWO MATERIAL INPUTS
    left->GetCollisionModel()->BuildModel();

    auto back = m_ground->GetSystem()->NewBody();
    back->SetIdentifier(--ground_id);
    back->SetMass(cmass);
    back->SetPos(pback);
    back->SetBodyFixed(true);
    back->SetCollide(true);

    back->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(back, m_material, sback, VNULL, QUNIT, m_vis_enabled);  // UPDATE FOR TWO MATERIAL INPUTS
    back->GetCollisionModel()->BuildModel();

    auto front = m_ground->GetSystem()->NewBody();
    front->SetIdentifier(--ground_id);
    front->SetMass(cmass);
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
    m_ground->GetSystem()->AddBody(frntPtr);*/


	// Move the ground body at patch location
    int ground_id = m_ground->GetSystem()->Get_bodylist().size() * -1;
    m_ground->SetPos(center);
    m_ground->SetIdentifier(m_start_id);

    // Create particles fixed to ground.
    if (m_rough_surface) {
        // Re-name and define certain size parameters
        double marg = radius * 1.01;
        double sft_yx = marg;
        double sft_yy = 2.0 * marg * sin(CH_C_PI / 3.0);
        double numx = ceil(length / (marg * 2.0));
        double numy = ceil(width / sft_yy);

        // Add roughness to the base of the box because collision detection does not support cylinder - box contacts
        ChVector<> pos_ref = center + ChVector<>(-length / 2.0 + marg, -width / 2.0 + marg, marg);
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
                        sphere->SetBodyFixed(false);   //  TRUE
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

    // Register the custom collision callback for boundary conditions.
    auto cb = chrono_types::make_shared<BoundaryContactMMX>(this);
    m_ground->GetSystem()->RegisterCustomCollisionCallback(cb);

}







// FIX THIS ?
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

// FIX THIS ? 
float MMXTerrain::GetCoefficientFriction(const ChVector<>& loc) const {
    if (m_friction_fun)
        return (*m_friction_fun)(loc);

    return m_material->GetSfriction();
}

}  // end namespace vehicle
}  // end namespace chrono
