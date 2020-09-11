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
// Template for a rigid tire
//
// =============================================================================

#include <algorithm>

#include "chrono/core/ChGlobal.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChContactContainer.h"

#include "chrono_vehicle/wheeled_vehicle/tire/MMXPaddleTire.h"

#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MMXPaddleTire::MMXPaddleTire(const std::string& name) : ChTire(name), m_use_contact_mesh(false), m_trimesh(nullptr) {}

MMXPaddleTire::~MMXPaddleTire() {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void MMXPaddleTire::SetMeshFilename(const std::string& mesh_file, double sweep_sphere_radius) {
    m_use_contact_mesh = true;
    m_contact_meshFile = mesh_file;
    m_sweep_sphere_radius = sweep_sphere_radius;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void MMXPaddleTire::Initialize(std::shared_ptr<ChWheel> wheel) {
    ChTire::Initialize(wheel);

    auto wheel_body = wheel->GetSpindle();

    CreateContactMaterial(wheel_body->GetSystem()->GetContactMethod());
    assert(m_material && m_material->GetContactMethod() == wheel_body->GetSystem()->GetContactMethod());
    
    wheel_body->SetCollide(true);

    wheel_body->GetCollisionModel()->ClearModel();

    wheel_body->GetCollisionModel()->SetFamily(WheeledCollisionFamily::TIRES);

    if (m_use_contact_mesh) {
        // Mesh contact
        m_trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        m_trimesh->LoadWavefrontMesh(m_contact_meshFile, true, false);

        //// RADU
        // Hack to deal with current limitation: cannot set offset on a trimesh collision shape!
        double offset = GetOffset();
        if (std::abs(offset) > 1e-3) {
            for (int i = 0; i < m_trimesh->m_vertices.size(); i++)
                m_trimesh->m_vertices[i].y() += offset;
        }

        wheel_body->GetCollisionModel()->AddTriangleMesh(m_material, m_trimesh, false, false, ChVector<>(0),
                                                         ChMatrix33<>(1), m_sweep_sphere_radius);
    } else {
        double ngrouser = 9;
        double glength = GetRadius() / 5.35;
        double gthickness = glength / 2.0;
        double goffset = glength / 10.0;

		utils::AddCylinderGeometry(wheel_body.get(), m_material, GetRadius() - glength, GetWidth() / 2.0, 
			ChVector<>(0, GetOffset(), 0));
        
		for (int ig = 0; ig < ngrouser; ++ig) {
            double gradius = GetRadius() - (glength + goffset) / 2.0;
            double theta = ig * (2.0 * CH_C_PI / ngrouser);

            ChQuaternion<> z2g;
            z2g.Q_from_AngY(theta);
            utils::AddBoxGeometry(wheel_body.get(), m_material,
                                  ChVector<>(gthickness, GetWidth(), glength + goffset) / 2.0,
                                  ChVector<>(gradius * sin(theta), GetOffset(), gradius * cos(theta)),
								  z2g);		
		}
    }

    wheel_body->GetCollisionModel()->BuildModel();

}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void MMXPaddleTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;
    
	// TODO....visualization is on, even for VisualizationType::NONE
	// The shape asset is now taken care of at the same time as the collision model 
	auto mvisual = chrono_types::make_shared<ChColorAsset>();
    mvisual->SetColor(ChColor(0.49f, 0.73f, 0.91f));
    m_wheel->GetSpindle()->AddAsset(mvisual);

}

void MMXPaddleTire::RemoveVisualizationAssets() {
    // Make sure we only remove the assets added by ChRigidTire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    auto& assets = m_wheel->GetSpindle()->GetAssets();
    {
        auto it = std::find(assets.begin(), assets.end(), m_cyl_shape);
        if (it != assets.end())
            assets.erase(it);
    }
    {
        auto it = std::find(assets.begin(), assets.end(), m_texture);
        if (it != assets.end())
            assets.erase(it);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

// Callback class to process contacts on a rigid tire.
// Accumulate contact forces and torques on the associated wheel body.
// Express them in the global frame, as applied to the wheel center.
class RigidPaddleTireContactReporter : public ChContactContainer::ReportContactCallback {
  public:
    RigidPaddleTireContactReporter(std::shared_ptr<ChBody> body) : m_body(body) {}

    // Accumulated force, expressed in global frame, applied to wheel center.
    const ChVector<>& GetAccumulatedForce() const { return m_force; }

    // Accumulated torque, expressed in global frame.
    const ChVector<>& GetAccumulatedTorque() const { return m_torque; }

  private:
    virtual bool OnReportContact(const ChVector<>& pA,
                                 const ChVector<>& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const double& eff_radius,
                                 const ChVector<>& rforce,
                                 const ChVector<>& rtorque,
                                 ChContactable* modA,
                                 ChContactable* modB) override {
        // Filter contacts that involve the tire body.
        if (modA == m_body.get() || modB == m_body.get()) {
            // Express current contact force and torque in global frame
            ChVector<> force = plane_coord * rforce;
            ChVector<> torque = plane_coord * rtorque;
            // Wheel center in global frame
            const ChVector<>& center = m_body->GetPos();
            // Accumulate
            m_force += force;
            m_torque += torque + Vcross(Vsub(pA, center), force);
        }

        return true;
    }

    std::shared_ptr<ChBody> m_body;
    ChVector<> m_force;
    ChVector<> m_torque;
};

TerrainForce MMXPaddleTire::GetTireForce() const {
    // A ChRigidTire always returns zero force and moment since tire forces are automatically applied
    // to the associated wheel through Chrono's frictional contact system.
    TerrainForce tire_force;
    tire_force.point = m_wheel->GetPos();
    tire_force.force = ChVector<>(0, 0, 0);
    tire_force.moment = ChVector<>(0, 0, 0);

    return tire_force;
}

TerrainForce MMXPaddleTire::ReportTireForce(ChTerrain* terrain) const {
    // If interacting with an SCM terrain, interrogate the terrain system
    // for the cumulative force on the associated rigid body.
    if (auto scm = dynamic_cast<SCMDeformableTerrain*>(terrain)) {
        return scm->GetContactForce(m_wheel->GetSpindle());
    }

    // Otherwise, calculate and return the resultant of the contact forces acting on the tire.
    // The resulting tire force and moment are expressed in global frame, as applied at the center
    // of the associated spindle body.

    TerrainForce tire_force;
    tire_force.point = m_wheel->GetSpindle()->GetPos();
    tire_force.force = m_wheel->GetSpindle()->GetContactForce();
    tire_force.moment = m_wheel->GetSpindle()->GetContactTorque();

    // Approach using the RigidTireContactReporter does not work in Chrono::Parallel
    // since contact forces and torques passed to OnReportContact are always zero.
    /*
    auto reporter = chrono_types::make_shared<RigidPaddleTireContactReporter>(m_wheel);
    m_wheel->GetSpindle()->GetSystem()->GetContactContainer()->ReportAllContacts(&reporter);
    TerrainForce tire_force;
    tire_force.point = m_wheel->GetSpindle()->GetPos();
    tire_force.force = reporter->GetAccumulatedForce();
    tire_force.moment = reporter->GetAccumulatedTorque();
    */

    return tire_force;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
unsigned int MMXPaddleTire::GetNumVertices() const {
    assert(m_use_contact_mesh);
    return static_cast<unsigned int>(m_trimesh->getCoordsVertices().size());
}

unsigned int MMXPaddleTire::GetNumTriangles() const {
    assert(m_use_contact_mesh);
    return static_cast<unsigned int>(m_trimesh->getIndicesVertexes().size());
}

const std::vector<ChVector<int>>& MMXPaddleTire::GetMeshConnectivity() const {
    assert(m_use_contact_mesh);
    return m_trimesh->getIndicesVertexes();
}

const std::vector<ChVector<>>& MMXPaddleTire::GetMeshVertices() const {
    assert(m_use_contact_mesh);
    return m_trimesh->getCoordsVertices();
}

const std::vector<ChVector<>>& MMXPaddleTire::GetMeshNormals() const {
    assert(m_use_contact_mesh);
    return m_trimesh->getCoordsNormals();
}

void MMXPaddleTire::GetMeshVertexStates(std::vector<ChVector<>>& pos, std::vector<ChVector<>>& vel) const {
    assert(m_use_contact_mesh);
    auto vertices = m_trimesh->getCoordsVertices();

    for (size_t i = 0; i < vertices.size(); ++i) {
        pos.push_back(m_wheel->GetSpindle()->TransformPointLocalToParent(vertices[i]));
        vel.push_back(m_wheel->GetSpindle()->PointSpeedLocalToParent(vertices[i]));
    }
}

}  // end namespace vehicle
}  // end namespace chrono
