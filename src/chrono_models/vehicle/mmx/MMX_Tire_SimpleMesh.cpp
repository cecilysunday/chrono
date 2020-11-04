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
// Authors: Radu Serban with modifications by Cecily Sunday
// =============================================================================
//
// MMX simple rigid tire subsystem
//
// =============================================================================

#include "chrono_models/vehicle/mmx/MMX_Tire_SimpleMesh.h"
#include "chrono/core/ChGlobal.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace mmx {

// -----------------------------------------------------------------------------
// Static variables - units in mm g s
// -----------------------------------------------------------------------------

const double MMX_Tire_SimpleMesh::m_radius = 107.0;
const double MMX_Tire_SimpleMesh::m_width = 53.0;
const double MMX_Tire_SimpleMesh::m_mass = 1.0;

const ChVector<> MMX_Tire_SimpleMesh::m_inertia(4321.333, 8174.500, 4321.333);

const std::string MMX_Tire_SimpleMesh::m_meshFile("vehicle/mmx/MMX_Tire_Simple_Mesh10.obj");

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MMX_Tire_SimpleMesh::MMX_Tire_SimpleMesh(const std::string& name) : MMXTire(name) {
	vehicle::SetDataPath(GetChronoDataPath());
    SetMeshFilename(vehicle::GetDataFile(m_meshFile), 0);
}

void MMX_Tire_SimpleMesh::CreateContactMaterial(ChContactMethod contact_method) {
    MaterialInfo minfo;
    minfo.mu = 0.45f;
    minfo.cr = 0.50f;
    minfo.Y = 2.0e7f;
    minfo.nu = 0.24f;
    m_material = minfo.CreateMaterial(contact_method);
}

void MMX_Tire_SimpleMesh::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE) {
        return;
    } else {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile, m_meshFile);
    }
}

void MMX_Tire_SimpleMesh::RemoveVisualizationAssets() {
    MMXTire::RemoveVisualizationAssets();
    RemoveVisualizationMesh(m_trimesh_shape);
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
