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
// MMX cylindrical tire subsystem
//
// =============================================================================

#include "chrono_models/vehicle/mmx/MMX_Tire_CylinderMesh.h"
#include "chrono/core/ChGlobal.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace mmx {

// -----------------------------------------------------------------------------
// Static variables - units in mm g s
// -----------------------------------------------------------------------------

const double MMX_Tire_CylinderMesh::m_radius = 50.0;
const double MMX_Tire_CylinderMesh::m_width = 100.0;
const double MMX_Tire_CylinderMesh::m_mass = 1000;

const ChVector<> MMX_Tire_CylinderMesh::m_inertia(1458333.33, 1458333.33, 1250000.0);

const std::string MMX_Tire_CylinderMesh::m_meshFile("vehicle/mmx/MMX_Tire_Cylinder_Mesh3.obj");

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MMX_Tire_CylinderMesh::MMX_Tire_CylinderMesh(const std::string& name) : MMXTire(name) {
	vehicle::SetDataPath(GetChronoDataPath());
    SetMeshFilename(vehicle::GetDataFile(m_meshFile), 0);
}

void MMX_Tire_CylinderMesh::CreateContactMaterial(ChContactMethod contact_method) {
    MaterialInfo minfo;
    minfo.mu = 0.45f;
    minfo.cr = 0.50f;
    minfo.Y = 7.0e8f;
    m_material = minfo.CreateMaterial(contact_method);
}

void MMX_Tire_CylinderMesh::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE) {
        return;
    } else {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile, m_meshFile);
    }
}

void MMX_Tire_CylinderMesh::RemoveVisualizationAssets() {
    MMXTire::RemoveVisualizationAssets();
    RemoveVisualizationMesh(m_trimesh_shape);
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
