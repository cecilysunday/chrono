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
// MMX rigid 'paddle-wheel' subsystem
//
// =============================================================================

#include "chrono_models/vehicle/mmx/MMX_Tire_PaddleMesh.h"
#include "chrono/core/ChGlobal.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace mmx {

// -----------------------------------------------------------------------------
// Static variables - units in mm g s
// -----------------------------------------------------------------------------

const double MMX_Tire_PaddleMesh::m_radius = 107.0;
const double MMX_Tire_PaddleMesh::m_width = 53.0;
const double MMX_Tire_PaddleMesh::m_mass = 1.0; // 838.270;

const ChVector<> MMX_Tire_PaddleMesh::m_inertia(3176.280, 5941.130, 3176.280);  // (2662580.480, 4980271.335, 2662580.480);

const std::string MMX_Tire_PaddleMesh::m_meshFile("vehicle/mmx/MMX_Tire_Paddle_Mesh10.obj"); // ("vehicle/mmx/MMX_Tire_Paddle_Mesh3.obj");

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MMX_Tire_PaddleMesh::MMX_Tire_PaddleMesh(const std::string& name) : MMXTire(name) {
	vehicle::SetDataPath(GetChronoDataPath());
    SetMeshFilename(vehicle::GetDataFile(m_meshFile), 0);
}

void MMX_Tire_PaddleMesh::CreateContactMaterial(ChContactMethod contact_method) {
    MaterialInfo minfo;
    minfo.mu = 0.45f;
    minfo.cr = 0.50f;
    minfo.Y = 2.0e7f;
    m_material = minfo.CreateMaterial(contact_method);
}

void MMX_Tire_PaddleMesh::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE) {
        return;
    } else {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile, m_meshFile);
    }
}

void MMX_Tire_PaddleMesh::RemoveVisualizationAssets() {
    MMXTire::RemoveVisualizationAssets();
    RemoveVisualizationMesh(m_trimesh_shape);
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
