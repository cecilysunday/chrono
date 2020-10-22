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
// MMX simple DLR tire subsystem
//
// =============================================================================

#include "chrono_models/vehicle/mmx/MMX_Tire_DLR1.h"
#include "chrono/core/ChGlobal.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace mmx {

// -----------------------------------------------------------------------------
// Static variables - units in mm g s
// -----------------------------------------------------------------------------

const double MMX_Tire_DLR1::m_radius = 104.0;
const double MMX_Tire_DLR1::m_width = 53.0;
const double MMX_Tire_DLR1::m_mass = 2816.763;

const ChVector<> MMX_Tire_DLR1::m_inertia(5242803.651, 9168366.293, 5242803.651);

const std::string MMX_Tire_DLR1::m_meshFile("vehicle/mmx/MMX_Tire_DLR1_Mesh6.obj");

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MMX_Tire_DLR1::MMX_Tire_DLR1(const std::string& name) : MMXTire(name) {
	vehicle::SetDataPath(GetChronoDataPath());
    SetMeshFilename(vehicle::GetDataFile(m_meshFile), 0);
}

void MMX_Tire_DLR1::CreateContactMaterial(ChContactMethod contact_method) {
    MaterialInfo minfo;
    minfo.mu = 0.45f;
    minfo.cr = 0.50f;
    minfo.Y = 7.0e8f;
    m_material = minfo.CreateMaterial(contact_method);
}

void MMX_Tire_DLR1::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE) {
        return;
    } else {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile, m_meshFile);
    }
}

void MMX_Tire_DLR1::RemoveVisualizationAssets() {
    MMXTire::RemoveVisualizationAssets();
    RemoveVisualizationMesh(m_trimesh_shape);
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
