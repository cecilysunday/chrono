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

#include "chrono_models/vehicle/mmx/MMX_Tire_Cylinder.h"

namespace chrono {
namespace vehicle {
namespace mmx {

// -----------------------------------------------------------------------------
// Static variables - units in mm g s
// -----------------------------------------------------------------------------

const double MMX_Tire_Cylinder::m_radius = 50.0;
const double MMX_Tire_Cylinder::m_width = 100.0;		
const double MMX_Tire_Cylinder::m_mass = 1000.0;
const ChVector<> MMX_Tire_Cylinder::m_inertia(1458333.33, 1458333.33, 1250000.0);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MMX_Tire_Cylinder::MMX_Tire_Cylinder(const std::string& name) : MMXTire(name) {
    SetTireType(TireType::CYLINDER);
}

void MMX_Tire_Cylinder::CreateContactMaterial(ChContactMethod contact_method) {
    MaterialInfo minfo;
    minfo.mu = 0.45f;
    minfo.cr = 0.50f;
    minfo.Y = 7.0e8f;
    minfo.nu = 0.24f;
    m_material = minfo.CreateMaterial(contact_method);
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
