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

#include "chrono_models/vehicle/mmx/MMX_Tire_Simple.h"

namespace chrono {
namespace vehicle {
namespace mmx {

// -----------------------------------------------------------------------------
// Static variables - units in mm g s
// -----------------------------------------------------------------------------

const double MMX_Tire_Simple::m_radius = 107.0;
const double MMX_Tire_Simple::m_width = 53.0;
const double MMX_Tire_Simple::m_mass = 1.0;

const ChVector<> MMX_Tire_Simple::m_inertia(4321.333, 8174.500, 4321.333);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MMX_Tire_Simple::MMX_Tire_Simple(const std::string& name) : MMXTire(name) {
    SetTireType(TireType::SIMPLE);
}

void MMX_Tire_Simple::CreateContactMaterial(ChContactMethod contact_method) {
    MaterialInfo minfo;
    minfo.mu = 0.45f;
    minfo.cr = 0.50f;
    minfo.Y = 2.0e7f;
    m_material = minfo.CreateMaterial(contact_method);
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
