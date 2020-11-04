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

#include "chrono_models/vehicle/mmx/MMX_Tire_Paddle.h"

namespace chrono {
namespace vehicle {
namespace mmx {

// -----------------------------------------------------------------------------
// Static variables - units in mm g s
// -----------------------------------------------------------------------------

const double MMX_Tire_Paddle::m_radius = 107.0;
const double MMX_Tire_Paddle::m_width = 53.0;
const double MMX_Tire_Paddle::m_mass = 838.270; // 1.0

const ChVector<> MMX_Tire_Paddle::m_inertia(2662580.480, 4980271.335, 2662580.480); // (3176.280, 5941.130, 3176.280);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MMX_Tire_Paddle::MMX_Tire_Paddle(const std::string& name) : MMXTire(name) {
    SetTireType(TireType::PADDLE);
}

void MMX_Tire_Paddle::CreateContactMaterial(ChContactMethod contact_method) {
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
