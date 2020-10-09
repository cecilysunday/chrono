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
// MMX rover rigid tire subsystem
//
// =============================================================================

#include "chrono_models/vehicle/mmx/MMX_SimpleTire.h"

namespace chrono {
namespace vehicle {
namespace mmx {

// -----------------------------------------------------------------------------
// Static variables - units in mm g s
// -----------------------------------------------------------------------------

const double MMX_SimpleTire::m_radius = 107.0;
const double MMX_SimpleTire::m_width = 53.0;
const double MMX_SimpleTire::m_mass = 1308.525;

const ChVector<> MMX_SimpleTire::m_inertia(5654572.105, 10696536.487, 5654572.105);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MMX_SimpleTire::MMX_SimpleTire(const std::string& name) : ChRigidTire(name) {}

void MMX_SimpleTire::CreateContactMaterial(ChContactMethod contact_method) {
    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.1f;
    minfo.Y = 2e7f;
    m_material = minfo.CreateMaterial(contact_method);
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
