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
// MMX rover wheel subsystem
//
// =============================================================================

#include "chrono_models/vehicle/mmx/MMX_Wheel.h"

namespace chrono {
namespace vehicle {
namespace mmx {

// -----------------------------------------------------------------------------
// Static variables - units in mm g s
// -----------------------------------------------------------------------------

const double MMX_Wheel::m_radius = 70.0;
const double MMX_Wheel::m_width = 53.0;
const double MMX_Wheel::m_mass = 979.0;
const ChVector<> MMX_Wheel::m_inertia(1000000.0, 2000000.0, 1000000.0);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MMX_Wheel::MMX_Wheel(const std::string& name) : ChWheel(name) {}


}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
