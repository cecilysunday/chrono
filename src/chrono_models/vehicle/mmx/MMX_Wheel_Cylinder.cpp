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
// MMX cylindrical wheel subsystem
//
// =============================================================================

#include "chrono_models/vehicle/mmx/MMX_Wheel_Cylinder.h"

namespace chrono {
namespace vehicle {
namespace mmx {

// -----------------------------------------------------------------------------
// Static variables - units in mm g s
// -----------------------------------------------------------------------------

const double MMX_Wheel_Cylinder::m_radius = 0.5;
const double MMX_Wheel_Cylinder::m_width = 1.0;
const double MMX_Wheel_Cylinder::m_mass = 1.0;
const ChVector<> MMX_Wheel_Cylinder::m_inertia(0.145833333, 0.145833333, 0.125);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MMX_Wheel_Cylinder::MMX_Wheel_Cylinder(const std::string& name) : ChWheel(name) {}


}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
