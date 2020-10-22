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
// MMX spherical wheel subsystem
//
// =============================================================================

#include "chrono_models/vehicle/mmx/MMX_Wheel_Sphere.h"

namespace chrono {
namespace vehicle {
namespace mmx {

// -----------------------------------------------------------------------------
// Static variables - units in mm g s
// -----------------------------------------------------------------------------

const double MMX_Wheel_Sphere::m_radius = 0.5;
const double MMX_Wheel_Sphere::m_width = 1.0;
const double MMX_Wheel_Sphere::m_mass = 1.0;
const ChVector<> MMX_Wheel_Sphere::m_inertia(0.1, 0.1, 0.1);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MMX_Wheel_Sphere::MMX_Wheel_Sphere(const std::string& name) : ChWheel(name) {}


}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
