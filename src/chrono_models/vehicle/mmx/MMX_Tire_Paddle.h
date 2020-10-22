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

#ifndef MMX_TIRE_PADDLE_H
#define MMX_TIRE_PADDLE_H

#include "chrono_vehicle/wheeled_vehicle/tire/MMXTire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace mmx {

/// @addtogroup vehicle_models_mmx
/// @{

/// Rigid tire model for the MMX rover
class CH_MODELS_API MMX_Tire_Paddle : public MMXTire {
  public:
    MMX_Tire_Paddle(const std::string& name);

    ~MMX_Tire_Paddle() {}

    virtual double GetRadius() const override { return m_radius; }
    virtual double GetWidth() const override { return m_width; }
    virtual double GetMass() const override { return m_mass; }
    virtual ChVector<> GetInertia() const override { return m_inertia; }

  private:
    virtual void CreateContactMaterial(ChContactMethod contact_method) override;

    static const double m_radius;
    static const double m_width;
    static const double m_mass;
    static const ChVector<> m_inertia;
};

/// @} vehicle_models_generic

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono

#endif
