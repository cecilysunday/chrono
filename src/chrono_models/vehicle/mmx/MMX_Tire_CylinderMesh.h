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

#ifndef MMX_TIRE_CYLINDERMESH_H
#define MMX_TIRE_CYLINDERMESH_H

#include "chrono_vehicle/wheeled_vehicle/tire/MMXTire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace mmx {

/// @addtogroup vehicle_models_mmx
/// @{

/// Rigid tire model for the MMX rover
class CH_MODELS_API MMX_Tire_CylinderMesh : public MMXTire {
  public:
    MMX_Tire_CylinderMesh(const std::string& name);

    ~MMX_Tire_CylinderMesh() {}

    virtual double GetRadius() const override { return m_radius; }
    virtual double GetWidth() const override { return m_width; }
    virtual double GetMass() const override { return m_mass; }
    virtual ChVector<> GetInertia() const override { return m_inertia; }

  private:
    virtual void CreateContactMaterial(ChContactMethod contact_method) override;
    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

    static const double m_radius;
    static const double m_width;
    static const double m_mass;
    static const ChVector<> m_inertia;

	static const std::string m_meshFile;
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
};

/// @} vehicle_models_generic

}  // end namespace mmx
}  // end namespace vehicle
}  // end namespace chrono

#endif
