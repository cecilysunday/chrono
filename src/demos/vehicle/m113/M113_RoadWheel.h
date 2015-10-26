// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// M113 road wheel subsystem.
//
// =============================================================================

#ifndef M113_ROAD_WHEEL_H
#define M113_ROAD_WHEEL_H

#include <string>

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/road_wheel/ChDoubleRoadWheel.h"

namespace m113 {

///
///
///
class M113_RoadWheel : public chrono::vehicle::ChDoubleRoadWheel {
  public:
    virtual ~M113_RoadWheel() {}

    /// Return the mass of the idler wheel body.
    virtual double GetWheelMass() const override { return m_wheel_mass; }
    /// Return the moments of inertia of the idler wheel body.
    virtual const chrono::ChVector<>& GetWheelInertia() override { return m_wheel_inertia; }
    /// Return the radius of the idler wheel.
    virtual double GetWheelRadius() const override { return m_wheel_radius; }
    /// Return the total width of the idler wheel.
    virtual double GetWheelWidth() const override { return m_wheel_width; }
    /// Return the gap width.
    virtual double GetWheelGap() const override { return m_wheel_gap; }

    /// Initialize this road wheel subsystem.
    virtual void Initialize(chrono::ChSharedPtr<chrono::ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            chrono::ChSharedPtr<chrono::ChBody> carrier,        ///< [in] handle to the carrier body
                            const chrono::ChVector<>& location  ///< [in] location relative to the chassis frame
                            ) override;

    void ExportMeshPovray(const std::string& out_dir);

  protected:
    M113_RoadWheel(const std::string& name, chrono::vehicle::VisualizationType vis_type);

    virtual const std::string& GetMeshName() const = 0;
    virtual const std::string& GetMeshFile() const = 0;

    static const double m_wheel_mass;
    static const chrono::ChVector<> m_wheel_inertia;
    static const double m_wheel_radius;
    static const double m_wheel_width;
    static const double m_wheel_gap;

    chrono::vehicle::VisualizationType m_vis_type;
};

class M113_RoadWheelLeft : public M113_RoadWheel {
  public:
    M113_RoadWheelLeft(chrono::vehicle::VisualizationType visType) : M113_RoadWheel("M113_RoadWheelLeft", visType) {}
    ~M113_RoadWheelLeft() {}

    virtual const std::string& GetMeshName() const override { return m_meshName; }
    virtual const std::string& GetMeshFile() const override { return m_meshFile; }

  private:
    static const std::string m_meshName;
    static const std::string m_meshFile;
};

class M113_RoadWheelRight : public M113_RoadWheel {
  public:
    M113_RoadWheelRight(chrono::vehicle::VisualizationType visType) : M113_RoadWheel("M113_RoadWheelRight", visType) {}
    ~M113_RoadWheelRight() {}

    virtual const std::string& GetMeshName() const override { return m_meshName; }
    virtual const std::string& GetMeshFile() const override { return m_meshFile; }

  private:
    static const std::string m_meshName;
    static const std::string m_meshFile;
};

}  // end namespace m113

#endif
