// This file is part of the Acts project.
//
// Copyright (C) 2016-2018 CERN for the benefit of the Acts project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.


#pragma once
#include "Acts/Definitions/Algebra.hpp"
#include "Acts/MagneticField/MagneticFieldContext.hpp"
#include "Acts/MagneticField/MagneticFieldProvider.hpp"
#include "Acts/Geometry/Volume.hpp"
#include "Acts/Geometry/CuboidVolumeBuilder.hpp"

#include "Acts/Geometry/CuboidVolumeBounds.hpp"
#include "Acts/Geometry/TrackingVolume.hpp"


/// @ingroup MagneticField
/// @brief Constant magnetic field provider
class RestrictedBField final : public Acts::MagneticFieldProvider {
 public:
  struct Cache {
    /// @brief constructor with context
    Cache(const Acts::MagneticFieldContext& /*mcfg*/) {}
  };

  /// @brief Default constructor
  RestrictedBField(const Acts::Vector3& field, const Acts::Vector3& position, const Acts::Vector3& fieldBounds);

  /// @copydoc MagneticFieldProvider::getField(const Vector3&,MagneticFieldProvider::Cache&) const
  Acts::Result<Acts::Vector3> getField(const Acts::Vector3& position, Acts::MagneticFieldProvider::Cache& cache) const override;

  /// @copydoc MagneticFieldProvider::getFieldGradient(const Vector3&,ActsMatrix<3,3>&,MagneticFieldProvider::Cache&) const
  Acts:: Result<Acts::Vector3> getFieldGradient(
      const Acts::Vector3& position, Acts::ActsMatrix<3, 3>& derivative, Acts::MagneticFieldProvider::Cache& cache) const;

  /// @copydoc MagneticFieldProvider::makeCache(const MagneticFieldContext&) const
  Acts::MagneticFieldProvider::Cache makeCache(const Acts::MagneticFieldContext& mctx) const override;


  /// @brief check whether given 3D position is inside look-up domain
  ///
  /// @param [in] position global 3D position
  /// @return @c true if position is inside the defined look-up grid,
  ///         otherwise @c false
  bool isInside(const Acts::Vector3& position) const;

 private:

    Acts::Vector3 m_position;
    Acts::Vector3 m_fieldBounds;

    std::shared_ptr<Acts::TrackingVolume> m_magVolume;
    /// magnetic field vector
    const Acts::Vector3 m_BField;
};