
#include "Acts/Definitions/Algebra.hpp"
#include "Acts/MagneticField/MagneticFieldContext.hpp"
#include "Acts/MagneticField/MagneticFieldProvider.hpp"
#include "Acts/Geometry/Volume.hpp"
#include "Acts/Geometry/CuboidVolumeBuilder.hpp"
#include "Acts/Geometry/CuboidVolumeBounds.hpp"
#include "Acts/Geometry/TrackingVolume.hpp"

#include "../include/RestrictedBField.h"


/// @brief Default constructor
RestrictedBField::RestrictedBField(const Acts::Vector3& field, const Acts::Vector3& position, const Acts::Vector3& fieldBounds)
    : m_BField(field),
    m_position(position),
    m_fieldBounds(fieldBounds),
    m_magVolume(std::make_shared<Acts::TrackingVolume>(
        Acts::Transform3(Acts::Translation3{position}),
        std::static_pointer_cast<Acts::VolumeBounds>(
            std::const_pointer_cast<Acts::CuboidVolumeBounds>(
                std::make_shared<const Acts::CuboidVolumeBounds>(fieldBounds.x(), fieldBounds.y(), fieldBounds.z()))),
        "MagneticFieldVolume")) {}

/// @copydoc MagneticFieldProvider::getField(const Vector3&,MagneticFieldProvider::Cache&) const
Acts::Result<Acts::Vector3> RestrictedBField::getField(const Acts::Vector3& position, Acts::MagneticFieldProvider::Cache& cache) const  {
(void)cache;
if (m_magVolume->inside(position)) {
    return Acts::Result<Acts::Vector3>::success(m_BField);
} else {
    return Acts::Result<Acts::Vector3>::success(Acts::Vector3::Zero());
}
}

/// @copydoc MagneticFieldProvider::getFieldGradient(const Vector3&,ActsMatrix<3,3>&,MagneticFieldProvider::Cache&) const
Acts::Result<Acts::Vector3> RestrictedBField::getFieldGradient(
    const Acts::Vector3& position, Acts::ActsMatrix<3, 3>& derivative,
    Acts::MagneticFieldProvider::Cache& cache) const {
(void)position;
(void)derivative;
(void)cache;
return Acts::Result<Acts::Vector3>::success(m_BField);
}

/// @copydoc MagneticFieldProvider::makeCache(const MagneticFieldContext&) const
Acts::MagneticFieldProvider::Cache RestrictedBField::makeCache(
    const Acts::MagneticFieldContext& mctx) const {
return Acts::MagneticFieldProvider::Cache(std::in_place_type<Cache>, mctx);
}

/// @brief check whether given 3D position is inside look-up domain
///
/// @param [in] position global 3D position
/// @return @c true if position is inside the defined look-up grid,
///         otherwise @c false
bool RestrictedBField::isInside(const Acts::Vector3& position) const {
    return m_magVolume->inside(position);
}
