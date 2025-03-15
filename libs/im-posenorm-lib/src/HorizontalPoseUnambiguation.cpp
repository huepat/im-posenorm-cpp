#include "HorizontalPoseUnambiguation.h"

#include <im-posenorm-lib/Geometry/AABox.h>
#include <im-posenorm-lib/Geometry/IShape.h>
#if IMPOSENORM_REPORT_TIMING
    #include <im-posenorm-util/Time/TimeReporter.h>
#endif

#include <glm/glm.hpp>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <vector>

#include "Config.h"

namespace IMPoseNorm {

    bool ApproximateEquals(
            double value1,
            double value2) {

        return std::abs(value2 - value1) < EPSILON;
    }

    bool ApproximateEquals(
            const glm::dvec3& value1,
            const glm::dvec3& value2) {

        return ApproximateEquals(value1.x, value2.x)
            && ApproximateEquals(value1.y, value2.y)
            && ApproximateEquals(value1.z, value2.z);
    }

    bool IsUnambiguable(
            const glm::dvec3& upAxis,
            const glm::dvec3& horizontalAxis) {

        return !(ApproximateEquals(
                    glm::abs(upAxis),
                    glm::dvec3(1.0, 0.0, 0.0))
                || ApproximateEquals(
                    glm::abs(upAxis),
                    glm::dvec3(1.0, 0.0, 0.0))
                || ApproximateEquals(
                    glm::abs(upAxis),
                    glm::dvec3(1.0, 0.0, 0.0)))
            && !(ApproximateEquals(
                    glm::abs(horizontalAxis),
                    glm::dvec3(1.0, 0.0, 0.0))
                || ApproximateEquals(
                    glm::abs(horizontalAxis),
                    glm::dvec3(1.0, 0.0, 0.0))
                || ApproximateEquals(
                    glm::abs(horizontalAxis),
                    glm::dvec3(1.0, 0.0, 0.0)));
    }

    glm::dmat3 OrientHorizontalAxisAlongLongerSideOfBBox(
            Geometry::IShape& shape,
            const glm::dvec3& upAxis,
            const glm::dvec3& horizontalAxis,
            const glm::dvec3& centroid) {

        glm::dvec3 size = shape.GetBBox()->GetSize();

        if (glm::dot(
                size,
                horizontalAxis)
            >= glm::dot(
                size,
                glm::cross(
                    upAxis,
                    horizontalAxis))) {

            return glm::dmat3{};
        }

        glm::dmat3 rotation = GetRotationAroundAxis(
            DEGREE_90,
            upAxis);

        shape.Rotate(
            centroid,
            rotation);

        shape.UpdateBBox();

        return rotation;
    }

    // ToDo: Parallelize!
    std::tuple<double, double> GetDensityInProximalSectionsOfBBox(
            Geometry::IShape& shape,
            const glm::dvec3& horizontalAxis,
            const std::vector<double>& sizeWeights) {

#if IMPOSENORM_REPORT_TIMING
        Util::Time::TimeReporter timeReporter{};
#endif

        std::shared_ptr<const Geometry::AABox> bBox = shape.GetBBox();

        double positionAlongHorizontalAxis;

        double size = glm::dot(
            bBox->GetSize(),
            horizontalAxis);

        double lowerBound = glm::dot(
                bBox->GetMin(),
                horizontalAxis)
            + UNAMBIGUOUS_ALIGN_SIZE_FRACTION * size;

        double upperBound = glm::dot(
                bBox->GetMax(),
                horizontalAxis)
            - UNAMBIGUOUS_ALIGN_SIZE_FRACTION * size;

        std::tuple<double, double> densities{
            0.0,
            0.0
        };

        for (std::size_t j = 0; j < shape.GetNormalCount(); j++) {

            positionAlongHorizontalAxis = glm::dot(
                shape.GetReferencePointForNormalVector(j),
                horizontalAxis);

            if (positionAlongHorizontalAxis <= lowerBound) {

                std::get<0>(densities) += shape.GetNormalSizeWeight(j);
            }
            else if (positionAlongHorizontalAxis >= upperBound) {

                std::get<1>(densities) += shape.GetNormalSizeWeight(j);
            }
        }

#if IMPOSENORM_REPORT_TIMING
        timeReporter.Report("Get Densities in Proximal Sections of BBox");
#endif

        return densities;
    }

    glm::dmat3 OrientPositiveHorizontalAxisToMoreDenseProximalSectionOfBBox(
            Geometry::IShape& shape,
            const glm::dvec3& upAxis,
            const glm::dvec3& horizontalAxis,
            const glm::dvec3& centroid,
            const std::vector<double>& sizeWeights) {

        std::tuple<double, double> densities = GetDensityInProximalSectionsOfBBox(
            shape,
            horizontalAxis,
            sizeWeights);

        if (std::get<0>(densities) <= std::get<1>(densities)) {

            return glm::dmat3{};
        }

        glm::dmat3 rotation = GetRotationAroundAxis(
            DEGREE_180,
            upAxis);

        shape.Rotate(
            centroid,
            rotation);

        return rotation;
    }

    glm::dmat3 HorizontallyUnambiguatePose(
            Geometry::IShape& shape,
            const glm::dvec3& upAxis,
            const glm::dvec3& horizontalAxis,
            const glm::dvec3& centroid,
            const std::vector<double>& sizeWeights) {

#if IMPOSENORM_REPORT_TIMING
        Util::Time::TimeReporter timeReporter{};
#endif

        if (IsUnambiguable(
                upAxis,
                horizontalAxis)) {

            throw std::runtime_error(
                "Unambiguation of poses is currently only supported for axis being {(0,0,+-1), (0,+-1,0), (+-1,0,0)}.");
        }

        shape.UpdateBBox();

        glm::dmat3 rotation = OrientHorizontalAxisAlongLongerSideOfBBox(
            shape,
            upAxis,
            horizontalAxis,
            centroid);

        rotation = OrientPositiveHorizontalAxisToMoreDenseProximalSectionOfBBox(
                shape,
                upAxis,
                horizontalAxis,
                centroid,
                sizeWeights)
            * rotation;

#if IMPOSENORM_REPORT_TIMING
        timeReporter.Report("Horizontal Pose Unambiguation");
#endif

        return rotation;
    }
}