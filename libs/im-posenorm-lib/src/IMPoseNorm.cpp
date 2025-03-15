#include <im-posenorm-lib/IMPoseNorm.h>

#include <im-posenorm-lib/Geometry/IShape.h>

#include <glm/glm.hpp>

#include <cstddef>
#include <numbers>
#include <vector>

#include "HorizontalPoseNormalization.h"
#include "HorizontalPoseUnambiguation.h"
#include "Util.h"
#include "VerticalPoseNormalization.h"

namespace IMPoseNorm {

    const double DegreeToRadian(
            double degree) {

        return degree * std::numbers::pi / 180.0;
    }

    const double RadianToDegree(
            double radian) {

        return radian * 180.0 / std::numbers::pi;
    }

    glm::dmat3 GetRotationAroundAxis(
            double angle,
            const glm::dvec3& axis) {

        double c = std::cos(angle);
        double s = std::sin(angle);
        double t = 1.0 - c;

        glm::dvec3 axisNormalized = glm::normalize(axis);

        double tmp1 = axisNormalized.x * axisNormalized.y * t;
        double tmp2 = axisNormalized.z * s;
        double tmp5 = axisNormalized.y * axisNormalized.z * t;
        double tmp6 = axisNormalized.x * s;
        double tmp3 = axisNormalized.x * axisNormalized.z * t;
        double tmp4 = axisNormalized.y * s;

        return glm::dmat3{
            c + axisNormalized.x * axisNormalized.x * t,
            tmp1 + tmp2,
            tmp3 - tmp4,
            tmp1 - tmp2,
            c + axisNormalized.y * axisNormalized.y * t,
            tmp5 + tmp6,
            tmp3 + tmp4,
            tmp5 - tmp6,
            c + axisNormalized.z * axisNormalized.z * t
        };
    }

    glm::dmat3 NormalizePose(
            Geometry::IShape& shape,
            const glm::dvec3& upAxis,
            const glm::dvec3& horizontalAxis,
            bool useHorizontalPoseUnambiguation) {

        glm::dvec3 centroid = shape.GetCentroid();

        std::vector<double> sizeWeights = GetData<double>(
            shape.GetNormalCount(),
            [&shape](std::size_t index) {
                return shape.GetNormalSizeWeight(index);
            });

        glm::dmat3 rotation = NormalizePoseVertically(
            shape,
            upAxis,
            horizontalAxis,
            centroid,
            sizeWeights);

        rotation = NormalizePoseHorizontally(
                shape,
                upAxis,
                horizontalAxis,
                centroid,
                sizeWeights)
            * rotation;

        if (useHorizontalPoseUnambiguation) {

            rotation = HorizontallyUnambiguatePose(
                    shape,
                    upAxis,
                    horizontalAxis,
                    centroid,
                    sizeWeights)
                * rotation;
        }

        return rotation;
    }
}