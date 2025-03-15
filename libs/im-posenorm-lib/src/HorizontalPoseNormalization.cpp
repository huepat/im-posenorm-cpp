#include "HorizontalPoseNormalization.h"

#include <im-posenorm-lib/IMPoseNorm.h>
#include <im-posenorm-lib/Geometry/IShape.h>
#if IMPOSENORM_PARALLELIZE
    #include <im-posenorm-util/Parallel/Parallel.h>
#endif
#if IMPOSENORM_REPORT_TIMING
    #include <im-posenorm-util/Time/TimeReporter.h>
#endif

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/vector_angle.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <queue>
#include <vector>

#include "Config.h"
#include "Util.h"
#include "WeightedValue.h"

namespace IMPoseNorm {

    glm::dvec3 OrthogonalProject(
            const glm::dvec3& vector,
            const glm::dvec3& axis) {

        return vector
            - glm::dot(
                vector,
                axis)
            * axis;
    }

    double AngleBetween(
            glm::dvec3 vector1,
            glm::dvec3 vector2,
            glm::dvec3 axisOfRotation) {

        return std::atan2(
            glm::dot(
                axisOfRotation,
                glm::cross(
                    vector1,
                    vector2)),
            glm::dot(
                vector1,
                vector2));
    }

    void ProcessAngleToHorizontalAxis(
            std::size_t normalIndex,
            Geometry::IShape& shape,
            const glm::dvec3& upAxis,
            const glm::dvec3& horizontalAxis,
            const std::vector<double>& sizeWeights,
            std::vector<WeightedValue<double>>& weightedAngles) {

        glm::dvec3 normal = shape.GetNormal(normalIndex);

        if (IsNaN(normal)
                || std::isnan(sizeWeights[normalIndex])) {

            return;
        }

        double verticalAngle = glm::abs(
            glm::angle(
                normal,
                upAxis));

        if (verticalAngle < HORIZONTAL_ALIGNMENT_VERTICAL_ANGLE_MIN_THRESHOLD
                || verticalAngle > HORIZONTAL_ALIGNMENT_VERTICAL_ANGLE_MAX_THRESHOLD) {

            return;
        }

        double horizontalAngle = AngleBetween(
            OrthogonalProject(
                normal,
                upAxis),
            horizontalAxis,
            upAxis);

        if (horizontalAngle < 0.0) {

            horizontalAngle += DEGREE_180;
        }

        if (horizontalAngle > DEGREE_90) {

            horizontalAngle -= DEGREE_90;
        }

        weightedAngles.push_back(
            WeightedValue{
                sizeWeights[normalIndex],
                horizontalAngle
            });
    }

#if IMPOSENORM_PARALLELIZE
    std::vector<WeightedValue<double>> GetWeightedAnglesToHorizontalAxis_(
            Geometry::IShape& shape,
            const glm::dvec3& upAxis,
            const glm::dvec3& horizontalAxis,
            const std::vector<double>& sizeWeights) {

        std::vector<WeightedValue<double>> weightedAngles;

        Util::Parallel::Parallel::For<std::vector<WeightedValue<double>>>(
            0,
            shape.GetNormalCount(),
            []() {
                return std::vector<WeightedValue<double>>{};
            },
            [&](
                std::size_t partitionStartIndex,
                std::size_t partitionStopIndex,
                std::vector<WeightedValue<double>> partitionAngles) {

                    for (std::size_t j = partitionStartIndex; j < partitionStopIndex; j++) {

                        ProcessAngleToHorizontalAxis(
                            j,
                            shape,
                            upAxis,
                            horizontalAxis,
                            sizeWeights,
                            partitionAngles);
                    }
            },
            [&weightedAngles](std::vector<WeightedValue<double>>& partitionAngles) {

                MoveInsert(
                    weightedAngles,
                    partitionAngles);
            });

        return weightedAngles;
    }
#else
    std::vector<WeightedValue<double>> GetWeightedAnglesToHorizontalAxis_(
            Geometry::IShape& shape,
            const glm::dvec3& upAxis,
            const glm::dvec3& horizontalAxis,
            const std::vector<double>& sizeWeights) {

        std::vector<WeightedValue<double>> weightedAngles;

        for (std::size_t j = 0; j < shape.GetNormalCount(); j++) {

            ProcessAngleToHorizontalAxis(
                j,
                shape,
                upAxis,
                horizontalAxis,
                sizeWeights,
                weightedAngles);
        }

        return weightedAngles;
    }
#endif

    std::vector<WeightedValue<double>> GetWeightedAnglesToHorizontalAxis(
            Geometry::IShape& shape,
            const glm::dvec3& upAxis,
            const glm::dvec3& horizontalAxis,
            const std::vector<double>& sizeWeights) {

#if IMPOSENORM_REPORT_TIMING
        Util::Time::TimeReporter timeReporter{};
#endif

        std::vector<WeightedValue<double>> weightedAngles = GetWeightedAnglesToHorizontalAxis_(
            shape,
            upAxis,
            horizontalAxis,
            sizeWeights);

#if IMPOSENORM_REPORT_TIMING
        timeReporter.Report("Calculate Weighted Angles to Horizontal Axis");
#endif

        return weightedAngles;
    }

    std::vector<double> CreateHorizontalGrid(
            const std::vector<WeightedValue<double>>& weightedAngles) {

        std::size_t a;

        std::vector<double> grid(
            HORIZONTAL_GRID_SIZE,
            0.0);

        for (const WeightedValue<double>& weightedAngle : weightedAngles) {

            a = static_cast<std::size_t>(
                weightedAngle.Value / HORIZONTAL_RESOLUTION);

            if (a == HORIZONTAL_GRID_SIZE) {

                a = 0;
            }

            grid[a] += weightedAngle.Weight;
        }

        return grid;
    }

    std::vector<WeightedValue<std::vector<std::size_t>>> ClusterHorizontalGrid(
            const std::vector<double>& horizontalGrid) {

        double threshold = HORIZONTAL_PEAK_RATIO
            * *std::max_element(
                horizontalGrid.begin(),
                horizontalGrid.end());

        std::vector<bool> isAlreadyClustered(
            HORIZONTAL_GRID_SIZE,
            false);

        std::vector<WeightedValue<std::vector<std::size_t>>> clusters{};

        std::queue<int> candidateCells{};

        for (std::size_t a = 0; a < HORIZONTAL_GRID_SIZE; a++) {

            if (isAlreadyClustered[a]
                    || horizontalGrid[a] < threshold) {

                continue;
            }

            WeightedValue<std::vector<std::size_t>> cluster{};

            candidateCells.push(a);

            do {
                int candidateCell = candidateCells.front();

                candidateCells.pop();

                if (isAlreadyClustered[candidateCell]) {
                    continue;
                }

                isAlreadyClustered[candidateCell] = true;

                cluster.Value.push_back(candidateCell);

                cluster.Weight += horizontalGrid[candidateCell];

                for (std::ptrdiff_t da = -1; da <= 1; da += 2) {

                    std::ptrdiff_t a2 = candidateCell + da;

                    if (a2 == -1) {
                        a2 = horizontalGrid.size() - 1;
                    }

                    if (a2 == horizontalGrid.size()) {
                        a2 = 0;
                    }

                    if (!isAlreadyClustered[a2]
                        && horizontalGrid[a2] >= threshold) {

                        candidateCells.push(a2);
                    }
                }
            } while (candidateCells.size() > 0);

            clusters.push_back(cluster);
        }

        return clusters;
    }

    double MapToReferenceAngle(
            double angle,
            double referenceAngle) {

        double diff = angle - referenceAngle;

        if (std::abs(diff) > DEGREE_45) {
            if (diff < 0.0) {
                diff += DEGREE_90;
            }
            else {
                diff -= DEGREE_90;
            }
        }

        return referenceAngle + diff;
    }

    double GetAngleOfRotationAroundVerticalAxis(
            const std::vector<double>& horizontalGrid,
            std::vector<WeightedValue<double>>& weightedAngles) {

        std::vector<WeightedValue<std::vector<std::size_t>>> clusters = ClusterHorizontalGrid(
            horizontalGrid);

        const WeightedValue<std::vector<std::size_t>>& mainCluster = *std::max_element(
            clusters.begin(),
            clusters.end(),
            [](
                const WeightedValue<std::vector<std::size_t>>& cluster1,
                const WeightedValue<std::vector<std::size_t>>& cluster2) {

                    return cluster1.Weight < cluster2.Weight;
            });

        double meanClusterAngle = 0.0;
        double weightSum = 0.0;
        double referenceAngle = mainCluster.Value[0] * HORIZONTAL_RESOLUTION;

        for (size_t a : mainCluster.Value) {

            weightSum += horizontalGrid[a];

            meanClusterAngle += horizontalGrid[a]
                * MapToReferenceAngle(
                    a * HORIZONTAL_RESOLUTION,
                    referenceAngle);
        }

        meanClusterAngle /= weightSum;

        std::vector<WeightedValue<double>> neighbouringAngles;

        weightSum = 0.0;

        for (WeightedValue<double>& weightedAngle : weightedAngles) {

            double diff = std::abs(
                meanClusterAngle - weightedAngle.Value);

            if (diff > DEGREE_45) {

                diff -= DEGREE_90;
            }

            if (std::abs(diff) < HORIZONTAL_ANGLE_REFINEMENT_ANGLE_RADIUS) {

                weightSum += weightedAngle.Weight;

                weightedAngle.Value = MapToReferenceAngle(
                    weightedAngle.Value,
                    meanClusterAngle);

                neighbouringAngles.push_back(weightedAngle);
            }
        }

        double result = WeightedMedian<double>(
            weightSum,
            neighbouringAngles,
            [](double value) {
                return value;
            });

        return result;
    }

    glm::dmat3 NormalizePoseHorizontally(
            Geometry::IShape& shape,
            const glm::dvec3& upAxis,
            const glm::dvec3& horizontalAxis,
            const glm::dvec3& centroid,
            const std::vector<double>& sizeWeights) {

#if IMPOSENORM_REPORT_TIMING
        Util::Time::TimeReporter timeReporter{};
#endif

        std::vector<WeightedValue<double>> weightedAngles = GetWeightedAnglesToHorizontalAxis(
            shape,
            upAxis,
            horizontalAxis,
            sizeWeights);

        std::vector<double> horizontalGrid = CreateHorizontalGrid(
            weightedAngles);

        double angle = GetAngleOfRotationAroundVerticalAxis(
            horizontalGrid,
            weightedAngles);

        glm::dmat3 rotation = GetRotationAroundAxis(
            angle,
            upAxis);

        shape.Rotate(
            centroid,
            rotation);

#if IMPOSENORM_REPORT_TIMING
        timeReporter.Report("Horizontal Pose Normalization");
#endif

        return rotation;
    }
}