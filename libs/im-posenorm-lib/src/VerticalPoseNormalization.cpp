#include "VerticalPoseNormalization.h"

#include <im-posenorm-lib/IMPoseNorm.h>
#include <im-posenorm-lib/Geometry/IShape.h>
#include <im-posenorm-util/Statistics/Statistics.h>
#if IMPOSENORM_REPORT_TIMING
    #include <im-posenorm-util/Time/TimeReporter.h>
#endif

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/vector_angle.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <queue>
#include <utility>
#include <vector>

#include "Config.h"
#include "Grid2D.h"
#include "Util.h"
#include "WeightedValue.h"

namespace IMPoseNorm {

    using WeightedNormal = WeightedValue<glm::dvec3>;
    using WeightedNormals = WeightedValue<std::vector<WeightedNormal>>;
    using VerticalGrid = Grid2D<WeightedNormals>;

    struct NormalCluster {
    public:
        Util::Statistics::VectorStatistics Center{};
        std::vector<WeightedNormal> Normals;
    };

    void HandleGridPole(
            VerticalGrid& verticalGrid) {

        WeightedNormals& poleCell = verticalGrid.Get(0, 0);

        for (std::size_t a = 1; a < VERTICAL_GRID_SIZE_0; a++) {

            WeightedNormals& cell = verticalGrid.Get(0, a);

            poleCell.Weight += cell.Weight;

            MoveInsert(
                poleCell.Value,
                cell.Value);

            cell.Weight = 0.0;
        }
    }

    std::vector<NormalCluster> MergeAntipodalClusters(
            std::vector<NormalCluster>& clusters) {

        std::vector<NormalCluster> mergedClusters{};

        for (NormalCluster& cluster : clusters) {

            bool found = false;

            for (NormalCluster& mergedCluster : mergedClusters) {

                if (std::abs(
                            glm::angle(
                                cluster.Center.GetMean(),
                                mergedCluster.Center.GetMean())
                            - DEGREE_180) 
                        < VERTICAL_GRID_CLUSTERING_THRESHOLD) {

                    found = true;

                    MoveInsert(
                        mergedCluster.Normals,
                        cluster.Normals);

                    break;
                }
            }

            if (!found) {

                NormalCluster mergedCluster{};

                mergedCluster.Center = cluster.Center;

                MoveInsert(
                    mergedCluster.Normals,
                    cluster.Normals);

                mergedClusters.push_back(mergedCluster);
            }
        }

        return mergedClusters;
    }

    std::vector<NormalCluster> ClusterNormals(
            const WeightedNormals& verticalGridCell) {

        std::vector<NormalCluster> clusters{};

        for (const WeightedNormal& weightedNormal : verticalGridCell.Value) {

            bool found = false;

            for (NormalCluster& cluster : clusters) {

                double angle = glm::angle(
                    weightedNormal.Value,
                    cluster.Center.GetMean());

                if (angle < VERTICAL_GRID_CLUSTERING_THRESHOLD) {

                    found = true;

                    cluster.Normals.push_back(weightedNormal);

                    cluster.Center.Update(
                        weightedNormal.Weight * weightedNormal.Value);

                    break;
                }
            }

            if (!found) {

                NormalCluster cluster{};

                cluster.Center.Update(
                    weightedNormal.Weight * weightedNormal.Value);

                cluster.Normals.push_back(weightedNormal);

                clusters.push_back(cluster);
            }
        }

        return MergeAntipodalClusters(clusters);
    }

    // ToDo: Parallelize!
    void ResolveSecondaryNormalClusters(
            VerticalGrid& verticalGrid) {

#if IMPOSENORM_REPORT_TIMING
        Util::Time::TimeReporter timeReporter{};
#endif

        for (std::size_t a = 0; a < VERTICAL_GRID_SIZE_0; a++) {
            for (std::size_t i = 0; i < VERTICAL_GRID_SIZE_1; i++) {

                WeightedNormals& cell = verticalGrid.Get(a, i);

                std::vector<NormalCluster> clusters = ClusterNormals(cell);

                if (clusters.size() < 1) {

                    continue;
                }

                NormalCluster& mainCluster = *std::max_element(
                    clusters.begin(),
                    clusters.end(),
                    [](
                        const NormalCluster& cluster1,
                        const NormalCluster& cluster2) {

                            return cluster1.Normals.size() < cluster2.Normals.size();
                    });

                cell.Weight = std::accumulate(
                    mainCluster.Normals.begin(),
                    mainCluster.Normals.end(),
                    0.0,
                    [](
                        double value1,
                        const WeightedNormal& value2) {

                            return value1 + value2.Weight;
                    });

                cell.Value = std::move(mainCluster.Normals);
            }
        }

#if IMPOSENORM_REPORT_TIMING
        timeReporter.Report("Resolve Secondary Normal Clusters");
#endif
    }

    // ToDo: Parallelize!
    VerticalGrid CreateVerticalGrid(
            const glm::dvec3& upAxis,
            const glm::dvec3& horizontalAxis,
            const std::vector<double>& sizeWeights,
            const std::vector<glm::dvec3>& normals) {

#if IMPOSENORM_REPORT_TIMING
        Util::Time::TimeReporter timeReporter{};
#endif

        std::size_t a;
        std::size_t i;
        double azimuth;
        double inclination;

        glm::dvec3 horizontalAxis2 = glm::cross(
            upAxis,
            horizontalAxis);

        VerticalGrid grid{
            VERTICAL_GRID_SIZE_0,
            VERTICAL_GRID_SIZE_1
        };

        for (std::size_t j = 0; j < normals.size(); ++j) {

            if (IsNaN(normals[j])
                    || std::isnan(sizeWeights[j])) {

                continue;
            }

            inclination = DEGREE_90
                - std::abs(
                    std::acos(
                        glm::dot(
                            normals[j],
                            upAxis))
                    - DEGREE_90);

            if (std::isnan(inclination)
                    || std::abs(inclination) > VERTICAL_ALIGNMENT_ANGLE_RADIUS) {

                continue;
            }

            azimuth = std::abs(
                std::abs(
                    std::atan2(
                        glm::dot(
                            normals[j],
                            horizontalAxis),
                        glm::dot(
                            normals[j],
                            horizontalAxis2)))
                - DEGREE_90);

            a = static_cast<std::size_t>(azimuth / VERTICAL_RESOLUTION);
            i = static_cast<std::size_t>(inclination / VERTICAL_RESOLUTION);

            if (a == VERTICAL_GRID_SIZE_0) {

                a = 0;
            }

            WeightedNormals& cell = grid.Get(a, i);

            cell.Weight += sizeWeights[j];

            cell.Value.push_back(
                WeightedValue<glm::dvec3>{
                    sizeWeights[j],
                    normals[j]
                });
        }

        HandleGridPole(grid);

#if IMPOSENORM_REPORT_TIMING
        timeReporter.Report("Create Vertical Grid");
#endif

        ResolveSecondaryNormalClusters(grid);

        return grid;
    }

    void AddToCluster(
            const GridCoordinate2D& gridCoordinate,
            WeightedNormals& cluster,
            VerticalGrid& verticalGrid) {

        WeightedNormals& cell = verticalGrid.Get(gridCoordinate);

        cluster.Weight += cell.Weight;

        MoveInsert(
            cluster.Value,
            cell.Value);
    }

    std::vector<GridCoordinate2D> HandlePole(
            const GridCoordinate2D& gridCoordinate,
            WeightedNormals& cluster,
            Grid2D<BoolWrapper>& isAlreadyClustered,
            VerticalGrid& verticalGrid) {

        std::vector<GridCoordinate2D> candidateGridCoordinates {
            gridCoordinate
        };

        if (gridCoordinate[0] == 0
                && gridCoordinate[1] == 0) {

            for (std::size_t a = 0; a < VERTICAL_GRID_SIZE_1; a++) {

                GridCoordinate2D candidateGridCoordinate{ a, 0 };

                isAlreadyClustered.Set(
                    candidateGridCoordinate,
                    true);

                AddToCluster(
                    candidateGridCoordinate,
                    cluster,
                    verticalGrid);

                candidateGridCoordinates.push_back(candidateGridCoordinate);
            }
        }

        return candidateGridCoordinates;
    }

    std::vector<WeightedNormals> ClusterVerticalGridCells(
            VerticalGrid& verticalGrid) {

        double threshold = verticalGrid
            .GetMax(
                [](
                    const WeightedNormals& cell1,
                    const WeightedNormals& cell2) {

                        return cell1.Weight < cell2.Weight;
                })
            .Weight * VERTICAL_PEAK_RATIO;

        Grid2D<BoolWrapper> isAlreadyClustered {
            VERTICAL_GRID_SIZE_0,
            VERTICAL_GRID_SIZE_1
        };

        std::queue<GridCoordinate2D> candidateGridCoordinates{};

        std::vector<WeightedNormals> clusters{};

        for (std::size_t a = 0; a < VERTICAL_GRID_SIZE_0; a++) {
            for (std::size_t i = 0; i < VERTICAL_GRID_SIZE_1; i++) {

                if (isAlreadyClustered.Get(a, i).Value) {
                    continue;
                }

                WeightedNormals& cell = verticalGrid.Get(a, i);

                if (cell.Weight < threshold) {
                    continue;
                }

                WeightedNormals cluster{};

                candidateGridCoordinates.push(
                    GridCoordinate2D{ a, i });

                do {
                    GridCoordinate2D candidateGridCoordinate = candidateGridCoordinates.front();

                    candidateGridCoordinates.pop();

                    if (isAlreadyClustered.Get(candidateGridCoordinate).Value) {

                        continue;
                    }

                    isAlreadyClustered.Set(
                        candidateGridCoordinate,
                        true);

                    AddToCluster(
                        candidateGridCoordinate,
                        cluster,
                        verticalGrid);

                    std::vector<GridCoordinate2D> candidateGridCoordinatesToCheckNeighbours = HandlePole(
                        candidateGridCoordinate,
                        cluster,
                        isAlreadyClustered,
                        verticalGrid);

                    for (const GridCoordinate2D& candidateGridCoordinateToCheckNeighbours : candidateGridCoordinatesToCheckNeighbours) {

                        for (std::ptrdiff_t da = -1; da <= 1; ++da) {
                            for (std::ptrdiff_t di = -1; di <= 1; ++di) {

                                if (da == 0 
                                        && di == 0) {

                                    continue;
                                }

                                std::ptrdiff_t a2 = candidateGridCoordinateToCheckNeighbours[0] + da;
                                std::ptrdiff_t i2 = candidateGridCoordinateToCheckNeighbours[1] + di;

                                if (i2 < 0 
                                        || i2 >= VERTICAL_GRID_SIZE_1) {

                                    continue;
                                }

                                if (a2 == -1) {

                                    a2 = VERTICAL_GRID_SIZE_0 - 1;
                                }

                                if (a2 == VERTICAL_GRID_SIZE_0) {

                                    a2 = 0;
                                }

                                std::size_t a3 = static_cast<std::size_t>(a2);
                                std::size_t i3 = static_cast<std::size_t>(i2);

                                if (!isAlreadyClustered.Get(a3, i3).Value
                                        && verticalGrid.Get(a3, i3).Weight >= threshold) {

                                    candidateGridCoordinates.push(
                                        GridCoordinate2D{ a3, i3 });
                                }
                            }
                        }
                    }

                } while (candidateGridCoordinates.size() > 0);

                clusters.push_back(cluster);
            } 
        }

        return clusters;
    }

    void NormalizeOrientation(
            WeightedNormal& weightedNormal,
            const glm::dvec3& upAxis) {

        if (glm::dot(
                upAxis,
                weightedNormal.Value)
            < 0.0) {

            weightedNormal.Value *= -1.0;
        }
    }

    // ToDo: Parallelize!
    glm::dvec3 GetOrientedUpAxis(
            const glm::dvec3& upAxis,
            VerticalGrid& verticalGrid,
            const std::vector<double>& sizeWeights,
            const std::vector<glm::dvec3>& normals) {

#if IMPOSENORM_REPORT_TIMING
        Util::Time::TimeReporter timeReporter{};
#endif

        std::vector<WeightedNormals> clusters = ClusterVerticalGridCells(verticalGrid);

        WeightedNormals& mainCluster = *std::max_element(
            clusters.begin(),
            clusters.end(),
            [](
                const WeightedNormals& cluster1,
                const WeightedNormals& cluster2) {

                    return cluster1.Weight < cluster2.Weight;
            });

        double weightSum = 0.0;
        glm::dvec3 orientedUpAxis{ 0.0 };

        for (WeightedNormal& weightedNormal : mainCluster.Value) {

            NormalizeOrientation(
                weightedNormal,
                upAxis);

            weightSum += weightedNormal.Weight;
            orientedUpAxis += weightedNormal.Weight * weightedNormal.Value;
        }

        orientedUpAxis *= (1.0 / weightSum);

        orientedUpAxis = glm::normalize(orientedUpAxis);

        weightSum = 0.0;

        std::vector<WeightedNormal> neighbouringNormals{};

        for (size_t j = 0; j < normals.size(); j++) {

            if (std::isnan(sizeWeights[j])) {

                continue;
            }

            double angle = std::abs(
                glm::angle(
                    normals[j],
                    orientedUpAxis));

            if (angle < VERTICAL_AXIS_REFINEMENT_ANGLE_RADIUS
                    || std::abs(angle - DEGREE_180) < VERTICAL_AXIS_REFINEMENT_ANGLE_RADIUS) {

                weightSum += sizeWeights[j];

                WeightedNormal weightedNormal = WeightedNormal{
                    sizeWeights[j],
                    normals[j]
                };

                NormalizeOrientation(
                    weightedNormal,
                    upAxis);

                neighbouringNormals.push_back(weightedNormal);
            }
        }

        orientedUpAxis = glm::dvec3{
            WeightedMedian<glm::dvec3>(
                weightSum,
                neighbouringNormals,
                [](const glm::dvec3& normal) {
                    return normal.x;
                }),
            WeightedMedian<glm::dvec3>(
                weightSum,
                neighbouringNormals,
                [](const glm::dvec3& normal) {
                    return normal.y;
                }),
            WeightedMedian<glm::dvec3>(
                weightSum,
                neighbouringNormals,
                [](const glm::dvec3& normal) {
                    return normal.z;
                })
        };

        orientedUpAxis = glm::normalize(orientedUpAxis);

#if IMPOSENORM_REPORT_TIMING
        timeReporter.Report("Get Oriented Up Axis");
#endif

        return orientedUpAxis;
    }

    glm::dmat3 GetRotationBetween(
            const glm::dvec3& direction1,
            const glm::dvec3& direction2) {

        glm::dvec3 axis = glm::normalize(
            glm::cross(
                direction1,
                direction2));

        double angle = std::acos(
            glm::dot(
                direction1,
                direction2));

        return GetRotationAroundAxis(
            angle,
            axis);
    }

    glm::dmat3 NormalizePoseVertically(
            Geometry::IShape& shape,
            const glm::dvec3& upAxis,
            const glm::dvec3& horizontalAxis,
            const glm::dvec3& centroid,
            const std::vector<double>& sizeWeights) {

#if IMPOSENORM_REPORT_TIMING
        Util::Time::TimeReporter timeReporter{};
#endif

        std::vector<glm::dvec3> normals = GetData<glm::dvec3>(
            shape.GetNormalCount(),
            [&shape](std::size_t index) {
                return shape.GetNormal(index);
            });

        VerticalGrid verticalGrid = CreateVerticalGrid(
            upAxis,
            horizontalAxis,
            sizeWeights,
            normals);

        glm::dvec3 orientedUpAxis = GetOrientedUpAxis(
            upAxis,
            verticalGrid,
            sizeWeights,
            normals);

        glm::dmat3 rotation = GetRotationBetween(
            orientedUpAxis,
            upAxis);

        shape.Rotate(
            centroid,
            rotation);

#if IMPOSENORM_REPORT_TIMING
        timeReporter.Report("Vertical Pose Normalization");
#endif

        return rotation;
    }
}