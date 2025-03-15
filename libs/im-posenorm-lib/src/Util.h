#pragma once

#if IMPOSENORM_REPORT_TIMING
    #include <im-posenorm-util/Time/TimeReporter.h>
#endif

#include <glm/glm.hpp>

#include <algorithm>
#include <cstddef>
#include <functional>
#include <iterator>
#include <stdexcept>
#include <vector>

#include "WeightedValue.h"

namespace IMPoseNorm {

    bool IsNaN(
        const glm::dvec3& value);

    template <typename T>
    void MoveInsert(
            std::vector<T>& destination,
            std::vector<T>& source) {

        destination.insert(
            destination.end(),
            std::make_move_iterator(
                source.begin()),
            std::make_move_iterator(
                source.end()));
    }

    template <typename T>
    std::vector<T> GetData(
            std::size_t dataCount,
            std::function<T(std::size_t index)> dataCallback) {

        std::vector<T> data(dataCount);

        for (std::size_t j = 0; j < dataCount; j++) {

            data[j] = dataCallback(j);
        }

        return data;
    }

    template <typename T>
    double WeightedMedian(
            double totalWeightSum,
            std::vector<WeightedValue<T>>& weightedValues,
            std::function<double(const T&)> valueCallback) {

        std::sort(
            weightedValues.begin(),
            weightedValues.end(),
            [valueCallback](
                const WeightedValue<T>& value1,
                const WeightedValue<T>& value2) {

                    return valueCallback(value1.Value) < valueCallback(value2.Value);
            });

        double weightSum = 0.0;
        double halfTotalWeightSum = totalWeightSum / 2.0;

        for (const WeightedValue<T>& weightedValue : weightedValues) {

            weightSum += weightedValue.Weight;

            if (weightSum >= halfTotalWeightSum) {

                return valueCallback(weightedValue.Value);
            }
        }

        throw std::runtime_error("");
    }
}