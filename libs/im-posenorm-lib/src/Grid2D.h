#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <utility>
#include <vector>

namespace IMPoseNorm {

    using GridCoordinate2D = std::array<std::size_t, 2>;

    struct BoolWrapper {
    public:
        bool Value;

        BoolWrapper() :
                    Value(false) {
        }

        BoolWrapper(
                bool value) :
                    Value(value) {
        }
    };

    template <typename T>
    class Grid2D {

    public:
        using Comparator = std::function<bool(
            const T& value1,
            const T& value2)>;

    private:
        const std::array<std::size_t, 2> size;
        std::vector<T> data;

    public:

        Grid2D(
                std::size_t size0,
                std::size_t size1) :
                    size(
                        std::array<std::size_t, 2> {
                            size0,
                            size1
                        }),
                    data(
                        std::vector<T>(size0 * size1)) {
        }

        std::size_t GetSize(
                std::size_t dimension) const {

            return this->size[dimension];
        }

        void Set(
                std::size_t index0,
                std::size_t index1,
                const T& value) {

            this->data[index0 * index1 + index1] = value;
        }

        void Set(
                const GridCoordinate2D& gridCoordinate,
                const T& value) {

            this->Set(
                gridCoordinate[0],
                gridCoordinate[1],
                value);
        }

        void Set(
                std::size_t index0,
                std::size_t index1,
                T&& value) {

            this->data[index0 * index1 + index1] = std::move(value);
        }

        void Set(
                const GridCoordinate2D& gridCoordinate,
                T&& value) {

            this->Set(
                gridCoordinate[0],
                gridCoordinate[1],
                std::move(value));
        }

        T& Get(
                std::size_t index0,
                std::size_t index1) {

            return this->data[index0 * index1 + index1];
        }

        T& Get(
                const GridCoordinate2D& gridCoordinate) {

            return this->Get(
                gridCoordinate[0],
                gridCoordinate[1]);
        }

        T& GetMax(
                Comparator comparator) {

            T& maxValue = *std::max_element(
                this->data.begin(),
                this->data.end(),
                comparator);

            return maxValue;
        }
    };
}