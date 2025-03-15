#pragma once

template <typename T>
struct WeightedValue {

public:
    double Weight;
    T Value{};

    WeightedValue() :
                Weight(0.0) {
    }

    WeightedValue(
            double weight,
            T value) :
                Weight(weight),
                Value(value) {
    }
};