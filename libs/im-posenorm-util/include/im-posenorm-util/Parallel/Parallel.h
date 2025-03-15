#pragma once

#include <cmath>
#include <cstddef>
#include <functional>
#include <future>
#include <thread>
#include <vector>

namespace IMPoseNorm::Util::Parallel {

    class Parallel {
    private:

        template <typename T>
        using FutureInitializationCallback = std::function<std::future<T>(
            std::size_t partitionStartIndex,
            std::size_t partitionStopIndex)>;

        template <typename T>
        using FutureFinalizationCallback = std::function<void(
            std::future<T>& future)>;

    public:

        template <typename T>
        using ProcessPartitionCallback = std::function<void(
            std::size_t partitionStartIndex,
            std::size_t partitionEndIndex,
            T& partitionData)>;

        static void For(
            std::size_t start,
            std::size_t size,
            std::function<void(std::size_t index)> callback);

        template <typename T>
        static void For(
                std::size_t start,
                std::size_t size,
                std::function<T()> partitionInitializationCallback,
                ProcessPartitionCallback<T> processPartitionCallback,
                std::function<void(T& partitionResult)> finalizationCallback) {

            For<T>(
                start,
                size,
                [
                    &partitionInitializationCallback, 
                    &processPartitionCallback
                ](
                    std::size_t partitionStartIndex,
                    std::size_t partitionStopIndex) {

                        return std::async(
                            [
                                partitionStartIndex,
                                partitionStopIndex,
                                &partitionInitializationCallback,
                                &processPartitionCallback
                            ] {
                                    T partitionData = partitionInitializationCallback();

                                    processPartitionCallback(
                                        partitionStartIndex,
                                        partitionStopIndex,
                                        partitionData);

                                    return partitionData;
                            });
                },
                [&finalizationCallback](std::future<T>& future) {

                    T partitionResult = future.get();

                    finalizationCallback(partitionResult);
                });
        }

    private:

        template <typename T>
        static void For(
                std::size_t start,
                std::size_t size,
                FutureInitializationCallback<T> futureInitializationCallback,
                FutureFinalizationCallback<T> futureFinalizationCallback) {

            std::size_t processorCount = std::thread::hardware_concurrency();

            std::size_t partitionSize = static_cast<std::size_t>(
                std::ceil(
                    static_cast<double>(size) / processorCount));

            std::vector<std::future<T>> futures(processorCount);

            for (size_t i = 0; i < processorCount - 1; i++) {

                futures[i] = futureInitializationCallback(
                    start + i * partitionSize,
                    start + (i + 1) * partitionSize);
            }

            futures[processorCount - 1] = futureInitializationCallback(
                start + (processorCount - 1) * partitionSize,
                start + size);

            for (size_t i = 0; i < processorCount; i++) {

                futureFinalizationCallback(futures[i]);
            }
        }
    };

#if IMPOSENORM_TEST_PARALLELIZATION 
    void TestParallelization(
        std::size_t size);
#endif
}