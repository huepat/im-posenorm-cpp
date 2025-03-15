#include <im-posenorm-util/Parallel/Parallel.h>

#if IMPOSENORM_TEST_PARALLELIZATION 
    #include <im-posenorm-util/Time/TimeReporter.h>
#endif

#include <cmath>
#include <cstddef>
#include <functional>
#include <future>
#include <thread>
#include <vector>

namespace IMPoseNorm::Util::Parallel { 

    void Parallel::For(
            std::size_t start,
            std::size_t size,
            std::function<void(std::size_t index)> callback) {

        For<void>(
            start,
            size,
            [&callback](
                std::size_t partitionStartIndex,
                std::size_t partitionStopIndex) {

                    return std::async(
                        [
                            partitionStartIndex,
                            partitionStopIndex,
                            &callback
                        ]() {

                            for (size_t i = partitionStartIndex; i < partitionStopIndex; i++) {

                                callback(i);
                            }
                        });
            },
            [](std::future<void>& future) {
                future.get();
            });
    }

#if IMPOSENORM_TEST_PARALLELIZATION
    void TestSequential1(
            std::size_t size) {

        Time::TimeReporter timeReporter{};

        for (size_t i = 0; i < size; i++) {

            std::sqrt(static_cast<double>(i));
        }

        timeReporter.Report("Sequential 1");
    }

    void TestParallel1(
            std::size_t size) {

        Time::TimeReporter timeReporter{};

        Parallel::For(
            static_cast<std::size_t>(0),
            size,
            [](std::size_t i) {
                std::sqrt(static_cast<double>(i));
            });

        timeReporter.Report("Parallel 1");
    }

    void TestSequential2(
            std::size_t size) {

        Time::TimeReporter timeReporter{};

        std::size_t sum = 0;

        for (size_t i = 0; i < size; i++) {

            sum += i;
        }

        timeReporter.Report("Sequential 2");
    }

    void TestParallel2(
            std::size_t size) {

        Time::TimeReporter timeReporter{};

        std::size_t sum = 0;

        Parallel::For<std::size_t>(
            static_cast<std::size_t>(0),
            size,
            []() -> std::size_t {
                return 0;
            },
            [](
                std::size_t partitionStartIndex,
                std::size_t partitionStopIndex,
                std::size_t& partitionSum) {

                    for (size_t i = partitionStartIndex; i < partitionStopIndex; i++) {

                        partitionSum += i;
                    }
            },
            [&sum](const std::size_t& partitionSum) {

                sum += partitionSum;
            });

        timeReporter.Report("Parallel 2");
    }
    
    void TestParallelization(
            std::size_t size) {

        TestSequential1(size);
        TestParallel1(size);
        TestSequential2(size);
        TestParallel2(size);
    }
#endif
}