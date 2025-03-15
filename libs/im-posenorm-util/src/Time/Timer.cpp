#include <im-posenorm-util/Time/Timer.h>

#include <chrono>
#include <optional>
#include <stdexcept>

namespace IMPoseNorm::Util::Time {

	Timer::Timer() :
				startTimestamp(std::nullopt) {
	}

	void Timer::Start() {

		this->startTimestamp = std::chrono::steady_clock::now();
	}

	double Timer::Stop() {

		if (!this->startTimestamp) {

			throw std::runtime_error("Start() has not been called.");
		}

		std::chrono::duration<double> duration = std::chrono::steady_clock::now()
			- this->startTimestamp.value();

		this->startTimestamp = std::nullopt;

		return duration.count();
	}
}