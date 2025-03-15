#pragma once

#include <chrono>
#include <optional>

namespace IMPoseNorm::Util::Time {

	class Timer {

	private:
		std::optional<std::chrono::steady_clock::time_point> startTimestamp;

	public:
		Timer();

		void Start();
		double Stop();
	};
}