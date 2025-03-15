#pragma once

#include <random>

namespace IMPoseNorm::Eval {

	class Random {

	public:
		static double GetDouble(
			double min,
			double max);
	};
}