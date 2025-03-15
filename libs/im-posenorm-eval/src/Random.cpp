#include "Random.h"

#include <random>

namespace IMPoseNorm::Eval {

	std::default_random_engine randomEngine;

	double Random::GetDouble(
			double min,
			double max) {

		std::uniform_real_distribution<double> distribution(
			min,
			max);

		return distribution(randomEngine);
	}
}