#include <im-posenorm-util/Time/TimeUtil.h>

#include <cmath>
#include <cstddef>
#include <format>
#include <string>

namespace IMPoseNorm::Util::Time {

	std::string FormatSeconds(
			double seconds) {

		double milliseconds = seconds * 1000.0;
		std::int32_t days, hours, minutes, seconds_;

		days = std::floor(static_cast<std::int32_t>(
			milliseconds / (24 * 60 * 60 * 1000)));

		if (days > 0) {

			milliseconds -= days * 24 * 60 * 60 * 1000;
		}

		hours = std::floor(static_cast<std::int32_t>(
			milliseconds / (60 * 60 * 1000)));

		if (hours > 0) {

			milliseconds -= hours * 60 * 60 * 1000;
		}

		minutes = std::floor(static_cast<std::int32_t>(
			milliseconds / (60 * 1000)));

		if (minutes > 0) {

			milliseconds -= minutes * 60 * 1000;
		}

		seconds_ = std::floor(static_cast<std::int32_t>(
			milliseconds / 1000));

		if (seconds_ > 0) {

			milliseconds -= seconds_ * 1000;
		}

		std::string s{ "" };

		if (days > 0) {

			s += std::format(
				"{}d/",
				days);
		}

		if (hours > 0) {

			s += std::format(
				"{}h/",
				hours);
		}

		if (minutes > 0) {

			s += std::format(
				"{}m/",
				minutes);
		}

		if (seconds_ > 0) {

			s += std::format(
				"{}s/",
				seconds_);
		}

		s += std::format(
			"{:.3f}ms",
			milliseconds);

		return s;
	}
}