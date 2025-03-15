#if IMPOSENORM_REPORT_TIMING

	#include <im-posenorm-util/Time/TimeReporter.h>

	#include <im-posenorm-util/Time/Timer.h>
	#include <im-posenorm-util/Time/TimeUtil.h>
	
	#include <format>
	#include <iostream>
	#include <stdexcept>
	#include <string>

	namespace IMPoseNorm::Util::Time {

		TimeReporter::TimeReporter() :
					hasBeenUsed(false),
					timer(Timer{}) {

			this->timer.Start();
		}

		void TimeReporter::Report(
				const std::string& message) {

			if (this->hasBeenUsed) {

				throw std::runtime_error(
					"TimeReporter can only be used once.");
			}

			double duration = this->timer.Stop();

			std::cout << std::format(
				"[TIMING REPORT] {}: {}\n",
				message,
				Util::Time::FormatSeconds(duration));

			this->hasBeenUsed = true;
		}
	}
#endif