#pragma once

#if IMPOSENORM_REPORT_TIMING

	#include <im-posenorm-util/Time/Timer.h>

	#include <string>

	namespace IMPoseNorm::Util::Time {

		class TimeReporter {

		private:
			bool hasBeenUsed;
			Timer timer;

		public:
			TimeReporter();

			void Report(
				const std::string& message);
		};
	}
#endif