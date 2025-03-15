project(${PROJECT_NAME})

set(IMPOSENORM_UTIL_FILES )

list(APPEND IMPOSENORM_UTIL_FILES

    # Parallel
    "include/im-posenorm-util/Parallel/Parallel.h"
	"src/Parallel/Parallel.cpp"

    # Statistics
    "include/im-posenorm-util/Statistics/Statistics.h"
	"src/Statistics/Statistics.cpp"

    # Time
    "include/im-posenorm-util/Time/Timer.h"
    "include/im-posenorm-util/Time/TimeUtil.h"
    "src/Time/Timer.cpp"
    "src/Time/TimeUtil.cpp"
)

if(REPORT_TIMING)
	list(APPEND IMPOSENORM_UTIL_FILES

        # Time
        "include/im-posenorm-util/Time/TimeReporter.h"
        "src/Time/TimeReporter.cpp"
    )
endif(REPORT_TIMING)

add_library(im-posenorm-util STATIC
    ${IMPOSENORM_UTIL_FILES}
)

target_include_directories(im-posenorm-util PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/im-posenorm-util>
    $<INSTALL_INTERFACE:include/im-posenorm-util>
	${CMAKE_SOURCE_DIR}/dependencies/glm
)

target_include_directories(im-posenorm-util PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)