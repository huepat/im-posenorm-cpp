cmake_minimum_required(VERSION 3.17)

set(PROJECT_NAME im-posenorm)

project(${PROJECT_NAME} LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)

set(REPORT_TIMING true)
set(PARALLELIZE false)
set(TEST_PARALLELIZATION false)

if(REPORT_TIMING)
	add_compile_definitions(
		IMPOSENORM_REPORT_TIMING
	)
endif(REPORT_TIMING)

if(PARALLELIZE)
	add_compile_definitions(
		IMPOSENORM_PARALLELIZE
	)
endif(PARALLELIZE)

if(TEST_PARALLELIZATION)
	add_compile_definitions(
		IMPOSENORM_REPORT_TIMING
		IMPOSENORM_TEST_PARALLELIZATION
	)
endif(TEST_PARALLELIZATION)

add_subdirectory(${CMAKE_SOURCE_DIR}/app)
add_subdirectory(${CMAKE_SOURCE_DIR}/libs/im-posenorm-lib)
add_subdirectory(${CMAKE_SOURCE_DIR}/libs/im-posenorm-eval)
add_subdirectory(${CMAKE_SOURCE_DIR}/libs/im-posenorm-util)