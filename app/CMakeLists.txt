include_directories("${CMAKE_SOURCE_DIR}/libs/im-posenorm-lib/include")

add_executable(im-posenorm-demo-app main.cpp)

set(IMPOSENORM_DEMO_APP_LIBS )

list(APPEND IMPOSENORM_DEMO_APP_LIBS
    im-posenorm-lib
	im-posenorm-eval
)

if(TEST_PARALLELIZATION)
	list(APPEND IMPOSENORM_DEMO_APP_LIBS
		im-posenorm-util
    )
endif(TEST_PARALLELIZATION)

target_link_libraries(im-posenorm-demo-app PRIVATE
	${IMPOSENORM_DEMO_APP_LIBS}
)

target_include_directories(im-posenorm-demo-app PRIVATE
	${CMAKE_SOURCE_DIR}/dependencies/glm
)