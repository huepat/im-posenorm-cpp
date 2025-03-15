project(${PROJECT_NAME})

add_library(im-posenorm-eval STATIC
    "include/im-posenorm-eval/Config.h"
    "include/im-posenorm-eval/Eval.h"
    "src/Config.cpp"
    "src/Eval.cpp"
    "src/Definitions.h"
    "src/Random.h"
    "src/Random.cpp"
)

target_link_libraries(im-posenorm-eval PRIVATE
	im-posenorm-lib
    im-posenorm-util
)

target_include_directories(im-posenorm-eval PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/im-posenorm-eval>
    $<INSTALL_INTERFACE:include/im-posenorm-eval>
	${CMAKE_SOURCE_DIR}/dependencies/glm
)

target_include_directories(im-posenorm-eval PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)