#pragma once

#include <array>
#include <string>

namespace IMPoseNorm::IO {

    const std::string DEFAULT_FACE_VERTICES_PROPERTY_LABEL = "vertex_indices";

    const std::array<std::string, 3> DEFAULT_COORDINATE_PROPERTY_LABELS{
        "x",
        "y",
        "z"
    };

    const std::array<std::string, 3> DEFAULT_NORMAL_PROPERTY_LABELS{
        "nx",
        "ny",
        "nz"
    };

    const std::array<std::string, 3> DEFAULT_COLOR_PROPERTY_LABELS{
        "red",
        "green",
        "blue"
    };

    enum PLYEncoding {
        BINARY_LITTLE_ENDIAN,
        BINARY_BIG_ENDIAN,
        ASCII
    };

    std::string ToString(
        PLYEncoding encoding);
}