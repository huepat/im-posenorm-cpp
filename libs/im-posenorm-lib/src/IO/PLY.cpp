#include <im-posenorm-lib/IO/PLY.h>

#include <stdexcept>
#include <string>

namespace IMPoseNorm::IO {

	std::string ToString(
			PLYEncoding encoding) {

		switch (encoding) {
		case PLYEncoding::BINARY_LITTLE_ENDIAN:
			return "binary_little_endian";
		case PLYEncoding::BINARY_BIG_ENDIAN:
			return "binary_big_endian";
		case PLYEncoding::ASCII:
			return "ascii";
		}

		throw std::runtime_error{ "" };
	}
}