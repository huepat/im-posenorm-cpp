#include "IEncoder.h"

#include <im-posenorm-lib/IO/PLY.h>

#include <filesystem>
#include <memory>

#include "ASCIIEncoder.h"
#include "BinaryEncoder.h"

namespace IMPoseNorm::IO::Out {

	std::unique_ptr<IEncoder> CreateEncoder(
			PLYEncoding encoding,
			const std::filesystem::path& filePath) {

		switch (encoding) {

		case PLYEncoding::BINARY_LITTLE_ENDIAN:
			return std::make_unique<BinaryEncoder>(
				true,
				filePath);

		case PLYEncoding::BINARY_BIG_ENDIAN:
			return std::make_unique<BinaryEncoder>(
				false,
				filePath);

		case PLYEncoding::ASCII:
			return std::make_unique<ASCIIEncoder>(
				filePath);
		}

		throw std::runtime_error{ "" };
	}
}