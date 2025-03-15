#include "BinaryDecoder.h"

#include <cstddef>
#include <filesystem>
#include <fstream>

#include "../IOUtil.h"

namespace IMPoseNorm::IO::In {

	BinaryDecoder::BinaryDecoder(
			bool useLittleEndian,
			std::size_t readerPosition,
			const std::filesystem::path& filePath) :
				needsEndiannessSwap(
					NeedsEndiannessSwap(useLittleEndian)),
				file(
					filePath,
					std::ios::in | std::ios::binary) {

		file.seekg(readerPosition);
	}

	BinaryDecoder::~BinaryDecoder() {

		this->file.close();
	}

	void BinaryDecoder::SwitchLine() {

		// nothing to do
	}

	std::uint8_t BinaryDecoder::ReadUChar() {

		return this->Read<std::uint8_t>();
	}

	std::int32_t BinaryDecoder::ReadInt() {

		return this->Read<std::int32_t>();
	}

	std::uint32_t BinaryDecoder::ReadUInt() {
		
		return this->Read<std::uint32_t>();
	}

	std::int64_t BinaryDecoder::ReadLong() {

		return this->Read<std::int64_t>();
	}

	float BinaryDecoder::ReadFloat() {

		return this->Read<float>();
	}

	double BinaryDecoder::ReadDouble() {

		return this->Read<double>();
	}
}