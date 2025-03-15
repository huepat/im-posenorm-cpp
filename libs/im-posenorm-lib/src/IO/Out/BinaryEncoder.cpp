#include "BinaryEncoder.h"

#include <cstdint>
#include <filesystem>
#include <fstream>

#include "../IOUtil.h"

namespace IMPoseNorm::IO::Out {

	BinaryEncoder::BinaryEncoder(
			bool useLittleEndian,
			const std::filesystem::path& filePath) :
				needsEndiannessSwap(
					NeedsEndiannessSwap(useLittleEndian)),
				file(
					filePath,
					std::ios::app | std::ios::binary) {
	}

	BinaryEncoder::~BinaryEncoder() {

		this->file.close();
	}

	void BinaryEncoder::Encode(
			double value) {

		this->EncodeValue(value);
	}

	void BinaryEncoder::Encode(
			const glm::dvec3& value,
			bool encodeAsFloat) {

		if (encodeAsFloat) {

			this->EncodeValue(static_cast<float>(value.x));
			this->EncodeValue(static_cast<float>(value.y));
			this->EncodeValue(static_cast<float>(value.z));
		}
		else {
			this->EncodeValue(value.x);
			this->EncodeValue(value.y);
			this->EncodeValue(value.z);
		}
	}

	void BinaryEncoder::Encode(
			const glm::u8vec3& color) {

		this->file << static_cast<unsigned char>(color[0]);
		this->file << static_cast<unsigned char>(color[1]);
		this->file << static_cast<unsigned char>(color[2]);
	}

	void BinaryEncoder::Encode(
			const glm::u32vec3& face) {

		this->file << static_cast<unsigned char>(3);

		this->EncodeValue(face[0]);
		this->EncodeValue(face[1]);
		this->EncodeValue(face[2]);
	}

	void BinaryEncoder::EncodePropertySeperator() {

		// nothing to do
	}

	void BinaryEncoder::EncodeLineBreak() {

		// nothing to do
	}

	void BinaryEncoder::Flush() {

		this->file.flush();
	}
}