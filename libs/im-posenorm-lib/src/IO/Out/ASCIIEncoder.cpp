#include "ASCIIEncoder.h"

#include <filesystem>
#include <format>
#include <fstream>

namespace IMPoseNorm::IO::Out {

	ASCIIEncoder::ASCIIEncoder(
			const std::filesystem::path& filePath) :
				file(
					filePath,
					std::ios::app) {
	}

	ASCIIEncoder::~ASCIIEncoder() {

		this->file.close();
	}

	void ASCIIEncoder::Encode(
			double value) {

		this->file << value;
	}

	void ASCIIEncoder::Encode(
			const glm::dvec3& value,
			bool encodeAsFloat) {

		if (encodeAsFloat) {

			this->file << std::format(
				"{} {} {}",
				static_cast<float>(value.x),
				static_cast<float>(value.y),
				static_cast<float>(value.z));
		}
		else {
			this->file << std::format(
				"{} {} {}",
				value.x,
				value.y,
				value.z);
		}
	}

	void ASCIIEncoder::Encode(
		const glm::u8vec3& color) {

		this->file << std::format(
			"{} {} {}",
			color[0],
			color[1],
			color[2]);
	}

	void ASCIIEncoder::Encode(
			const glm::u32vec3& face) {

		this->file << std::format(
			"3 {} {} {}",
			face[0],
			face[1],
			face[2]);
	}

	void ASCIIEncoder::EncodePropertySeperator() {

		this->file << " ";
	}

	void ASCIIEncoder::EncodeLineBreak() {

		this->file << "\n";
	}

	void ASCIIEncoder::Flush() {

		this->file.flush();
	}
}