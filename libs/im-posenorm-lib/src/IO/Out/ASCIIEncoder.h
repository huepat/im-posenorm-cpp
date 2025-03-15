#pragma once

#include <filesystem>
#include <fstream>

#include "IEncoder.h"

namespace IMPoseNorm::IO::Out {

	class ASCIIEncoder : public IEncoder {

	private:
		std::ofstream file;

	public:
		ASCIIEncoder(
			const std::filesystem::path& filePath);

		~ASCIIEncoder();

		virtual void Encode(
			double value) override;

		virtual void Encode(
			const glm::dvec3& value,
			bool encodeAsFloat) override;

		virtual void Encode(
			const glm::u8vec3& value) override;

		virtual void Encode(
			const glm::u32vec3& face) override;

		virtual void EncodePropertySeperator() override;
		virtual void EncodeLineBreak() override;
		virtual void Flush() override;
	};
}