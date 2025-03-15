#pragma once

#include <glm/glm.hpp>

#include <filesystem>
#include <memory>

namespace IMPoseNorm::IO {
	enum PLYEncoding;
}

namespace IMPoseNorm::IO::Out {

	class IEncoder {

	public:
		virtual void Encode(
			double value) = 0;

		virtual void Encode(
			const glm::dvec3& value,
			bool encodeAsFloat) = 0;

		virtual void Encode(
			const glm::u8vec3& color) = 0;

		virtual void Encode(
			const glm::u32vec3& face) = 0;

		virtual void EncodePropertySeperator() = 0;

		virtual void EncodeLineBreak() = 0;

		virtual void Flush() = 0;
	};

	std::unique_ptr<IEncoder> CreateEncoder(
		PLYEncoding encoding,
		const std::filesystem::path& filePath);
}