#pragma once

#include <filesystem>
#include <fstream>

#include "../IOUtil.h"
#include "IEncoder.h"

namespace IMPoseNorm::IO::Out {

	class BinaryEncoder : public IEncoder {
		
	private:
		bool needsEndiannessSwap;
		std::ofstream file;

	public:
		BinaryEncoder(
			bool useLittleEndian,
			const std::filesystem::path& filePath);

		~BinaryEncoder();

		virtual void Encode(
			double value) override;

		virtual void Encode(
			const glm::dvec3& value,
			bool encodeAsFloat) override;

		virtual void Encode(
			const glm::u8vec3& color) override;

		virtual void Encode(
			const glm::u32vec3& face) override;

		virtual void EncodePropertySeperator() override;
		virtual void EncodeLineBreak() override;
		virtual void Flush() override;

	private:
		template <typename T>
		inline void EncodeValue(
				T value) {

			if (this->needsEndiannessSwap) {

				value = SwapEndianness<T>(value);
			}

			this->file.write(
				reinterpret_cast<const char*>(&value),
				sizeof(T));
		}
	};
}