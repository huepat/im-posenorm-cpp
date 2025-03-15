#pragma once

#include <cstddef>
#include <filesystem>
#include <fstream>

#include "../IOUtil.h"
#include "IDecoder.h"

namespace IMPoseNorm::IO::In {

	class BinaryDecoder : public IDecoder {

	private:
		bool needsEndiannessSwap;
		std::ifstream file;

	public:
		BinaryDecoder(
			bool useLittleEndian,
			std::size_t readerPosition,
			const std::filesystem::path& filePath);

		~BinaryDecoder();

		virtual void SwitchLine() override;

	protected:
		virtual std::uint8_t ReadUChar() override;
		virtual std::int32_t ReadInt() override;
		virtual std::uint32_t ReadUInt() override;
		virtual std::int64_t ReadLong() override;
		virtual float ReadFloat() override;
		virtual double ReadDouble() override;

	private:

		template <typename T>
		inline T Read() {

			T value;

			this->file.read(
				reinterpret_cast<char*>(&value),
				sizeof(value));

			if (this->needsEndiannessSwap) {

				value = SwapEndianness<T>(value);
			}

			return value;
		}
	};
}