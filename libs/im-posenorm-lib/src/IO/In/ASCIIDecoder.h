#pragma once

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "IDecoder.h"

namespace IMPoseNorm::IO::In {

	class ASCIIDecoder : public IDecoder {
	private:

		std::size_t propertyIndex;
		std::ifstream file;
		std::vector<std::string> values;

	public:
		ASCIIDecoder(
			const std::filesystem::path& filePath);

		~ASCIIDecoder();

		virtual void SwitchLine() override;

	protected:
		virtual std::uint8_t ReadUChar() override;
		virtual std::int32_t ReadInt() override;
		virtual std::uint32_t ReadUInt() override;
		virtual std::int64_t ReadLong() override;
		virtual float ReadFloat() override;
		virtual double ReadDouble() override;
	};
}