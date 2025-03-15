#pragma once

#include <im-posenorm-lib/IO/PLY.h>

#include <cstddef>
#include <filesystem>
#include <memory>

#include "Property.h"

namespace IMPoseNorm::IO::In {

	class IDecoder {

	public:
		virtual void SwitchLine() = 0;

		PropertyValue Read(
			PropertyType propertyType);

	protected:
		virtual std::uint8_t ReadUChar() = 0;
		virtual std::int32_t ReadInt() = 0;
		virtual std::uint32_t ReadUInt() = 0;
		virtual std::int64_t ReadLong() = 0;
		virtual float ReadFloat() = 0;
		virtual double ReadDouble() = 0;
	};

	std::unique_ptr<IDecoder> CreateDecoder(
		std::size_t readerPosition,
		PLYEncoding encoding,
		const std::filesystem::path& filePath);
}