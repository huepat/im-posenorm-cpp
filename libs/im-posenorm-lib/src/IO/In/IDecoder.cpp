#include "IDecoder.h"

#include <im-posenorm-lib/IO/PLY.h>

#include <cstddef>
#include <filesystem>
#include <memory>
#include <stdexcept>

#include "ASCIIDecoder.h"
#include "BinaryDecoder.h"

namespace IMPoseNorm::IO::In {

	PropertyValue IDecoder::Read(
			PropertyType propertyType) {

		PropertyValue value;

		switch (propertyType) {

		case PropertyType::UCHAR:
			value = this->ReadUChar();
			break;

		case PropertyType::INT:
			value = this->ReadInt();
			break;

		case PropertyType::UINT:
			value = this->ReadUInt();
			break;

		case PropertyType::LONG:
			value = this->ReadLong();
			break;

		case PropertyType::FLOAT:
			value = this->ReadFloat();
			break;

		case PropertyType::DOUBLE:
			value = this->ReadDouble();
			break;

		default:
			throw std::runtime_error("");
		}

		return value;
	}

	std::unique_ptr<IDecoder> CreateDecoder(
			std::size_t readerPosition,
			PLYEncoding encoding,
			const std::filesystem::path& filePath) {

		switch (encoding) {

		case PLYEncoding::BINARY_LITTLE_ENDIAN:
			return std::make_unique<BinaryDecoder>(
				true,
				readerPosition,
				filePath);

		case PLYEncoding::BINARY_BIG_ENDIAN:
			return std::make_unique<BinaryDecoder>(
				false,
				readerPosition,
				filePath);

		case PLYEncoding::ASCII:
			return std::make_unique<ASCIIDecoder>(
				filePath);
		}

		throw std::runtime_error{ "" };
	}
}