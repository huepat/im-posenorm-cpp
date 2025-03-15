#include "ASCIIDecoder.h"

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "InUtil.h"

namespace IMPoseNorm::IO::In {

	ASCIIDecoder::ASCIIDecoder(
			const std::filesystem::path& filePath) :
				file(
					filePath,
					std::ios::in) {

		std::string line;

		do {
			std::getline(
				file,
				line);

		} while (line != "end_header");

		this->SwitchLine();
	}

	ASCIIDecoder::~ASCIIDecoder() {

		this->file.close();
	}

	void ASCIIDecoder::SwitchLine() {

		std::string line;

		std::getline(
			this->file,
			line);

		this->values = Split(
			line,
			" ");

		this->propertyIndex = 0;
	}

	std::uint8_t ASCIIDecoder::ReadUChar() {

		return atoi(
			this->values[this->propertyIndex++]
				.c_str());
	}

	std::int32_t ASCIIDecoder::ReadInt() {

		return static_cast<std::int32_t>(
			std::stol(
				this->values[this->propertyIndex++]));
	}

	std::uint32_t ASCIIDecoder::ReadUInt() {

		return static_cast<std::uint32_t>(
			std::stoul(
				this->values[this->propertyIndex++]));
	}

	std::int64_t ASCIIDecoder::ReadLong() {

		return static_cast<std::int64_t>(
			std::stoll(
				this->values[this->propertyIndex++]));
	}

	float ASCIIDecoder::ReadFloat() {

		return std::stof(
			this->values[this->propertyIndex++]);
	}

	double ASCIIDecoder::ReadDouble() {

		return std::stod(
			this->values[this->propertyIndex++]);
	}
}