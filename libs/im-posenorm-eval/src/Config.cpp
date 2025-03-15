#include <im-posenorm-eval/Config.h>

#include <im-posenorm-lib/IMPoseNorm.h>
#include <im-posenorm-lib/IO/PLY.h>

#include <glm/glm.hpp>

#include <array>
#include <cstddef>
#include <filesystem>
#include <format>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "Definitions.h"
#include "Random.h"

namespace IMPoseNorm::Eval {

	const double DEGREE_360 = DegreeToRadian(360.0);

	Config::Config(
			double verticalAngleRange,
			const glm::dvec3& upAxis,
			const glm::dvec3& horizontalAxis,
			const std::filesystem::path& filePath) :
				outputPLY(false),
				horizontallyUnambiguateAlignment(false),
				outputPLYEncoding(IO::PLYEncoding::BINARY_LITTLE_ENDIAN),
				sampleCount(10),
				verticalAngleRange(verticalAngleRange),
				upAxis(upAxis),
				horizontalAxis(horizontalAxis),
				horizontalAxis2(
					glm::cross(
						upAxis,
						horizontalAxis)),
				filePath(filePath),
				outputDirectoryPath(
					std::filesystem::path{ "." }),
				coordinateLabels(
					std::array<std::string, 3>{
						"x", "y", "z"
					}),
				normalVectorLabels(
					std::array<std::string, 3>{
						"nx", "ny", "nz"
					}) {

		std::cout << std::endl;

		std::cout << std::format(
			"UP AXIS: ({}, {}, {})\n",
			this->upAxis.x,
			this->upAxis.y,
			this->upAxis.z);

		std::cout << std::format(
			"HORIZONTAL AXIS: ({}, {}, {})\n",
			this->horizontalAxis.x,
			this->horizontalAxis.y,
			this->horizontalAxis.z);

		std::cout << std::format(
			"HORIZONTAL AXIS 2: ({}, {}, {})\n",
			this->horizontalAxis2.x,
			this->horizontalAxis2.y,
			this->horizontalAxis2.z);

		std::cout << std::endl;
	}

	void Config::SetOutputPLY(
			bool outputPLY) {

		this->outputPLY = outputPLY;
	}

	void Config::SetHorizontallyUnambiguateAlignment(
			bool horizontallyUnambiguateAlignment) {

		this->horizontallyUnambiguateAlignment = horizontallyUnambiguateAlignment;
	}

	void Config::SetOutputPLYEncoding(
			IO::PLYEncoding outputPLYEncoding) {

		this->outputPLYEncoding = outputPLYEncoding;
	}

	void Config::SetSampleCount(
			std::size_t sampleCount) {

		this->sampleCount = sampleCount;
	}

	void Config::SetVerticalAngleRange(
			double verticalAngleRange) {

		this->verticalAngleRange = verticalAngleRange;
	}

	void Config::SetOutputDirectoryPath(
			const std::filesystem::path& outputDirectoryPath) {

		this->outputDirectoryPath = outputDirectoryPath;
	}

	void Config::SetCoordinateLabels(
			const std::array<std::string, 3>& coordinateLabels) {

		this->coordinateLabels = coordinateLabels;
	}

	void Config::SetNormalVectorLabels(
			const std::array<std::string, 3>& normalVectorLabels) {

		this->normalVectorLabels = normalVectorLabels;
	}

	void Config::SetRotations(
			const std::vector<glm::dmat3>& rotations) {

		this->rotations = rotations;
		this->sampleCount = this->rotations.size();
	}

	void Config::SetRotations(
		std::vector<glm::dmat3>&& rotations) {

		this->rotations = std::move(rotations);
		this->sampleCount = this->rotations.size();
	}

	void Config::SetRotationAngles(
			const std::vector<glm::dvec3>& rotationAngles) {

		std::cout << std::endl << "INPUT ANGLES:" << std::endl;

		for (std::size_t i = 0; i < rotationAngles.size(); i++) {

			std::cout << std::format(
				"    {}: ({:.2f}{}, {:.2f}{}, {:.2f}{})\n",
				i,
				RadianToDegree(rotationAngles[i].x),
				SYMBOL_DEGREE,
				RadianToDegree(rotationAngles[i].y),
				SYMBOL_DEGREE,
				RadianToDegree(rotationAngles[i].z),
				SYMBOL_DEGREE);
		}

		std::cout << std::endl;

		this->sampleCount = rotationAngles.size();

		this->rotations = std::vector<glm::dmat3>(this->sampleCount);

		for (std::size_t i = 0; i < this->sampleCount; i++) {

			this->rotations[i] = GetRotationAroundAxis(
					rotationAngles[i].z,
					this->horizontalAxis2)
				* GetRotationAroundAxis(
					rotationAngles[i].y,
					this->horizontalAxis)
				* GetRotationAroundAxis(
					rotationAngles[i].x,
					this->upAxis);
		}
	}

	bool Config::DoOutputPLY() const {

		return this->outputPLY;
	}

	bool Config::DoesHorizontallyUnambiguateAlignment() const {

		return this->horizontallyUnambiguateAlignment;
	}

	IO::PLYEncoding Config::GetOutputPLYEncoding() const {

		return this->outputPLYEncoding;
	}

	std::size_t Config::GetSampleCount() const {

		return this->sampleCount;
	}

	const std::filesystem::path& Config::GetFilePath() const {

		return this->filePath;
	}

	const std::filesystem::path& Config::GetOutputDirectoryPath() const {

		return this->outputDirectoryPath;
	}

	const glm::dvec3& Config::GetUpAxis() const {

		return this->upAxis;
	}

	const glm::dvec3& Config::GetHorizontalAxis() const {

		return this->horizontalAxis;
	}

	const glm::dvec3& Config::GetHorizontalAxis2() const {

		return this->horizontalAxis2;
	}

	const std::array<std::string, 3>& Config::GetCoordinateLabels() const {

		return this->coordinateLabels;
	}

	const std::array<std::string, 3>& Config::GetNormalVectorLabels() const {

		return this->normalVectorLabels;
	}

	const std::vector<glm::dmat3>& Config::GetRotations() {

		if (this->rotations.size() != this->sampleCount) {

			std::vector<glm::dvec3> rotationAngles(
				this->sampleCount);

			for (std::size_t i = 0; i < this->sampleCount; i++) {

				rotationAngles[i] = glm::dvec3 {
					Random::GetDouble(
						0.0,
						DEGREE_360),
					Random::GetDouble(
						-this->verticalAngleRange,
						this->verticalAngleRange),
					Random::GetDouble(
						-this->verticalAngleRange,
						this->verticalAngleRange)
				};
			}

			this->SetRotationAngles(rotationAngles);
		}

		return this->rotations;
	}
}