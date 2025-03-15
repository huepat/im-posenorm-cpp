#pragma once

#include <im-posenorm-lib/IO/PLY.h>

#include <glm/glm.hpp>

#include <array>
#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace IMPoseNorm::Eval {

	class Config {

	private:
		bool outputPLY;
		bool horizontallyUnambiguateAlignment;
		IO::PLYEncoding outputPLYEncoding;
		std::size_t sampleCount;
		double verticalAngleRange;
		glm::dvec3 upAxis;
		glm::dvec3 horizontalAxis;
		glm::dvec3 horizontalAxis2;
		std::filesystem::path filePath;
		std::filesystem::path outputDirectoryPath;
		std::array<std::string, 3> coordinateLabels;
		std::array<std::string, 3> normalVectorLabels;
		std::vector<glm::dmat3> rotations;
		
	public:

		Config(
			double verticalAngleRange,
			const glm::dvec3& upAxis,
			const glm::dvec3& horizontalAxis,
			const std::filesystem::path& filePath);

		void SetOutputPLY(
			bool outputPLY);

		void SetHorizontallyUnambiguateAlignment(
			bool horizontallyUnambiguateAlignment);

		void SetOutputPLYEncoding(
			IO::PLYEncoding outputPLYEncoding);

		void SetSampleCount(
			std::size_t sampleCount);

		void SetVerticalAngleRange(
			double verticalAngleRange);

		void SetOutputDirectoryPath(
			const std::filesystem::path& outputDirectoryPath);

		void SetCoordinateLabels(
			const std::array<std::string, 3>& coordinateLabels);

		void SetNormalVectorLabels(
			const std::array<std::string, 3>& normalVectorLabels);

		void SetRotations(
			const std::vector<glm::dmat3>& rotations);

		void SetRotations(
			std::vector<glm::dmat3>&& rotations);

		void SetRotationAngles(
			const std::vector<glm::dvec3>& rotationAngles);

		bool DoOutputPLY() const;
		bool DoesHorizontallyUnambiguateAlignment() const;
		IO::PLYEncoding GetOutputPLYEncoding() const;
		std::size_t GetSampleCount() const;
		const std::filesystem::path& GetFilePath() const;
		const std::filesystem::path& GetOutputDirectoryPath() const;
		const glm::dvec3& GetUpAxis() const;
		const glm::dvec3& GetHorizontalAxis() const;
		const glm::dvec3& GetHorizontalAxis2() const;
		const std::array<std::string, 3>& GetCoordinateLabels() const;
		const std::array<std::string, 3>& GetNormalVectorLabels() const;
		const std::vector<glm::dmat3>& GetRotations();
	};
}