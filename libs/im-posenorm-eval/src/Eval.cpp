#include <im-posenorm-eval/Eval.h>

#include <im-posenorm-lib/IMPoseNorm.h>
#include <im-posenorm-lib/Geometry/IShape.h>
#include <im-posenorm-lib/IO/In/PLYReader.h>
#include <im-posenorm-lib/IO/Out/PLYWriter.h>
#include <im-posenorm-util/Statistics/Statistics.h>
#include <im-posenorm-util/Time/Timer.h>
#include <im-posenorm-util/Time/TimeUtil.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/vector_angle.hpp>

#include <array>
#include <cstddef>
#include <filesystem>
#include <format>
#include <iostream>
#include <memory>
#include <vector>

#include "Config.h"
#include "Definitions.h"

namespace IMPoseNorm::Eval {

	const double DEGREE_45 = DegreeToRadian(45.0);
	const double DEGREE_90 = DegreeToRadian(90.0);

	std::unique_ptr<Geometry::IShape> Load(
			const Config& config) {

		IO::In::PLYReader reader{};

		reader.SetCoordinatePropertyLabels(
			config.GetCoordinateLabels());

		reader.SetNormalPropertyLabels(
			config.GetNormalVectorLabels());

		return reader.Read(
			config.GetFilePath());
	}

	void Export(
			const std::filesystem::path& path,
			const Config& config,
			const std::unique_ptr<Geometry::IShape>& model) {
		
		IO::Out::PLYWriter writer{
			config.GetOutputPLYEncoding()
		};

		writer.Write(
			path,
			*model);
	}

	void Evaluate(
			Config& config) {

		if (!std::filesystem::exists(
				config.GetOutputDirectoryPath())) {

			std::filesystem::create_directory(
				config.GetOutputDirectoryPath());
		}

		std::cout << std::endl << "EVALUATION" << std::endl;

		Util::Time::Timer timer{};
		Util::Statistics::DoubleStatistics timeStatistics{};

		std::array<Util::Statistics::AngleStatistics, 2> angleDeviationStatistics{
			Util::Statistics::AngleStatistics{},
			Util::Statistics::AngleStatistics{}
		};

		const std::vector<glm::dmat3>& referenceRotations = config.GetRotations();

		std::unique_ptr<Geometry::IShape> model = Load(config);

		glm::dvec3 centroid = model->GetCentroid();

		if (config.DoOutputPLY()) {

			Export(
				std::format(
					"{}/reference.ply",
					config
						.GetOutputDirectoryPath()
						.string()),
				config,
				model);
		}

		for (std::size_t i = 0; i < referenceRotations.size(); ++i) {

			std::unique_ptr<Geometry::IShape> modelCopy = model->Clone();

			modelCopy->Rotate(
				centroid,
				referenceRotations[i]);

			if (config.DoOutputPLY()) {

				Export(
					std::format(
						"{}/in_{}.ply",
						config
							.GetOutputDirectoryPath()
							.string(),
						i),
					config,
					modelCopy);
			}

			timer.Start();

			glm::dmat3 rotation = NormalizePose(
				*modelCopy,
				config.GetUpAxis(),
				config.GetHorizontalAxis(),
				config.DoesHorizontallyUnambiguateAlignment());

			double duration = timer.Stop();

			timeStatistics.Update(duration);

			if (config.DoOutputPLY()) {

				Export(
					std::format(
						"{}/out_{}.ply",
						config
							.GetOutputDirectoryPath()
							.string(),
						i),
					config,
					modelCopy);
			}

			glm::dvec2 referenceAngles{
				std::abs(
					glm::angle(
						config.GetHorizontalAxis(),
						referenceRotations[i] * config.GetHorizontalAxis())),
				std::abs(
					glm::angle(
						config.GetUpAxis(),
						referenceRotations[i] * config.GetUpAxis()))
			};

			glm::dvec2 angleDeviations{
				std::abs(
					glm::angle(
						config.GetHorizontalAxis(),
						rotation * referenceRotations[i] * config.GetHorizontalAxis())),
				std::abs(
					glm::angle(
						config.GetUpAxis(),
						rotation * referenceRotations[i] * config.GetUpAxis()))
			};

			while (angleDeviations[0] >= DEGREE_45) {
				angleDeviations[0] -= DEGREE_90;
			}

			angleDeviations[0] = std::abs(angleDeviations[0]);

			std::get<0>(angleDeviationStatistics).Update(angleDeviations[0]);
			std::get<1>(angleDeviationStatistics).Update(angleDeviations[1]);

			std::cout << std::format(
				"{}: ({:.2f}{}, {:.2f}{}) -> ({:.2f}{}, {:.2f}{}) [{}]\n",
				i,
				RadianToDegree(referenceAngles[0]),
				SYMBOL_DEGREE,
				RadianToDegree(referenceAngles[1]),
				SYMBOL_DEGREE,
				RadianToDegree(angleDeviations[0]),
				SYMBOL_DEGREE,
				RadianToDegree(angleDeviations[1]),
				SYMBOL_DEGREE,
				Util::Time::FormatSeconds(duration));
		}

		std::cout << "___________________________________________________________" << std::endl;

		std::cout << std::format(
			"({:.2f}{} +- {:.2f}{}, {:.2f}{} +-{:.2f}{}) [{} +- {}]\n",
			RadianToDegree(std::get<0>(angleDeviationStatistics).GetMean()),
			SYMBOL_DEGREE,
			RadianToDegree(std::get<0>(angleDeviationStatistics).GetStandardDeviation()),
			SYMBOL_DEGREE,
			RadianToDegree(std::get<1>(angleDeviationStatistics).GetMean()),
			SYMBOL_DEGREE,
			RadianToDegree(std::get<1>(angleDeviationStatistics).GetStandardDeviation()),
			SYMBOL_DEGREE,
			Util::Time::FormatSeconds(timeStatistics.GetMean()),
			Util::Time::FormatSeconds(timeStatistics.GetStandardDeviation()));
	}
}