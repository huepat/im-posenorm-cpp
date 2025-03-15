#include <im-posenorm-lib/Geometry/PointCloud.h>

#include <im-posenorm-lib/Geometry/AABox.h>
#if IMPOSENORM_PARALLELIZE
	#include <im-posenorm-util/Parallel/Parallel.h>
#endif
#if IMPOSENORM_REPORT_TIMING
	#include <im-posenorm-util/Time/TimeReporter.h>
#endif

#include <glm/glm.hpp>

#include <cstddef>
#include <format>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "GeometryUtil.h"

namespace IMPoseNorm::Geometry {

	void PointCloud::Check(
			std::size_t pointCloudCount,
			PointCloudCallback pointCloudCallback) {

		PointCloud& referencePointCloud = pointCloudCallback(0);

		const std::vector<std::string>& referencePropertyLabels 
			= referencePointCloud.GetAdditionalPropertyLabels();

		for (std::size_t i = 1; i < pointCloudCount; i++) {

			PointCloud& pointCloud = pointCloudCallback(i);

			if (pointCloud.GetPointCount() != referencePointCloud.GetPointCount()) {

				throw std::runtime_error(
					"All point clouds need to have same size.");
			}

			if (pointCloud.HasNormals() != referencePointCloud.HasNormals()) {

				throw std::runtime_error(
					"Either all point clouds or none should have normals.");
			}

			if (pointCloud.HasColor() != referencePointCloud.HasColor()) {

				throw std::runtime_error(
					"Either all point clouds or none should have colors.");
			}

			const std::vector<std::string>& propertyLabels = pointCloud.GetAdditionalPropertyLabels();

			if (propertyLabels.size() != referencePropertyLabels.size()) {

				throw std::runtime_error(
					"All point clouds need to have same number of additional properties.");
			}

			for (size_t j = 0; j < propertyLabels.size(); j++) {

				if (propertyLabels[j] != referencePropertyLabels[j]) {

					throw std::runtime_error(
						std::format(
							"Additional property #{} is '{}' for point cloud #0 but '{}' for point cloud #{}.",
							j,
							referencePropertyLabels[j],
							propertyLabels[j],
							i));
				}
			}
		}
	}

	std::size_t PointCloud::GetTotalPointCount(
			std::size_t pointCloudCount,
			PointCloudCallback pointCloudCallback) {
		
		Check(
			pointCloudCount,
			pointCloudCallback);

		std::size_t totalSize = 0;

		for (size_t i = 0; i < pointCloudCount; i++) {

			totalSize += pointCloudCallback(i).GetPointCount();
		}

		return totalSize;
	}

	PointCloud::PointCloud(
			const std::vector<glm::dvec3> &points) :
				PointCloud(
					points,
					std::vector<glm::dvec3>(0)) {
	}

	PointCloud::PointCloud(
			std::vector<glm::dvec3>&& points) :
				PointCloud(
					std::move(points),
					std::vector<glm::dvec3>(0)) {
	}

	PointCloud::PointCloud(
			const std::vector<glm::dvec3>& points,
			const std::vector<glm::dvec3>& normals) :
				points(points),
				normals(normals),
				bBox(
					std::shared_ptr<const AABox>{
						nullptr
					}) {

		this->CheckValidity(
			this->normals,
			"normals");
	}

	PointCloud::PointCloud(
			std::vector<glm::dvec3>&& points,
			std::vector<glm::dvec3>&& normals) :
				points(
					std::move(points)),
				normals(
					std::move(normals)),
				bBox(
					std::shared_ptr<const AABox>{
						nullptr
					}) {

		this->CheckValidity(
			this->normals,
			"normals");
	}

	PointCloud::PointCloud(
			std::vector<PointCloud>& pointClouds) :
				PointCloud(
					false,
					pointClouds.size(),
					[pointClouds](std::size_t index) mutable -> PointCloud& {
						return pointClouds[index];
					}) {
	}

	PointCloud::PointCloud(
			std::vector<PointCloud>&& pointClouds) :
				PointCloud(
					true,
					pointClouds.size(),
					[pointClouds](std::size_t index) mutable -> PointCloud& {
						return pointClouds[index];
					}) {
	}

	PointCloud::PointCloud(
			bool useMoveSemantics,
			std::size_t pointCloudCount,
			PointCloudCallback pointCloudCallback) :
				PointCloud(
					useMoveSemantics,
					GetTotalPointCount(
						pointCloudCount,
						pointCloudCallback),
					pointCloudCount,
					pointCloudCallback,
					[pointCloudCallback](std::size_t index) {
						return pointCloudCallback(index).GetPointCount();
					}) {
	}

	PointCloud::PointCloud(
			bool useMoveSemantics,
			std::size_t totalPointCount,
			std::size_t pointCloudCount,
			PointCloudCallback pointCloudCallback,
			SizeCallback sizeCallback) :
				points(
					Merge<glm::dvec3>(
						totalPointCount,
						pointCloudCount,
						sizeCallback,
						[pointCloudCallback](std::size_t index) -> std::vector<glm::dvec3>& {
							return pointCloudCallback(index).points;
						},
						useMoveSemantics ?
							MoveData<glm::dvec3> :
							CopyData<glm::dvec3>)),
				normals(
					Merge<glm::dvec3>(
						totalPointCount,
						pointCloudCount,
						sizeCallback,
						[pointCloudCallback](std::size_t index) -> std::vector<glm::dvec3>& {
							return pointCloudCallback(index).normals;
						},
						useMoveSemantics ?
							MoveData<glm::dvec3> :
							CopyData<glm::dvec3>)),
				colors(
					Merge<glm::u8vec3>(
						totalPointCount,
						pointCloudCount,
						sizeCallback,
						[pointCloudCallback](std::size_t index) -> std::vector<glm::u8vec3>& {
							return pointCloudCallback(index).colors;
						},
						useMoveSemantics ?
							MoveData<glm::u8vec3> :
							CopyData<glm::u8vec3>)),
				additionalProperties(
					Merge<std::vector<float>>(
						totalPointCount,
						pointCloudCount,
						sizeCallback,
						[pointCloudCallback](std::size_t index) -> std::vector<std::vector<float>>& {
							return pointCloudCallback(index).additionalProperties;
						},
						useMoveSemantics ?
							MoveData<std::vector<float>> :
							CopyData<std::vector<float>>)),
				additionalPropertyLabels(
					pointCloudCallback(0).additionalPropertyLabels) {
	}

	void PointCloud::UpdateBBox() {

		this->bBox = std::make_shared<const AABox>(
			IMPoseNorm::Geometry::GetBBox(this->points));
	}

	void PointCloud::SetColors(
			const std::vector<glm::u8vec3>& colors) {

		this->colors = colors;

		this->CheckValidity(
			colors,
			"colors");
	}

	void PointCloud::SetColors(
			std::vector<glm::u8vec3>&& colors) {

		this->colors = std::move(colors);

		this->CheckValidity(
			colors,
			"colors");
	}

	void PointCloud::SetAdditionalProperties(
			const std::vector<std::string>& additionalPropertyLabels,
			const std::vector<std::vector<float>>& additionalProperties) {

		this->additionalPropertyLabels = additionalPropertyLabels;
		this->additionalProperties = additionalProperties;

		this->CheckValidityOfAdditionalProperties();
	}

	void PointCloud::SetAdditionalProperties(
			const std::vector<std::string>& additionalPropertyLabels,
			std::vector<std::vector<float>>&& additionalProperties) {

		this->additionalPropertyLabels = additionalPropertyLabels;
		this->additionalProperties = std::move(additionalProperties);

		this->CheckValidityOfAdditionalProperties();
	}

	void PointCloud::Rotate(
			bool rotateNormals,
			std::size_t pointIndex,
			const glm::dvec3& anchor,
			const glm::dmat3& rotation) {

		this->points[pointIndex] = rotation * (this->points[pointIndex] - anchor) + anchor;

		if (rotateNormals) {

			this->normals[pointIndex] = rotation * this->normals[pointIndex];
		}
	}

#if IMPOSENORM_PARALLELIZE
	void PointCloud::Rotate_(
			bool rotateNormals,
			const glm::dvec3& anchor,
			const glm::dmat3& rotation) {

		Util::Parallel::Parallel::For(
			0,
			this->points.size(),
			[
				this, 
				rotateNormals,
				&anchor,
				&rotation
			](std::size_t i) {

				this->Rotate(
					rotateNormals,
					i,
					anchor,
					rotation);
			});
	}
#else
	void PointCloud::Rotate_(
			bool rotateNormals,
			const glm::dvec3& anchor,
			const glm::dmat3& rotation) {

		for (std::size_t i = 0; i < this->points.size(); i++) {

			this->Rotate(
				rotateNormals,
				i,
				anchor,
				rotation);
		}
	}
#endif

	void PointCloud::Rotate(
			const glm::dvec3& anchor,
			const glm::dmat3& rotation) {

#if IMPOSENORM_REPORT_TIMING
		Util::Time::TimeReporter timeReporter{};
#endif

		bool rotateNormals = this->normals.size() != 0;

		this->Rotate_(
			rotateNormals,
			anchor,
			rotation);

#if IMPOSENORM_REPORT_TIMING
		timeReporter.Report("Rotating Shape");
#endif
	}

	bool PointCloud::HasNormals() const {

		return this->normals.size() != 0;
	}

	bool PointCloud::HasPointNormals() const {

		return this->HasNormals();
	}

	bool PointCloud::HasColor() const {

		return this->colors.size() != 0;
	}

	bool PointCloud::HasAdditionalProperties() const {

		return this->additionalPropertyLabels.size() != 0;
	}

	ShapeType PointCloud::GetType() const {

		return ShapeType::POINT_CLOUD;
	}

	std::size_t PointCloud::GetPointCount() const {

		return this->points.size();
	}

	std::size_t PointCloud::GetNormalCount() const {

		return this->normals.size();
	}

	std::size_t PointCloud::GetFaceCount() const {

		return 0;
	}

	std::size_t PointCloud::GetAdditionalPropertyCount() const {

		return this->additionalPropertyLabels.size();
	}

	double PointCloud::GetNormalSizeWeight(
			std::size_t normalIndex) const {

		return 1.0;
	}

	const glm::dvec3& PointCloud::GetPoint(
			std::size_t pointIndex) const {

		return this->points[pointIndex];
	}

	glm::dvec3 PointCloud::GetNormal(
			std::size_t normalIndex) const {

		return this->normals[normalIndex];
	}

	glm::dvec3 PointCloud::GetReferencePointForNormalVector(
			std::size_t normalIndex) const {

		return this->points[normalIndex];
	}

	const glm::u8vec3& PointCloud::GetColor(
			std::size_t pointIndex) const {

		return this->colors[pointIndex];
	}

	float PointCloud::GetAdditionalProperty(
			std::size_t pointIndex,
			std::size_t propertyIndex) const {

		return this->additionalProperties[propertyIndex][pointIndex];
	}

	const glm::u32vec3& PointCloud::GetFace(
			std::size_t faceIndex) const {

		throw std::runtime_error(
			"PointCloud does not have faces.");
	}

#if IMPOSENORM_PARALLELIZE
	glm::dvec3 PointCloud::GetCentroid_() const {

		glm::dvec3 centroid{
			0.0f
		};

		Util::Parallel::Parallel::For<glm::dvec3>(
			0,
			this->points.size(),
			[]() {
				return glm::dvec3 {
					0.0f
				};
			},
			[this](
				std::size_t partitionStartIndex,
				std::size_t partitionStopIndex,
				glm::dvec3& partitionCentroid){

					for (size_t i = partitionStartIndex; i < partitionStopIndex; i++) {

						partitionCentroid += this->points[i];
					}
			},
			[&centroid](const glm::dvec3& partitionCentroid) {

				centroid += partitionCentroid;
			});

		return centroid / static_cast<double>(this->points.size());
	}
#else
	glm::dvec3 PointCloud::GetCentroid_() const {

		glm::dvec3 centroid{
			0.0f
		};

		for (const glm::dvec3& point : this->points) {

			centroid += point;
		}

		return centroid / static_cast<double>(this->points.size());
	}
#endif

	glm::dvec3 PointCloud::GetCentroid() const {

#if IMPOSENORM_REPORT_TIMING
		Util::Time::TimeReporter timeReporter{};
#endif

		glm::dvec3 centroid = this->GetCentroid_();

#if IMPOSENORM_REPORT_TIMING
		timeReporter.Report("Centroid from Shape");
#endif

		return centroid;
	}

	std::shared_ptr<const AABox> PointCloud::GetBBox() const {

		return this->bBox;
	}

	const std::vector<std::string>& PointCloud::GetAdditionalPropertyLabels() const {

		return this->additionalPropertyLabels;
	}

	std::unique_ptr<IShape> PointCloud::Clone() const {

		return std::make_unique<PointCloud>(*this);
	}

	void PointCloud::CheckValidityOfAdditionalProperties() const {

		std::string label{  
			"each additional property"
		};

		for (const std::vector<float>& additionalProperty : this->additionalProperties) {

			this->CheckValidity(
				additionalProperty,
				label);
		}
	}
}