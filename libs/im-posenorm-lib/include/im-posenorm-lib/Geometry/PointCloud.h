#pragma once

#include <im-posenorm-lib/Geometry/IShape.h>

#include <glm/glm.hpp>

#include <cstddef>
#include <format>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace IMPoseNorm::Geometry {

	class AABox;

	class PointCloud : public IShape {

	private:
		using PointCloudCallback = std::function<PointCloud& (
			std::size_t index)>;

	protected:

		using SizeCallback = std::function<std::size_t (
			std::size_t index)>;

		template <typename T>
		using DataCallback = std::function<std::vector<T>& (
			std::size_t index)>;

		template <typename T>
		using DataTransferCallback = std::function<void (
			typename std::vector<T>::iterator sourceStart,
			typename std::vector<T>::iterator sourceEnd,
			typename std::vector<T>::iterator destinationStart)>;

		template <typename T>
		using DataManipulationCallback = std::function<void (
			std::size_t shapeIndex,
			typename std::vector<T>::iterator start,
			typename std::vector<T>::iterator end)>;

		template <typename T>
		static inline std::vector<T> Merge(
				std::size_t totalPointCount,
				std::size_t shapeCount,
				SizeCallback sizeCallback,
				DataCallback<T> dataCallback,
				DataTransferCallback<T> dataTransferCallback) {

			return Merge(
				totalPointCount,
				shapeCount,
				sizeCallback,
				dataCallback,
				dataTransferCallback,
				[](
					std::size_t shapeIndex, 
					std::vector<T>::iterator start,
					std::vector<T>::iterator end) {});
		}

		template <typename T>
		static inline std::vector<T> Merge(
				std::size_t mergedSize,
				std::size_t shapeCount,
				SizeCallback sizeCallback,
				DataCallback<T> dataCallback,
				DataTransferCallback<T> dataTransferCallback,
				DataManipulationCallback<T> dataManipulationCallback) {

			if (dataCallback(0).size() == 0) {

				return std::vector<T>(0);
			}

			std::size_t position = 0;
			std::vector<T> merged(mergedSize);

			for (size_t i = 0; i < shapeCount; i++) {

				size_t size = sizeCallback(i);

				std::vector<T>& data = dataCallback(i);

				dataTransferCallback(
					data.begin(),
					data.end(),
					merged.begin() + position);

				dataManipulationCallback(
					i,
					merged.begin() + position,
					merged.begin() + position + size);

				position += size;
			}

			return merged;
		}

		template <typename T>
		static inline void MoveData(
				typename std::vector<T>::iterator sourceStart,
				typename std::vector<T>::iterator sourceEnd,
				typename std::vector<T>::iterator destinationStart) {

			std::move(
				sourceStart,
				sourceEnd,
				destinationStart);
		}

		template <typename T>
		static inline void CopyData(
				typename std::vector<T>::iterator sourceStart,
				typename std::vector<T>::iterator sourceEnd,
				typename std::vector<T>::iterator destinationStart) {

			std::copy(
				sourceStart,
				sourceEnd,
				destinationStart);
		}

	private:

		static void Check(
			std::size_t pointCloudCount,
			PointCloudCallback pointCloudCallback);

		static std::size_t GetTotalPointCount(
			std::size_t pointCloudCount,
			PointCloudCallback pointCloudCallback);

		std::shared_ptr<const AABox> bBox;
		std::vector<glm::dvec3> points;
		std::vector<glm::dvec3> normals;
		std::vector<glm::u8vec3> colors;
		std::vector<std::string> additionalPropertyLabels;
		std::vector<std::vector<float>> additionalProperties;

	public:

		PointCloud(
			const std::vector<glm::dvec3>& points);

		PointCloud(
			std::vector<glm::dvec3>&& points);

		PointCloud(
			const std::vector<glm::dvec3>& points,
			const std::vector<glm::dvec3>& normals);

		PointCloud(
			std::vector<glm::dvec3>&& points,
			std::vector<glm::dvec3>&& normals);

		PointCloud(
			std::vector<PointCloud>& pointClouds);

		PointCloud(
			std::vector<PointCloud>&& pointClouds);

	protected:

			PointCloud(
				bool useMoveSemantics,
				std::size_t pointCloudCount,
				PointCloudCallback pointCloudCallback);

	private:

			PointCloud(
				bool useMoveSemantics,
				std::size_t totalPointSize,
				std::size_t pointCloudCount,
				PointCloudCallback pointCloudCallback,
				SizeCallback sizeCallback);

	public:

		virtual void UpdateBBox() override;

		virtual void SetColors(
			const std::vector<glm::u8vec3>& colors) override;

		virtual void SetColors(
			std::vector<glm::u8vec3>&& colors) override;

		virtual void SetAdditionalProperties(
			const std::vector<std::string>& additionalPropertyLabels,
			const std::vector<std::vector<float>>& additionalProperties) override;

		virtual void SetAdditionalProperties(
			const std::vector<std::string>& additionalPropertyLabels,
			std::vector<std::vector<float>>&& additionalProperties) override;

		virtual void Rotate(
			const glm::dvec3& anchor,
			const glm::dmat3& rotation) override;

		virtual bool HasNormals() const override;
		virtual bool HasPointNormals() const override;
		virtual bool HasColor() const override;
		virtual bool HasAdditionalProperties() const override;
		virtual ShapeType GetType() const override;
		virtual std::size_t GetPointCount() const override;
		virtual std::size_t GetNormalCount() const override;
		virtual std::size_t GetFaceCount() const override;
		virtual std::size_t GetAdditionalPropertyCount() const override;

		virtual double GetNormalSizeWeight(
			std::size_t normalIndex) const override;

		virtual const glm::dvec3& GetPoint(
			std::size_t pointIndex) const override;

		virtual glm::dvec3 GetNormal(
			std::size_t normalIndex) const override;

		virtual glm::dvec3 GetReferencePointForNormalVector(
			std::size_t normalIndex) const override;

		virtual const glm::u8vec3& GetColor(
			std::size_t pointIndex) const override;

		virtual float GetAdditionalProperty(
			std::size_t pointIndex,
			std::size_t propertyIndex) const override;

		virtual const glm::u32vec3& GetFace(
			std::size_t faceIndex) const override;

		virtual glm::dvec3 GetCentroid() const override;
		virtual std::shared_ptr<const AABox> GetBBox() const override;
		virtual const std::vector<std::string>& GetAdditionalPropertyLabels() const override;
		virtual std::unique_ptr<IShape> Clone() const override;

	private:

		void Rotate(
			bool rotateNormals,
			std::size_t pointIndex,
			const glm::dvec3& anchor,
			const glm::dmat3& rotation);

		void Rotate_(
			bool rotateNormals,
			const glm::dvec3& anchor,
			const glm::dmat3& rotation);

		glm::dvec3 GetCentroid_() const;

		template <typename T>
		inline void CheckValidity(
				const std::vector<T>& data,
				const std::string& label) const {

			if (data.size() != 0
					&& this->points.size() != data.size()) {

				throw std::runtime_error(
					std::format(
						"Number of {} needs to be either 0 or equal to number of points.",
						label));
			}
		}

		void CheckValidityOfAdditionalProperties() const;
	};
}