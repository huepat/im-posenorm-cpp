#pragma once

#include <glm/glm.hpp>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace IMPoseNorm::Geometry {

	enum ShapeType {
		POINT_CLOUD,
		MESH
	};

	class AABox;

	class IShape {

	public:
		virtual void UpdateBBox() = 0;

		virtual void SetColors(
			const std::vector<glm::u8vec3>& colors) = 0;

		virtual void SetColors(
			std::vector<glm::u8vec3>&& colors) = 0;

		virtual void SetAdditionalProperties(
			const std::vector<std::string>& additionalPropertyLabels,
			const std::vector<std::vector<float>>& additionalProperties) = 0;

		virtual void SetAdditionalProperties(
			const std::vector<std::string>& additionalPropertyLabels,
			std::vector<std::vector<float>>&& additionalProperties) = 0;

		virtual void Rotate(
			const glm::dvec3& anchor,
			const glm::dmat3& rotation) = 0;

		virtual bool HasNormals() const = 0;
		virtual bool HasPointNormals() const = 0;
		virtual bool HasColor() const = 0;
		virtual bool HasAdditionalProperties() const = 0;
		virtual ShapeType GetType() const = 0;
		virtual std::size_t GetPointCount() const = 0;
		virtual std::size_t GetNormalCount() const = 0;
		virtual std::size_t GetFaceCount() const = 0;
		virtual std::size_t GetAdditionalPropertyCount() const = 0;

		virtual double GetNormalSizeWeight(
			std::size_t normalIndex) const = 0;

		virtual const glm::dvec3& GetPoint(
			std::size_t pointIndex) const = 0;

		virtual glm::dvec3 GetNormal(
			std::size_t normalIndex) const = 0;

		virtual glm::dvec3 GetReferencePointForNormalVector(
			std::size_t normalIndex) const = 0;

		virtual const glm::u8vec3& GetColor(
			std::size_t pointIndex) const = 0;

		virtual float GetAdditionalProperty(
			std::size_t pointIndex,
			std::size_t propertyIndex) const = 0;

		virtual const glm::u32vec3& GetFace(
			std::size_t faceIndex) const = 0;

		virtual glm::dvec3 GetCentroid() const = 0;
		virtual std::shared_ptr<const AABox> GetBBox() const = 0;
		virtual const std::vector<std::string>& GetAdditionalPropertyLabels() const = 0;
		virtual std::unique_ptr<IShape> Clone() const = 0;
	};
}