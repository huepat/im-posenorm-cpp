#pragma once

#include <im-posenorm-lib/Geometry/PointCloud.h>

#include <glm/glm.hpp>

#include <cstddef>
#include <functional>
#include <memory>
#include <tuple>
#include <vector>

namespace IMPoseNorm::Geometry {

	class Mesh : public PointCloud {

	private:

		using FaceVertices = std::tuple<
			const glm::dvec3&,
			const glm::dvec3&,
			const glm::dvec3&>;

		using MeshCallback = std::function<Mesh& (
			std::size_t index)>;

		std::vector<glm::u32vec3> faces;

	public:

		Mesh(
			const std::vector<glm::dvec3>& vertices,
			const std::vector<glm::u32vec3>& faces);

		Mesh(
			std::vector<glm::dvec3>&& vertices,
			std::vector<glm::u32vec3>&& faces);

		Mesh(
			std::vector<Mesh>& meshes);

		Mesh(
			std::vector<Mesh>&& meshes);

	private:

		Mesh(
			bool useMoveSemantics,
			std::size_t meshCount,
			std::size_t totalFaceCount,
			std::vector<std::size_t> vertexCounts,
			MeshCallback meshCallback);

	public:

		virtual bool HasNormals() const override;
		virtual bool HasPointNormals() const override;
		virtual ShapeType GetType() const override;
		virtual std::size_t GetNormalCount() const override;
		virtual std::size_t GetFaceCount() const override;

		virtual double GetNormalSizeWeight(
			std::size_t normalIndex) const override;

		virtual glm::dvec3 GetNormal(
			std::size_t normalIndex) const override;

		virtual glm::dvec3 GetReferencePointForNormalVector(
			std::size_t normalIndex) const override;

		virtual const glm::u32vec3& GetFace(
			std::size_t faceIndex) const override;

		virtual std::unique_ptr<IShape> Clone() const override;

	private:
		double GetFaceArea(
			std::size_t faceIndex) const;

		FaceVertices GetFaceVertices(
			std::size_t faceIndex) const;
	};
}