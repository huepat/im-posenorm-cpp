#include <im-posenorm-lib/Geometry/Mesh.h>

#include <im-posenorm-lib/Geometry/PointCloud.h>

#include <glm/glm.hpp>

#include <cstddef>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "GeometryUtil.h"

namespace IMPoseNorm::Geometry {

	std::vector<std::size_t> GetVertexOffsets(
			const std::vector<Mesh>& meshes) {

		std::size_t counter = 0;
		std::vector<std::size_t> vertexCounts(meshes.size());

		for (size_t i = 0; i < meshes.size(); i++) {

			vertexCounts[i] = counter;
				
			counter += meshes[i].GetPointCount();
		}

		return vertexCounts;
	}

	std::size_t GetTotalFaceCount(
			const std::vector<Mesh>& meshes) {

		std::size_t totalFaceCount = 0;

		for (size_t i = 0; i < meshes.size(); i++) {

			totalFaceCount += meshes[i].GetFaceCount();
		}

		return totalFaceCount;
	}

	Mesh::Mesh(
			const std::vector<glm::dvec3>& vertices,
			const std::vector<glm::u32vec3>& faces) :
				PointCloud(vertices),
				faces(faces) {
	}

	Mesh::Mesh(
			std::vector<glm::dvec3>&& vertices,
			std::vector<glm::u32vec3>&& faces) :
				PointCloud(
					std::move(vertices)),
				faces(
					std::move(faces)) {
	}

	Mesh::Mesh(
			std::vector<Mesh>& meshes) :
				Mesh(
					false,
					meshes.size(),
					GetTotalFaceCount(meshes),
					GetVertexOffsets(meshes),
					[meshes](std::size_t index) mutable -> Mesh& {
						return meshes[index];
					}) {
	}

	Mesh::Mesh(
			std::vector<Mesh>&& meshes) :
				Mesh(
					true,
					meshes.size(),
					GetTotalFaceCount(meshes),
					GetVertexOffsets(meshes),
					[meshes](std::size_t index) mutable -> Mesh& {
						return meshes[index];
					}) {
	}

	Mesh::Mesh(
			bool useMoveSemantics,
			std::size_t meshCount,
			std::size_t totalFaceCount,
			std::vector<std::size_t> vertexOffsets,
			MeshCallback meshCallback) :
				PointCloud(
					useMoveSemantics,
					meshCount,
					[meshCallback](std::size_t index) -> PointCloud& {
						return meshCallback(index);
					}),
				faces(
					Merge<glm::u32vec3>(
						totalFaceCount,
						meshCount,
						[meshCallback](std::size_t index) {
							return meshCallback(index).GetFaceCount();
						},
						[meshCallback](std::size_t index) -> std::vector<glm::u32vec3>& {
							return meshCallback(index).faces;
						},
						useMoveSemantics ?
							MoveData<glm::u32vec3> :
							CopyData<glm::u32vec3>,
						[vertexOffsets](
							std::size_t shapeIndex,
							std::vector<glm::u32vec3>::iterator start,
							std::vector<glm::u32vec3>::iterator end) {

								for (std::vector<glm::u32vec3>::iterator face = start; 
										face < end; 
										face++) {

									*face += static_cast<std::uint32_t>(vertexOffsets[shapeIndex]);
								}
						})) {
	}

	bool Mesh::HasNormals() const {

		return true;
	}

	bool Mesh::HasPointNormals() const {

		return false;
	}

	ShapeType Mesh::GetType() const {

		return ShapeType::MESH;
	}

	std::size_t Mesh::GetNormalCount() const {

		return this->faces.size();
	}

	std::size_t Mesh::GetFaceCount() const {

		return this->faces.size();
	}

	double Mesh::GetNormalSizeWeight(
			std::size_t normalIndex) const {

		return this->GetFaceArea(normalIndex);
	}

	glm::dvec3 Mesh::GetNormal(
			std::size_t normalIndex) const {

		FaceVertices vertices = this->GetFaceVertices(normalIndex);

		return glm::normalize(
			glm::cross(
				std::get<1>(vertices) - std::get<0>(vertices),
				std::get<2>(vertices) - std::get<0>(vertices)));
	}

	glm::dvec3 Mesh::GetReferencePointForNormalVector(
			std::size_t normalIndex) const {

		FaceVertices vertices = this->GetFaceVertices(normalIndex);

		return (std::get<0>(vertices)
			+ std::get<1>(vertices)
			+ std::get<2>(vertices)
		) / 3.0;
	}

	const glm::u32vec3& Mesh::GetFace(
			std::size_t faceIndex) const {

		return this->faces[faceIndex];
	}

	std::unique_ptr<IShape> Mesh::Clone() const {

		return std::make_unique<Mesh>(*this);
	}

	double Mesh::GetFaceArea(
			std::size_t faceIndex) const {

		FaceVertices vertices = this->GetFaceVertices(faceIndex);

		double a = glm::distance(
			std::get<0>(vertices),
			std::get<1>(vertices));

		double b = glm::distance(
			std::get<0>(vertices),
			std::get<2>(vertices));

		double c = glm::distance(
			std::get<1>(vertices),
			std::get<2>(vertices));

		double s = (a + b + c) / 2.0;

		return std::sqrt(
			s * (s - a) * (s - b) * (s - c));
	}

	Mesh::FaceVertices Mesh::GetFaceVertices(
			std::size_t faceIndex) const {

		const glm::u32vec3& face = this->faces[faceIndex];

		return FaceVertices{
			this->PointCloud::GetPoint(
				static_cast<std::size_t>(face[0])),
			this->PointCloud::GetPoint(
				static_cast<std::size_t>(face[1])),
			this->PointCloud::GetPoint(
				static_cast<std::size_t>(face[2]))
		};
	}
}