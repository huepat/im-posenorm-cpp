#include <im-posenorm-lib/Geometry/AABox.h>

#include <im-posenorm-lib/Geometry/Mesh.h>

#include <glm/glm.hpp>

#include <utility>
#include <vector>

namespace IMPoseNorm::Geometry {

	AABox::AABox(
		    const glm::dvec3& min,
		    const glm::dvec3& max) :
			    min(min),
			    max(max) {
	}

    AABox::AABox(
            glm::dvec3&& min,
            glm::dvec3&& max) :
                min(std::move(min)),
                max(std::move(max)) {
    }

	glm::dvec3 AABox::GetSize() const {

		return this->max - this->min;
	}

    const glm::dvec3& AABox::GetMin() const {

        return this->min;
    }

    const glm::dvec3& AABox::GetMax() const {

        return this->max;
    }

	Mesh AABox::ConvertToMesh() const {

        return Mesh(
            std::vector<glm::dvec3>{
                glm::dvec3{ this->min },
                glm::dvec3{ this->min.x, this->max.y, this->min.z },
                glm::dvec3{ this->min.x, this->max.y, this->max.z },
                glm::dvec3{ this->min.x, this->min.y, this->max.z },
                glm::dvec3{ this->max.x, this->min.y, this->min.z },
                glm::dvec3{ this->max.x, this->max.y, this->min.z },
                glm::dvec3{ this->max },
                glm::dvec3{ this->max.x, this->min.y, this->max.z }
            },
            std::vector<glm::u32vec3>{
                glm::u32vec3{ 0, 3, 1 },
                glm::u32vec3{ 1, 3, 2 },
                glm::u32vec3{ 5, 2, 6 },
                glm::u32vec3{ 1, 2, 5 },
                glm::u32vec3{ 2, 3, 6 },
                glm::u32vec3{ 3, 7, 6 },
                glm::u32vec3{ 4, 6, 7 },
                glm::u32vec3{ 4, 5, 6 },
                glm::u32vec3{ 0, 7, 3 },
                glm::u32vec3{ 0, 4, 7 },
                glm::u32vec3{ 0, 1, 4 },
                glm::u32vec3{ 1, 5, 4 }
            });
	}
}