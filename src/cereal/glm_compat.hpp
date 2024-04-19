#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/glm.hpp>
DISABLE_WARNINGS_POP()

namespace glm {
    // Vectors serialization
    template<class Archive> void serialize(Archive& archive, glm::vec2& v) { archive(v.x, v.y); }
    template<class Archive> void serialize(Archive& archive, glm::vec3& v) { archive(v.x, v.y, v.z); }
    template<class Archive> void serialize(Archive& archive, glm::vec4& v) { archive(v.x, v.y, v.z, v.w); }
    template<class Archive> void serialize(Archive& archive, glm::ivec2& v) { archive(v.x, v.y); }
    template<class Archive> void serialize(Archive& archive, glm::ivec3& v) { archive(v.x, v.y, v.z); }
    template<class Archive> void serialize(Archive& archive, glm::ivec4& v) { archive(v.x, v.y, v.z, v.w); }
    template<class Archive> void serialize(Archive& archive, glm::uvec2& v) { archive(v.x, v.y); }
    template<class Archive> void serialize(Archive& archive, glm::uvec3& v) { archive(v.x, v.y, v.z); }
    template<class Archive> void serialize(Archive& archive, glm::uvec4& v) { archive(v.x, v.y, v.z, v.w); }
    template<class Archive> void serialize(Archive& archive, glm::dvec2& v) { archive(v.x, v.y); }
    template<class Archive> void serialize(Archive& archive, glm::dvec3& v) { archive(v.x, v.y, v.z); }
    template<class Archive> void serialize(Archive& archive, glm::dvec4& v) { archive(v.x, v.y, v.z, v.w); }

    // Matrices serialization
    template<class Archive> void serialize(Archive& archive, glm::mat2& m) { archive(m[0], m[1]); }
    template<class Archive> void serialize(Archive& archive, glm::dmat2& m) { archive(m[0], m[1]); }
    template<class Archive> void serialize(Archive& archive, glm::mat3& m) { archive(m[0], m[1], m[2]); }
    template<class Archive> void serialize(Archive& archive, glm::mat4& m) { archive(m[0], m[1], m[2], m[3]); }
    template<class Archive> void serialize(Archive& archive, glm::dmat4& m) { archive(m[0], m[1], m[2], m[3]); }
}
