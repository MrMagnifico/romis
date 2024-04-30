#include "tone_mapping.h"

DISABLE_WARNINGS_PUSH()
#include <glm/exponential.hpp>
DISABLE_WARNINGS_POP()


glm::vec3 exposureToneMapping(glm::vec3 color, const Features& features) {
    glm::vec3 mapped = glm::vec3(1.0f) - glm::exp(features.exposure * -color);
    return glm::pow(mapped, glm::vec3(1.0f / features.gamma));
}
