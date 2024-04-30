#include "texture.h"
#include <framework/image.h>

glm::vec3 acquireTexel(const Image& image, const glm::vec2& texCoord, const Features& features) {
    size_t discreteX = texCoord.x * (image.width    - 1);
    size_t discreteY = texCoord.y * (image.height   - 1);
    size_t memoryLoc = (discreteY * image.width) + discreteX;
    return image.pixels[memoryLoc];
}