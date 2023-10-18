#pragma once
#ifndef _TONE_MAPPING_H_
#define _TONE_MAPPING_H_

#include "common.h"

#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/vec3.hpp>
DISABLE_WARNINGS_POP()

glm::vec3 exposureToneMapping(glm::vec3 color, const Features& features);

#endif
