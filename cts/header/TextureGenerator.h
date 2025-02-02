// Copyright 2021 The Khronos Group
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "anari/anari_cpp/ext/glm.h"

#include <vector>

namespace cts {

class TextureGenerator
{
 public:
  static std::vector<glm::vec4> generateGreyScale(size_t resolution);
  static std::vector<glm::vec4> generateCheckerBoard(size_t resolution);
  static std::vector<glm::vec4> generateRGBRamp(
      size_t resolution);

  static std::vector<glm::vec4> generateCheckerBoardNormalMap(
      size_t resolution);

  static std::vector<glm::vec4> generateCheckerBoardHDR(
      size_t resolution);

  static glm::vec3 convertColorToNormal(glm::vec3 color);
  static float convertNormalToColor(float input, bool isZ);
  static uint8_t convertShortNormalToColor(int16_t input, bool isZ);
};
} // namespace cts
