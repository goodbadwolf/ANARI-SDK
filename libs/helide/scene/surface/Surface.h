// Copyright 2022 The Khronos Group
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "scene/surface/geometry/Geometry.h"
#include "scene/surface/material/Material.h"

namespace helide {

struct Surface : public Object
{
  Surface(HelideGlobalState *s);
  ~Surface() override;

  void commit() override;

  const Geometry *geometry() const;
  const Material *material() const;

  float4 getSurfaceColor(const Ray &ray) const;
  float getSurfaceOpacity(const Ray &ray) const;

  float adjustedAlpha(float a) const;

  void markCommitted() override;
  bool isValid() const override;

 private:
  helium::IntrusivePtr<Geometry> m_geometry;
  helium::IntrusivePtr<Material> m_material;
};

// Inlined definitions ////////////////////////////////////////////////////////

inline float Surface::adjustedAlpha(float a) const
{
  if (!material())
    return 0.f;

  return adjustOpacityFromMode(
      a, material()->alphaCutoff(), material()->alphaMode());
}

} // namespace helide

HELIDE_ANARI_TYPEFOR_SPECIALIZATION(helide::Surface *, ANARI_SURFACE);
