/*
 * Copyright (C) 2023 Recep Aslantas
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef spatial_pose_h
#define spatial_pose_h

#include "common.h"

/*
 * Canonical spatial types. See spec/pose.md.
 *
 * - spatial_transform_t  : position + unit quaternion (physics-compatible)
 * - spatial_pose_t       : position + rotation + scale (graphics / editor)
 * - spatial_transform2_t : 2D variant (Box2D-compatible)
 * - spatial_dtransform_t : large-world variant (optional, SPATIAL_DOUBLE_PRECISION)
 */

typedef struct spatial_transform_t {
  vec3   position;
  versor rotation;
} spatial_transform_t;

typedef struct spatial_pose_t {
  vec3   position;
  versor rotation;
  vec3   scale;
} spatial_pose_t;

typedef struct spatial_rot2_t {
  float c;
  float s;
} spatial_rot2_t;

typedef struct spatial_transform2_t {
  vec2        position;
  spatial_rot2_t rotation;
} spatial_transform2_t;

typedef struct spatial_pose2_t {
  vec2        position;
  spatial_rot2_t rotation;
  vec2        scale;
} spatial_pose2_t;

#ifdef SPATIAL_DOUBLE_PRECISION
typedef struct spatial_dtransform_t {
  double position[3];
  versor rotation;
} spatial_dtransform_t;
#endif

#define SPATIAL_TRANSFORM_IDENTITY ((spatial_transform_t){                 \
    .position = {0.0f, 0.0f, 0.0f},                                  \
    .rotation = {0.0f, 0.0f, 0.0f, 1.0f}                             \
})

#define SPATIAL_POSE_IDENTITY ((spatial_pose_t){                           \
    .position = {0.0f, 0.0f, 0.0f},                                  \
    .rotation = {0.0f, 0.0f, 0.0f, 1.0f},                            \
    .scale    = {1.0f, 1.0f, 1.0f}                                   \
})

#define SPATIAL_ROT2_IDENTITY ((spatial_rot2_t){ .c = 1.0f, .s = 0.0f })

/* Conversions. */

SPATIAL_EXPORT
void
spatial_transform_to_mat4(const spatial_transform_t * __restrict t, mat4 out);

SPATIAL_EXPORT
void
spatial_pose_to_mat4(const spatial_pose_t * __restrict p, mat4 out);

SPATIAL_EXPORT
void
spatial_mat4_to_pose(const mat4 m, spatial_pose_t * __restrict out);

/* Composition: out = a ∘ b  (apply b in a's frame). */

SPATIAL_EXPORT
void
spatial_transform_compose(const spatial_transform_t * __restrict a,
                       const spatial_transform_t * __restrict b,
                       spatial_transform_t       * __restrict out);

SPATIAL_EXPORT
void
spatial_pose_compose(const spatial_pose_t * __restrict a,
                  const spatial_pose_t * __restrict b,
                  spatial_pose_t       * __restrict out);

/* 2D math. */

SPATIAL_EXPORT
void
spatial_rot2_mul(spatial_rot2_t a, spatial_rot2_t b, spatial_rot2_t *out);

SPATIAL_EXPORT
void
spatial_rot2_rotatev(spatial_rot2_t r, const vec2 v, vec2 out);

SPATIAL_EXPORT
void
spatial_pose2_compose(const spatial_pose2_t * __restrict a,
                      const spatial_pose2_t * __restrict b,
                      spatial_pose2_t       * __restrict out);

SPATIAL_EXPORT
void
spatial_pose2_to_mat3(const spatial_pose2_t * __restrict p, mat3 out);

#endif /* spatial_pose_h */
