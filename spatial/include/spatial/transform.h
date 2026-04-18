/*
 * Copyright (C) 2023 Recep Aslantas
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef transform_h
#define transform_h

#include "common.h"

/*
 * Transform expression chain (spatial-expr layer).
 *
 * These types are authoring inputs. They evaluate into the canonical
 * spatial_pose_t stored on a node (see pose.h, node.h).
 *
 * Only graphics, editor, and animation systems need expressions.
 * Physics and audio consume the resolved pose directly.
 */

typedef enum spatial_transform_type_t {
  SPATIAL_TRANS_LOOK_AT   = 1,
  SPATIAL_TRANS_MATRIX    = 2,
  SPATIAL_TRANS_ROTATE    = 3,
  SPATIAL_TRANS_SCALE     = 4,
  SPATIAL_TRANS_SKEW      = 5,
  SPATIAL_TRANS_TRANSLATE = 6,
  SPATIAL_TRANS_QUAT      = 7
} spatial_transform_type_t;

typedef struct spatial_transform_item_t {
  struct spatial_transform_item_t *prev;
  struct spatial_transform_item_t *next;
  spatial_transform_type_t         type;
} spatial_transform_item_t;

typedef struct spatial_matrix_t {
  spatial_transform_item_t base;
  mat4                  value;
} spatial_matrix_t;

typedef struct spatial_lookat_t {
  spatial_transform_item_t base;
  vec3                  value[3];
} spatial_lookat_t;

typedef struct spatial_rotate_t {
  spatial_transform_item_t base;
  vec4                  value;
} spatial_rotate_t;

typedef struct spatial_scale_t {
  spatial_transform_item_t base;
  vec3                  value;
} spatial_scale_t;

typedef struct spatial_translate_t {
  spatial_transform_item_t base;
  vec3                  value;
} spatial_translate_t;

typedef struct spatial_quaternion_t {
  spatial_transform_item_t base;
  versor                value;
} spatial_quaternion_t;

typedef struct spatial_skew_t {
  spatial_transform_item_t base;
  float                 angle;
  vec3                  rotateAxis;
  vec3                  aroundAxis;
} spatial_skew_t;

#endif /* transform_h */
