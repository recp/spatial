/*
 * Copyright (C) 2023 Recep Aslantas
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef transform_h
#define transform_h

#include "common.h"

/* Dependency: https://github.com/recp/cglm */
#include <cglm/cglm.h>

typedef enum sptl_transform_flags_t {
  SPTL_TRANSF_NONE          =  0,
  SPTL_TRANSF_LOCAL         = (1 << 0),
  SPTL_TRANSF_LOCAL_ISVALID = (1 << 1) | SPTL_TRANSF_LOCAL,
  SPTL_TRANSF_WORLD         = (1 << 2),
  SPTL_TRANSF_WORLD_ISVALID = (1 << 3) | SPTL_TRANSF_WORLD,
  SPTL_TRANSF_FMAT          =  1 << 4,
  SPTL_TRANSF_FMAT_MV       =  1 << 5,
  SPTL_TRANSF_FMAT_MVP      =  1 << 6,
  SPTL_TRANSF_FMAT_NORMAT   =  1 << 7,  /* use normal matrix or not */
  SPTL_TRANSF_CALC_VIEW     =  1 << 8
} sptl_transform_flags_t;

typedef enum sptl_transform_type_t {
  SPTL_TRANS_LOOK_AT   = 1,
  SPTL_TRANS_MATRIX    = 2,
  SPTL_TRANS_ROTATE    = 3,
  SPTL_TRANS_SCALE     = 4,
  SPTL_TRANS_SKEW      = 5,
  SPTL_TRANS_TRANSLATE = 6,
  SPTL_TRANS_QUAT      = 7
} sptl_transform_type_t;

typedef struct sptl_transform_item_t {
  struct sptl_transform_item_t *prev;
  struct sptl_transform_item_t *next;
  sptl_transform_type_t         type;
} sptl_transform_item_t;

/* individual transforms */

typedef struct sptl_matrix_t {
  sptl_transform_item_t base;
  mat4                  value;
} sptl_matrix_t;

typedef struct sptl_lookat_t {
  sptl_transform_item_t base;
  vec3                  value[3];
} sptl_lookat_t;

typedef struct sptl_rotate_t {
  sptl_transform_item_t base;
  vec4                  value;
} sptl_rotate_t;

typedef struct sptl_scale_t {
  sptl_transform_item_t base;
  vec3                  value;
} sptl_scale_t;

typedef struct sptl_translate_t {
  sptl_transform_item_t base;
  vec3                  value;
} sptl_translate_t;

typedef struct sptl_quaternion_t {
  sptl_transform_item_t base;
  versor                value;
} sptl_quaternion_t;

typedef struct sptl_skew_t {
  sptl_transform_item_t base;
  float                 angle;
  vec3                  rotateAxis;
  vec3                  aroundAxis;
} sptl_skew_t;

typedef struct sptl_transform_t {
  struct sptl_transform_t *parent;
  struct sptl_transform_t *chld;
  void                    *obj;
  sptl_transform_item_t   *item;   /* individual transforms             */
  mat4                     local;  /* cached local transform as matrix  */
  mat4                     world;  /* cached world transform as matrix  */
  sptl_transform_flags_t   flags;
} sptl_transform_t;

#endif /* transform_h */
