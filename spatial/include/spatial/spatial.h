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

#ifndef spatial_h
#define spatial_h

#include "common.h"

/* Dependency: https://github.com/recp/cglm */
#include <cglm/cglm.h>

typedef struct sptl_transform_t {
  struct sptl_transform_t *parent;
  struct sptl_transform_t *chld;
  struct sptl_transform_t *next;
  mat4                     mat;
} sptl_transform_t;

typedef struct sptl_space_t {
  sptl_transform_t roottrans;
  bool             needsupdate;
} sptl_space_t;

SPTL_INLINE
void
sptl_invalidate(sptl_space_t * __restrict space) {
  scape->needsupdate = true;
}

SPTL_EXPORT
void
sptl_update(sptl_space_t * __restrict space);

#endif /* filt_h */
