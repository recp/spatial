/*
 * Copyright (C) 2023 Recep Aslantas
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef spatial_h
#define spatial_h

#include "common.h"
#include "pose.h"
#include "node.h"

/*
 * Runtime context for the spatial system.
 * See DESIGN.md "Evolution of spatial_space_t" and spec/update.md.
 */

typedef struct spatial_space_t {
  spatial_node_data_t *nodes;
  uint32_t         *generations;
  uint32_t          capacity;
  uint32_t          count;
  uint32_t          free_head;

  spatial_node_t       root;

  spatial_node_t      *dirty_roots;
  uint32_t          dirty_count;
  uint32_t          dirty_capacity;

  uint64_t          update_version;
  uint32_t          flags;
} spatial_space_t;

/* Space lifecycle. */

SPATIAL_EXPORT
spatial_space_t *
spatial_space_create(uint32_t initial_capacity);

SPATIAL_EXPORT
void
spatial_space_destroy(spatial_space_t * __restrict space);

/* Node lifecycle. */

SPATIAL_EXPORT
spatial_node_t
spatial_node_create(spatial_space_t      * __restrict space,
                 spatial_node_t                    parent,
                 const spatial_pose_t * __restrict local);

SPATIAL_EXPORT
void
spatial_node_destroy(spatial_space_t * __restrict space, spatial_node_t handle);

SPATIAL_EXPORT
bool
spatial_node_valid(const spatial_space_t * __restrict space, spatial_node_t handle);

/* Access. */

SPATIAL_EXPORT
spatial_node_data_t *
spatial_node_get(spatial_space_t * __restrict space, spatial_node_t handle);

SPATIAL_EXPORT
void
spatial_node_set_local(spatial_space_t      * __restrict space,
                    spatial_node_t                    handle,
                    const spatial_pose_t * __restrict local);

SPATIAL_EXPORT
void
spatial_node_set_world_physics(spatial_space_t      * __restrict space,
                            spatial_node_t                    handle,
                            const spatial_pose_t * __restrict world);

SPATIAL_EXPORT
bool
spatial_node_get_world(const spatial_space_t * __restrict space,
                    spatial_node_t                    handle,
                    spatial_pose_t        * __restrict out);

/* Hierarchy edits. */

SPATIAL_EXPORT
bool
spatial_node_attach(spatial_space_t * __restrict space,
                 spatial_node_t                child,
                 spatial_node_t                new_parent);

/* Matrix override (for skew / non-affine / graphics-only). */

SPATIAL_EXPORT
void
spatial_node_set_matrix(spatial_space_t * __restrict space,
                        spatial_node_t                handle,
                        const mat4                    local_matrix);

SPATIAL_EXPORT
void
spatial_node_clear_matrix(spatial_space_t * __restrict space,
                          spatial_node_t                handle);

/* Update. */

SPATIAL_EXPORT
void
spatial_update(spatial_space_t * __restrict space);

/* ================================================================== */
/* 2D API.                                                            */
/* ================================================================== */

typedef struct spatial_space2_t {
  spatial_node_data2_t *nodes;
  uint32_t             *generations;
  uint32_t              capacity;
  uint32_t              count;
  uint32_t              free_head;

  spatial_node_t        root;

  spatial_node_t       *dirty_roots;
  uint32_t              dirty_count;
  uint32_t              dirty_capacity;

  uint64_t              update_version;
  uint32_t              flags;
} spatial_space2_t;

#define SPATIAL_POSE2_IDENTITY ((spatial_pose2_t){                   \
    .position = {0.0f, 0.0f},                                        \
    .rotation = { .c = 1.0f, .s = 0.0f },                            \
    .scale    = {1.0f, 1.0f}                                         \
})

SPATIAL_EXPORT spatial_space2_t *spatial_space2_create(uint32_t initial_capacity);
SPATIAL_EXPORT void              spatial_space2_destroy(spatial_space2_t *space);

SPATIAL_EXPORT spatial_node_t
spatial_node2_create(spatial_space2_t       * __restrict space,
                     spatial_node_t                       parent,
                     const spatial_pose2_t  * __restrict  local);

SPATIAL_EXPORT void
spatial_node2_destroy(spatial_space2_t *space, spatial_node_t handle);

SPATIAL_EXPORT bool
spatial_node2_valid(const spatial_space2_t *space, spatial_node_t handle);

SPATIAL_EXPORT spatial_node_data2_t *
spatial_node2_get(spatial_space2_t *space, spatial_node_t handle);

SPATIAL_EXPORT void
spatial_node2_set_local(spatial_space2_t       * __restrict space,
                        spatial_node_t                       handle,
                        const spatial_pose2_t  * __restrict  local);

SPATIAL_EXPORT void
spatial_node2_set_world_physics(spatial_space2_t       * __restrict space,
                                spatial_node_t                       handle,
                                const spatial_pose2_t  * __restrict  world);

SPATIAL_EXPORT bool
spatial_node2_get_world(const spatial_space2_t *space,
                        spatial_node_t         handle,
                        spatial_pose2_t       *out);

SPATIAL_EXPORT bool
spatial_node2_attach(spatial_space2_t *space,
                     spatial_node_t    child,
                     spatial_node_t    new_parent);

SPATIAL_EXPORT void
spatial_update2(spatial_space2_t * __restrict space);

#endif /* spatial_h */
