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
 * Data-oriented spatial runtime.
 *
 * Storage is Struct-of-Arrays: every field lives in its own contiguous
 * array indexed by node slot. Physics and graphics hot loops iterate
 * these arrays directly for maximum cache efficiency.
 *
 * Fields are grouped by access pattern (hot path vs cold path):
 *   hot :  parents, flags, versions, world_positions, world_rotations
 *   warm:  local_*, world_scales, first_children, next_siblings
 *   cold:  world_matrices (derived), users, matrix_overrides
 *
 * All arrays grow together when capacity is exceeded. Handles remain
 * stable; direct array pointers may be invalidated by creates.
 */

typedef struct spatial_space_t {
  uint32_t         capacity;
  uint32_t         count;
  uint32_t         free_head;
  spatial_node_t   root;

  uint32_t        *generations;

  /* hierarchy */
  spatial_node_t  *parents;
  spatial_node_t  *first_children;
  spatial_node_t  *next_siblings;

  /* local pose SoA — vec4 storage for 16-byte alignment + SIMD.
     xyz are the actual value; w = 1.0f (pass-through, unused semantically). */
  vec4            *local_positions;
  versor          *local_rotations;
  vec4            *local_scales;

  /* world pose SoA */
  vec4            *world_positions;
  versor          *world_rotations;
  vec4            *world_scales;

  /* derived */
  mat4            *world_matrices;

  /* meta */
  uint32_t        *flags;
  uint32_t        *versions;
  void           **users;
  spatial_matrix_override_t **matrix_overrides;

  /* dirty tracking */
  spatial_node_t  *dirty_roots;
  uint32_t         dirty_count;
  uint32_t         dirty_capacity;

  /* iterative traversal scratch — main-thread stack (grown lazily) */
  void            *trav_stack;
  uint32_t         trav_capacity;

  /* Thread pool for parallel dirty-root traversal.
     NULL unless spatial_space_enable_parallel() was called. */
  void            *pool;
  uint32_t         parallel_threshold;

  uint64_t         update_version;
  uint32_t         space_flags;
} spatial_space_t;

/* Lifecycle */
SPATIAL_EXPORT spatial_space_t *spatial_space_create(uint32_t initial_capacity);
SPATIAL_EXPORT void             spatial_space_destroy(spatial_space_t *space);

/* Pre-allocate storage up to `capacity`. Guarantees no internal realloc
   (and thus no pointer invalidation) as long as node count stays <=
   capacity. Callers that cache raw SoA pointers across frames should
   reserve upfront to the expected maximum. No-op if capacity is already
   sufficient. */
SPATIAL_EXPORT void spatial_space_reserve(spatial_space_t *space,
                                          uint32_t         capacity);

/* Node lifecycle */
SPATIAL_EXPORT spatial_node_t spatial_node_create(spatial_space_t      *space,
                                                  spatial_node_t        parent,
                                                  const spatial_pose_t *local);
SPATIAL_EXPORT void           spatial_node_destroy(spatial_space_t *space,
                                                   spatial_node_t   handle);
SPATIAL_EXPORT bool           spatial_node_valid(const spatial_space_t *space,
                                                 spatial_node_t         handle);

/* Pose accessors */
SPATIAL_EXPORT void spatial_node_set_local(spatial_space_t      *space,
                                           spatial_node_t        handle,
                                           const spatial_pose_t *local);
SPATIAL_EXPORT bool spatial_node_get_local(const spatial_space_t *space,
                                           spatial_node_t         handle,
                                           spatial_pose_t        *out);
SPATIAL_EXPORT void spatial_node_set_world_physics(spatial_space_t      *space,
                                                   spatial_node_t        handle,
                                                   const spatial_pose_t *world);
SPATIAL_EXPORT bool spatial_node_get_world(const spatial_space_t *space,
                                           spatial_node_t         handle,
                                           spatial_pose_t        *out);
SPATIAL_EXPORT bool spatial_node_get_world_matrix(const spatial_space_t *space,
                                                  spatial_node_t         handle,
                                                  mat4                   out);

/* Meta accessors */
SPATIAL_EXPORT uint32_t       spatial_node_get_flags(const spatial_space_t *space,
                                                     spatial_node_t         handle);
SPATIAL_EXPORT uint32_t       spatial_node_get_version(const spatial_space_t *space,
                                                       spatial_node_t         handle);
SPATIAL_EXPORT spatial_node_t spatial_node_get_parent(const spatial_space_t *space,
                                                      spatial_node_t         handle);
SPATIAL_EXPORT void           spatial_node_set_user(spatial_space_t *space,
                                                    spatial_node_t   handle,
                                                    void            *user);
SPATIAL_EXPORT void          *spatial_node_get_user(const spatial_space_t *space,
                                                    spatial_node_t         handle);

/* Hierarchy */
SPATIAL_EXPORT bool spatial_node_attach(spatial_space_t *space,
                                        spatial_node_t   child,
                                        spatial_node_t   new_parent);

/* Matrix override */
SPATIAL_EXPORT void spatial_node_set_matrix(spatial_space_t *space,
                                            spatial_node_t   handle,
                                            const mat4       local);
SPATIAL_EXPORT void spatial_node_clear_matrix(spatial_space_t *space,
                                              spatial_node_t   handle);

/* Update */
SPATIAL_EXPORT void spatial_update(spatial_space_t *space);

/* Enable parallel spatial_update. Pass thread_count=0 to use hardware
   concurrency. Must be called before the first parallel-capable update.
   Sequential updates still work after enabling (single dirty root, etc.). */
SPATIAL_EXPORT void spatial_space_enable_parallel(spatial_space_t *space,
                                                  uint32_t         thread_count);

/* Set the minimum dirty_count below which spatial_update falls back to
   sequential even when the pool is enabled. Default is tuned for
   shallow hierarchies; physics-heavy scenes with deep subtrees may
   benefit from lower values. Setting 0 disables the threshold (always
   parallel when pool exists and dirty_count >= 2). */
SPATIAL_EXPORT void spatial_space_set_parallel_threshold(spatial_space_t *space,
                                                         uint32_t         min_dirty_roots);

/* ================================================================== */
/* 2D API. Parallel to 3D, same SoA discipline.                       */
/* ================================================================== */

typedef struct spatial_space2_t {
  uint32_t         capacity;
  uint32_t         count;
  uint32_t         free_head;
  spatial_node_t   root;

  uint32_t        *generations;

  spatial_node_t  *parents;
  spatial_node_t  *first_children;
  spatial_node_t  *next_siblings;

  vec2            *local_positions;
  spatial_rot2_t  *local_rotations;
  vec2            *local_scales;

  vec2            *world_positions;
  spatial_rot2_t  *world_rotations;
  vec2            *world_scales;

  mat3            *world_matrices;

  uint32_t        *flags;
  uint32_t        *versions;
  void           **users;

  spatial_node_t  *dirty_roots;
  uint32_t         dirty_count;
  uint32_t         dirty_capacity;

  void            *trav_stack;
  uint32_t         trav_capacity;

  uint64_t         update_version;
  uint32_t         space_flags;
} spatial_space2_t;

#define SPATIAL_POSE2_IDENTITY ((spatial_pose2_t){                   \
    .position = {0.0f, 0.0f},                                        \
    .rotation = { .c = 1.0f, .s = 0.0f },                            \
    .scale    = {1.0f, 1.0f}                                         \
})

SPATIAL_EXPORT spatial_space2_t *spatial_space2_create(uint32_t initial_capacity);
SPATIAL_EXPORT void              spatial_space2_destroy(spatial_space2_t *space);

SPATIAL_EXPORT spatial_node_t spatial_node2_create(spatial_space2_t       *space,
                                                   spatial_node_t          parent,
                                                   const spatial_pose2_t  *local);
SPATIAL_EXPORT void           spatial_node2_destroy(spatial_space2_t *space,
                                                    spatial_node_t    handle);
SPATIAL_EXPORT bool           spatial_node2_valid(const spatial_space2_t *space,
                                                  spatial_node_t          handle);

SPATIAL_EXPORT void spatial_node2_set_local(spatial_space2_t       *space,
                                            spatial_node_t          handle,
                                            const spatial_pose2_t  *local);
SPATIAL_EXPORT bool spatial_node2_get_local(const spatial_space2_t *space,
                                            spatial_node_t          handle,
                                            spatial_pose2_t        *out);
SPATIAL_EXPORT void spatial_node2_set_world_physics(spatial_space2_t       *space,
                                                    spatial_node_t          handle,
                                                    const spatial_pose2_t  *world);
SPATIAL_EXPORT bool spatial_node2_get_world(const spatial_space2_t *space,
                                            spatial_node_t          handle,
                                            spatial_pose2_t        *out);
SPATIAL_EXPORT bool spatial_node2_get_world_matrix(const spatial_space2_t *space,
                                                   spatial_node_t          handle,
                                                   mat3                    out);

SPATIAL_EXPORT uint32_t       spatial_node2_get_flags(const spatial_space2_t *space,
                                                      spatial_node_t          handle);
SPATIAL_EXPORT uint32_t       spatial_node2_get_version(const spatial_space2_t *space,
                                                        spatial_node_t          handle);
SPATIAL_EXPORT spatial_node_t spatial_node2_get_parent(const spatial_space2_t *space,
                                                       spatial_node_t          handle);

SPATIAL_EXPORT bool spatial_node2_attach(spatial_space2_t *space,
                                         spatial_node_t    child,
                                         spatial_node_t    new_parent);

SPATIAL_EXPORT void spatial_update2(spatial_space2_t *space);

#endif /* spatial_h */
