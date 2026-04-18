/*
 * Copyright (C) 2023 Recep Aslantas
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef spatial_node_h
#define spatial_node_h

#include "common.h"
#include "pose.h"

/*
 * Generational node handle. See spec/handle.md.
 * Slot 0 is reserved as the null sentinel; no live node occupies it.
 */

typedef struct spatial_node_t {
  uint32_t index;
  uint32_t generation;
} spatial_node_t;

#define SPATIAL_NODE_NULL ((spatial_node_t){ .index = 0, .generation = 0 })

SPATIAL_INLINE
bool
spatial_node_eq(spatial_node_t a, spatial_node_t b) {
  return a.index == b.index && a.generation == b.generation;
}

SPATIAL_INLINE
bool
spatial_node_is_null(spatial_node_t n) {
  return n.index == 0 && n.generation == 0;
}

/*
 * Node flags. See spec/authority.md.
 */

enum {
  SPATIAL_NODE_ALIVE        = 1u << 0,
  SPATIAL_NODE_DIRTY_LOCAL  = 1u << 1,
  SPATIAL_NODE_DIRTY_WORLD  = 1u << 2,
  SPATIAL_NODE_HAS_EXPR     = 1u << 3,
  SPATIAL_NODE_HAS_MATRIX   = 1u << 4,
  SPATIAL_NODE_PHYSICS_OWNS = 1u << 5,
  SPATIAL_NODE_KINEMATIC    = 1u << 6,
  SPATIAL_NODE_STATIC       = 1u << 7
};

/*
 * Matrix override side-car. Nodes with SPATIAL_NODE_HAS_MATRIX flag use
 * the local matrix below instead of composing from spatial_pose_t.
 * Used for skew and other non-affine transforms that don't decompose.
 */

typedef struct spatial_matrix_override_t {
  mat4 local;
} spatial_matrix_override_t;

/*
 * Node record. Stored by spatial_space_t as an AoS array.
 * Hierarchy pointers are handles, not pointers, so the storage can grow
 * without invalidating references.
 *
 * IMPORTANT: `parent` MUST be first field. The free-list reuses this slot
 * to thread freed nodes.
 */

typedef struct spatial_node_data_t {
  spatial_node_t parent;
  spatial_node_t first_child;
  spatial_node_t next_sibling;

  spatial_pose_t local;
  spatial_pose_t world;

  mat4        world_matrix;
  spatial_matrix_override_t *matrix_override;  /* null unless HAS_MATRIX */

  uint32_t    flags;
  uint32_t    version;

  void       *user;
} spatial_node_data_t;

/*
 * 2D node record. Parallel to the 3D node.
 * `parent` MUST be first field (same free-list convention).
 */

typedef struct spatial_node_data2_t {
  spatial_node_t  parent;
  spatial_node_t  first_child;
  spatial_node_t  next_sibling;

  spatial_pose2_t local;
  spatial_pose2_t world;

  mat3            world_matrix;

  uint32_t        flags;
  uint32_t        version;

  void           *user;
} spatial_node_data2_t;

#endif /* spatial_node_h */
