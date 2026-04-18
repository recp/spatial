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
 * Matrix override side-car. Allocated only for nodes with
 * SPATIAL_NODE_HAS_MATRIX set. Used for skew / non-affine transforms.
 */

typedef struct spatial_matrix_override_t {
  mat4 local;
} spatial_matrix_override_t;

#endif /* spatial_node_h */
