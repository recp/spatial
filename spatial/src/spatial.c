/*
 * Copyright (C) 2021 Recep Aslantas
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 */

#include "common.h"

#if !defined(_WIN32)
#  include <pthread.h>
#  include <unistd.h>
#  define SPATIAL_HAS_THREADS 1
#else
/* TODO: Windows thread pool via CreateThread / CONDITION_VARIABLE. */
#  define SPATIAL_HAS_THREADS 0
#endif

/* ============================================================== */
/* Static asserts.                                                  */
/* ============================================================== */

_Static_assert(sizeof(spatial_node_t)      == 8,  "node handle must be 8 bytes");
_Static_assert(sizeof(spatial_rot2_t)      == 8,  "rot2 must be 8 bytes");
_Static_assert(sizeof(spatial_transform2_t) == 16, "transform2 must be 16 bytes");

/* ============================================================== */
/* Pose math. SIMD comes from cglm primitives.                      */
/* ============================================================== */

SPATIAL_EXPORT
void
spatial_transform_to_mat4(const spatial_transform_t * __restrict t, mat4 out) {
  glm_quat_mat4((float *)t->rotation, out);
  out[3][0] = t->position[0];
  out[3][1] = t->position[1];
  out[3][2] = t->position[2];
  out[3][3] = 1.0f;
}

SPATIAL_EXPORT
void
spatial_pose_to_mat4(const spatial_pose_t * __restrict p, mat4 out) {
  glm_quat_mat4((float *)p->rotation, out);

  out[0][0] *= p->scale[0]; out[0][1] *= p->scale[0]; out[0][2] *= p->scale[0];
  out[1][0] *= p->scale[1]; out[1][1] *= p->scale[1]; out[1][2] *= p->scale[1];
  out[2][0] *= p->scale[2]; out[2][1] *= p->scale[2]; out[2][2] *= p->scale[2];

  out[3][0] = p->position[0];
  out[3][1] = p->position[1];
  out[3][2] = p->position[2];
  out[3][3] = 1.0f;
}

SPATIAL_EXPORT
void
spatial_mat4_to_pose(const mat4 m, spatial_pose_t * __restrict out) {
  vec3 sx, sy, sz;
  mat4 r;

  glm_vec3_copy((float *)m[0], sx);
  glm_vec3_copy((float *)m[1], sy);
  glm_vec3_copy((float *)m[2], sz);

  out->scale[0] = glm_vec3_norm(sx);
  out->scale[1] = glm_vec3_norm(sy);
  out->scale[2] = glm_vec3_norm(sz);

  glm_mat4_copy((vec4 *)m, r);
  if (out->scale[0] > 0.0f) glm_vec3_scale(r[0], 1.0f / out->scale[0], r[0]);
  if (out->scale[1] > 0.0f) glm_vec3_scale(r[1], 1.0f / out->scale[1], r[1]);
  if (out->scale[2] > 0.0f) glm_vec3_scale(r[2], 1.0f / out->scale[2], r[2]);
  r[3][0] = r[3][1] = r[3][2] = 0.0f;
  r[3][3] = 1.0f;

  glm_mat4_quat(r, out->rotation);

  out->position[0] = m[3][0];
  out->position[1] = m[3][1];
  out->position[2] = m[3][2];
}

SPATIAL_EXPORT
void
spatial_transform_compose(const spatial_transform_t * __restrict a,
                          const spatial_transform_t * __restrict b,
                          spatial_transform_t       * __restrict out) {
  vec3 rotated;
  glm_quat_rotatev((float *)a->rotation, (float *)b->position, rotated);
  glm_vec3_add((float *)a->position, rotated, out->position);
  glm_quat_mul((float *)a->rotation, (float *)b->rotation, out->rotation);
}

SPATIAL_EXPORT
void
spatial_pose_compose(const spatial_pose_t * __restrict a,
                     const spatial_pose_t * __restrict b,
                     spatial_pose_t       * __restrict out) {
  vec3   scaled, rotated, pos, scl;
  versor rot;

  glm_vec3_mul((float *)a->scale, (float *)b->position, scaled);
  glm_quat_rotatev((float *)a->rotation, scaled, rotated);
  glm_vec3_add((float *)a->position, rotated, pos);

  glm_quat_mul((float *)a->rotation, (float *)b->rotation, rot);
  glm_vec3_mul((float *)a->scale, (float *)b->scale, scl);

  glm_vec3_copy(pos, out->position);
  glm_quat_copy(rot, out->rotation);
  glm_vec3_copy(scl, out->scale);
}

/* Inline hot-path SoA compose. Positions / scales are vec4 for SIMD
   alignment; w lane carries through arithmetic (ignored semantically). */
static SPATIAL_INLINE
void
spatial__compose_slots(const vec4 pa, const versor qa, const vec4 sa,
                       const vec4 pb, const versor qb, const vec4 sb,
                       vec4 po,       versor qo,       vec4 so) {
  vec4 scaled;
  vec3 rotated;

  /* scaled = a.scale * b.position — SIMD 4-lane */
  glm_vec4_mul((float *)sa, (float *)pb, scaled);
  /* rotate: glm_quat_rotatev writes first 3 floats */
  glm_quat_rotatev((float *)qa, scaled, rotated);
  /* po = a.position + rotated  (scalar add on 3 lanes; w preserved from pa) */
  po[0] = pa[0] + rotated[0];
  po[1] = pa[1] + rotated[1];
  po[2] = pa[2] + rotated[2];
  po[3] = 0.0f;

  glm_quat_mul((float *)qa, (float *)qb, qo);

  /* so = a.scale * b.scale — SIMD 4-lane */
  glm_vec4_mul((float *)sa, (float *)sb, so);
}

/* Inline hot-path pose_to_mat4. SIMD broadcast-scale on mat4 columns. */
static SPATIAL_INLINE
void
spatial__pose_slots_to_mat4(const vec4 p, const versor q, const vec4 s, mat4 out) {
  glm_quat_mat4((float *)q, out);
  /* Each column is a vec4 → broadcast scalar scale across 4 lanes. */
  glm_vec4_scale(out[0], s[0], out[0]);
  glm_vec4_scale(out[1], s[1], out[1]);
  glm_vec4_scale(out[2], s[2], out[2]);
  out[3][0] = p[0];
  out[3][1] = p[1];
  out[3][2] = p[2];
  out[3][3] = 1.0f;
}

/* SIMD pose diff. 4-lane subs for pos/scale/quat then squared-sum on
   the semantically meaningful lanes. */
static SPATIAL_INLINE
bool
spatial__pose_slots_differ(const vec4 pa, const versor qa, const vec4 sa,
                           const vec4 pb, const versor qb, const vec4 sb) {
  const float eps2 = 1e-12f;
  vec4   dp, ds;
  versor dq;

  glm_vec4_sub((float *)pa, (float *)pb, dp);
  glm_vec4_sub((float *)sa, (float *)sb, ds);
  glm_vec4_sub((float *)qa, (float *)qb, dq);

  /* position / scale: only xyz matter; quat: all 4 lanes */
  return (dp[0] * dp[0] + dp[1] * dp[1] + dp[2] * dp[2]) > eps2
      || (ds[0] * ds[0] + ds[1] * ds[1] + ds[2] * ds[2]) > eps2
      || glm_vec4_norm2(dq) > eps2;
}

/* ============================================================== */
/* 2D math.                                                         */
/* ============================================================== */

SPATIAL_EXPORT
void
spatial_rot2_mul(spatial_rot2_t a, spatial_rot2_t b, spatial_rot2_t *out) {
  out->c = a.c * b.c - a.s * b.s;
  out->s = a.s * b.c + a.c * b.s;
}

SPATIAL_EXPORT
void
spatial_rot2_rotatev(spatial_rot2_t r, const vec2 v, vec2 out) {
  float x = v[0], y = v[1];
  out[0] = r.c * x - r.s * y;
  out[1] = r.s * x + r.c * y;
}

SPATIAL_EXPORT
void
spatial_pose2_compose(const spatial_pose2_t * __restrict a,
                      const spatial_pose2_t * __restrict b,
                      spatial_pose2_t       * __restrict out) {
  vec2           scaled, rotated;
  spatial_rot2_t rot;

  scaled[0] = a->scale[0] * b->position[0];
  scaled[1] = a->scale[1] * b->position[1];
  spatial_rot2_rotatev(a->rotation, scaled, rotated);
  spatial_rot2_mul(a->rotation, b->rotation, &rot);

  out->position[0] = a->position[0] + rotated[0];
  out->position[1] = a->position[1] + rotated[1];
  out->rotation    = rot;
  out->scale[0]    = a->scale[0] * b->scale[0];
  out->scale[1]    = a->scale[1] * b->scale[1];
}

SPATIAL_EXPORT
void
spatial_pose2_to_mat3(const spatial_pose2_t * __restrict p, mat3 out) {
  out[0][0] =  p->rotation.c * p->scale[0];
  out[0][1] =  p->rotation.s * p->scale[0];
  out[0][2] =  0.0f;
  out[1][0] = -p->rotation.s * p->scale[1];
  out[1][1] =  p->rotation.c * p->scale[1];
  out[1][2] =  0.0f;
  out[2][0] =  p->position[0];
  out[2][1] =  p->position[1];
  out[2][2] =  1.0f;
}

/* ============================================================== */
/* Internal helpers: SoA storage management.                        */
/* ============================================================== */

#define SPATIAL_INITIAL_DIRTY_CAPACITY 16
#define SPATIAL_INITIAL_TRAV_CAPACITY  64

/* Iterative traversal stack entry. */
typedef struct spatial__trav_t {
  spatial_node_t node;
  uint32_t       parent_changed;
} spatial__trav_t;

static
void
spatial__grow_arrays(spatial_space_t *space, uint32_t new_cap) {
  uint32_t old_cap = space->capacity;

  space->generations     = realloc(space->generations,     sizeof(uint32_t)       * new_cap);
  space->parents         = realloc(space->parents,         sizeof(spatial_node_t) * new_cap);
  space->first_children  = realloc(space->first_children,  sizeof(spatial_node_t) * new_cap);
  space->next_siblings   = realloc(space->next_siblings,   sizeof(spatial_node_t) * new_cap);
  space->local_positions = realloc(space->local_positions, sizeof(vec4)           * new_cap);
  space->local_rotations = realloc(space->local_rotations, sizeof(versor)         * new_cap);
  space->local_scales    = realloc(space->local_scales,    sizeof(vec4)           * new_cap);
  space->world_positions = realloc(space->world_positions, sizeof(vec4)           * new_cap);
  space->world_rotations = realloc(space->world_rotations, sizeof(versor)         * new_cap);
  space->world_scales    = realloc(space->world_scales,    sizeof(vec4)           * new_cap);
  space->world_matrices  = realloc(space->world_matrices,  sizeof(mat4)           * new_cap);
  space->flags           = realloc(space->flags,           sizeof(uint32_t)       * new_cap);
  space->versions        = realloc(space->versions,        sizeof(uint32_t)       * new_cap);
  space->users           = realloc(space->users,           sizeof(void *)         * new_cap);
  space->matrix_overrides= realloc(space->matrix_overrides,
                                   sizeof(spatial_matrix_override_t *) * new_cap);

  if (new_cap > old_cap) {
    uint32_t delta = new_cap - old_cap;
    memset(space->generations      + old_cap, 0, sizeof(uint32_t)       * delta);
    memset(space->parents          + old_cap, 0, sizeof(spatial_node_t) * delta);
    memset(space->first_children   + old_cap, 0, sizeof(spatial_node_t) * delta);
    memset(space->next_siblings    + old_cap, 0, sizeof(spatial_node_t) * delta);
    memset(space->flags            + old_cap, 0, sizeof(uint32_t)       * delta);
    memset(space->versions         + old_cap, 0, sizeof(uint32_t)       * delta);
    memset(space->users            + old_cap, 0, sizeof(void *)         * delta);
    memset(space->matrix_overrides + old_cap, 0,
           sizeof(spatial_matrix_override_t *) * delta);
  }

  space->capacity = new_cap;
}

static
uint32_t
spatial__alloc_slot(spatial_space_t * __restrict space) {
  uint32_t slot;

  if (space->free_head != 0) {
    slot = space->free_head;
    space->free_head = space->parents[slot].index;
  } else {
    if (SPATIAL_UNLIKELY(space->count >= space->capacity)) {
      spatial__grow_arrays(space, space->capacity * 2);
    }
    slot = space->count++;
  }
  return slot;
}

/* Dirty push with ancestor dedupe: skip if any ancestor is already dirty. */
static
void
spatial__push_dirty(spatial_space_t * __restrict space, spatial_node_t handle) {
  spatial_node_t a;
  const uint32_t dirty_mask = SPATIAL_NODE_DIRTY_LOCAL | SPATIAL_NODE_DIRTY_WORLD;

  a = space->parents[handle.index];
  while (!spatial_node_is_null(a)) {
    if (SPATIAL_UNLIKELY(a.index >= space->capacity)) break;
    if (space->flags[a.index] & dirty_mask) {
      return;  /* ancestor will cover us */
    }
    a = space->parents[a.index];
  }

  if (SPATIAL_UNLIKELY(space->dirty_count >= space->dirty_capacity)) {
    uint32_t nc = space->dirty_capacity * 2;
    space->dirty_roots    = realloc(space->dirty_roots, sizeof(spatial_node_t) * nc);
    space->dirty_capacity = nc;
  }
  space->dirty_roots[space->dirty_count++] = handle;
}

/* Per-thread traversal context. Main thread uses space->trav_stack;
   worker threads each own their own ctx. */
typedef struct spatial__trav_ctx_t {
  spatial__trav_t *stack;
  uint32_t         capacity;
} spatial__trav_ctx_t;

static
void
spatial__ctx_grow(spatial__trav_ctx_t *ctx, uint32_t want) {
  if (ctx->capacity < want) {
    uint32_t nc = ctx->capacity ? ctx->capacity : SPATIAL_INITIAL_TRAV_CAPACITY;
    while (nc < want) nc *= 2;
    ctx->stack    = realloc(ctx->stack, sizeof(spatial__trav_t) * nc);
    ctx->capacity = nc;
  }
}

/* Forward decls for thread pool (defined below spatial_update). */
#if SPATIAL_HAS_THREADS
struct spatial__pool_t;
static void spatial__pool_destroy(struct spatial__pool_t *p);
#endif

/* ============================================================== */
/* Space + node lifecycle.                                          */
/* ============================================================== */

SPATIAL_EXPORT
spatial_space_t *
spatial_space_create(uint32_t initial_capacity) {
  spatial_space_t *space;
  uint32_t         slot;

  if (initial_capacity < 2) initial_capacity = 16;

  space = calloc(1, sizeof(spatial_space_t));
  space->count     = 1;  /* reserve slot 0 for null */
  space->free_head = 0;
  spatial__grow_arrays(space, initial_capacity);

  space->dirty_roots    = calloc(SPATIAL_INITIAL_DIRTY_CAPACITY, sizeof(spatial_node_t));
  space->dirty_capacity = SPATIAL_INITIAL_DIRTY_CAPACITY;

  /* root at slot 1 */
  slot = spatial__alloc_slot(space);
  space->parents[slot]         = SPATIAL_NODE_NULL;
  space->first_children[slot]  = SPATIAL_NODE_NULL;
  space->next_siblings[slot]   = SPATIAL_NODE_NULL;
  glm_vec4_zero(space->local_positions[slot]);
  glm_quat_identity(space->local_rotations[slot]);
  glm_vec4_one(space->local_scales[slot]);
  glm_vec4_zero(space->world_positions[slot]);
  glm_quat_identity(space->world_rotations[slot]);
  glm_vec4_one(space->world_scales[slot]);
  glm_mat4_identity(space->world_matrices[slot]);
  space->flags[slot]    = SPATIAL_NODE_ALIVE;
  space->versions[slot] = 1;
  space->users[slot]    = NULL;
  space->matrix_overrides[slot] = NULL;
  space->generations[slot]      = 1;
  space->root = (spatial_node_t){ .index = slot, .generation = 1 };

  return space;
}

SPATIAL_EXPORT
void
spatial_space_reserve(spatial_space_t *space, uint32_t capacity) {
  if (!space) return;
  if (capacity > space->capacity) {
    spatial__grow_arrays(space, capacity);
  }
}

SPATIAL_EXPORT
void
spatial_space_destroy(spatial_space_t *space) {
  uint32_t i;

  if (!space) return;

#if SPATIAL_HAS_THREADS
  if (space->pool) {
    spatial__pool_destroy((struct spatial__pool_t *)space->pool);
    space->pool = NULL;
  }
#endif

  for (i = 1; i < space->count; i++) {
    if (space->matrix_overrides[i]) free(space->matrix_overrides[i]);
  }

  free(space->generations);
  free(space->parents);
  free(space->first_children);
  free(space->next_siblings);
  free(space->local_positions);
  free(space->local_rotations);
  free(space->local_scales);
  free(space->world_positions);
  free(space->world_rotations);
  free(space->world_scales);
  free(space->world_matrices);
  free(space->flags);
  free(space->versions);
  free(space->users);
  free(space->matrix_overrides);
  free(space->dirty_roots);
  free(space->trav_stack);
  free(space);
}

SPATIAL_EXPORT
bool
spatial_node_valid(const spatial_space_t * __restrict space, spatial_node_t handle) {
  if (SPATIAL_UNLIKELY(handle.index == 0 || handle.index >= space->capacity)) return false;
  if (SPATIAL_UNLIKELY(space->generations[handle.index] != handle.generation)) return false;
  return (space->flags[handle.index] & SPATIAL_NODE_ALIVE) != 0;
}

SPATIAL_EXPORT
spatial_node_t
spatial_node_create(spatial_space_t      * __restrict space,
                    spatial_node_t                    parent,
                    const spatial_pose_t * __restrict local) {
  uint32_t       slot;
  spatial_node_t handle;

  if (spatial_node_is_null(parent)) parent = space->root;
  if (SPATIAL_UNLIKELY(!spatial_node_valid(space, parent))) return SPATIAL_NODE_NULL;

  slot = spatial__alloc_slot(space);

  space->parents[slot]        = parent;
  space->first_children[slot] = SPATIAL_NODE_NULL;
  /* link under parent */
  space->next_siblings[slot]       = space->first_children[parent.index];
  space->first_children[parent.index] = (spatial_node_t){ .index = slot, .generation = 0 }; /* fix gen below */

  if (local) {
    glm_vec3_copy((float *)local->position, space->local_positions[slot]);
    space->local_positions[slot][3] = 0.0f;
    glm_quat_copy((float *)local->rotation, space->local_rotations[slot]);
    glm_vec3_copy((float *)local->scale,    space->local_scales[slot]);
    space->local_scales[slot][3] = 0.0f;
  } else {
    glm_vec4_zero(space->local_positions[slot]);
    glm_quat_identity(space->local_rotations[slot]);
    glm_vec4_one(space->local_scales[slot]);
  }
  glm_vec4_zero(space->world_positions[slot]);
  glm_quat_identity(space->world_rotations[slot]);
  glm_vec4_one(space->world_scales[slot]);
  glm_mat4_identity(space->world_matrices[slot]);
  space->flags[slot]            = SPATIAL_NODE_ALIVE | SPATIAL_NODE_DIRTY_LOCAL;
  space->versions[slot]         = 1;
  space->users[slot]            = NULL;
  space->matrix_overrides[slot] = NULL;

  space->generations[slot]++;
  if (SPATIAL_UNLIKELY(space->generations[slot] == 0)) space->generations[slot] = 1;
  handle = (spatial_node_t){ .index = slot, .generation = space->generations[slot] };

  /* patch sibling link we stored above with correct generation */
  space->first_children[parent.index] = handle;

  spatial__push_dirty(space, handle);
  return handle;
}

static
void
spatial__unlink_from_parent(spatial_space_t *space, spatial_node_t handle) {
  spatial_node_t parent, cur;

  parent = space->parents[handle.index];
  if (spatial_node_is_null(parent)) return;

  if (spatial_node_eq(space->first_children[parent.index], handle)) {
    space->first_children[parent.index] = space->next_siblings[handle.index];
  } else {
    cur = space->first_children[parent.index];
    while (!spatial_node_is_null(cur)) {
      if (spatial_node_eq(space->next_siblings[cur.index], handle)) {
        space->next_siblings[cur.index] = space->next_siblings[handle.index];
        break;
      }
      cur = space->next_siblings[cur.index];
    }
  }
  space->next_siblings[handle.index] = SPATIAL_NODE_NULL;
}

SPATIAL_EXPORT
void
spatial_node_destroy(spatial_space_t * __restrict space, spatial_node_t handle) {
  spatial_node_t child;

  if (!spatial_node_valid(space, handle)) return;
  if (spatial_node_eq(handle, space->root)) return;

  /* destroy children first (depth-first, iterative not strictly needed here) */
  child = space->first_children[handle.index];
  while (!spatial_node_is_null(child)) {
    spatial_node_t next = space->next_siblings[child.index];
    spatial_node_destroy(space, child);
    child = next;
  }

  spatial__unlink_from_parent(space, handle);

  if (space->matrix_overrides[handle.index]) {
    free(space->matrix_overrides[handle.index]);
    space->matrix_overrides[handle.index] = NULL;
  }

  space->flags[handle.index] = 0;
  /* thread free-list through parent.index */
  space->parents[handle.index] = (spatial_node_t){ .index = space->free_head, .generation = 0 };
  space->free_head = handle.index;
  space->generations[handle.index]++;
  if (SPATIAL_UNLIKELY(space->generations[handle.index] == 0)) space->generations[handle.index] = 1;
}

static
bool
spatial__is_descendant(const spatial_space_t *space,
                       spatial_node_t         root,
                       spatial_node_t         candidate) {
  spatial_node_t cur = candidate;
  while (!spatial_node_is_null(cur)) {
    if (spatial_node_eq(cur, root)) return true;
    if (cur.index >= space->capacity) return false;
    cur = space->parents[cur.index];
  }
  return false;
}

SPATIAL_EXPORT
bool
spatial_node_attach(spatial_space_t * __restrict space,
                    spatial_node_t                child,
                    spatial_node_t                new_parent) {
  if (!spatial_node_valid(space, child))      return false;
  if (spatial_node_is_null(new_parent))       new_parent = space->root;
  if (!spatial_node_valid(space, new_parent)) return false;
  if (spatial_node_eq(child, new_parent))     return false;
  if (spatial__is_descendant(space, child, new_parent)) return false;

  spatial__unlink_from_parent(space, child);

  space->parents[child.index]              = new_parent;
  space->next_siblings[child.index]        = space->first_children[new_parent.index];
  space->first_children[new_parent.index]  = child;

  space->flags[child.index] |= SPATIAL_NODE_DIRTY_LOCAL;
  spatial__push_dirty(space, child);
  return true;
}

/* ============================================================== */
/* Accessors.                                                       */
/* ============================================================== */

SPATIAL_EXPORT
void
spatial_node_set_local(spatial_space_t      * __restrict space,
                       spatial_node_t                    handle,
                       const spatial_pose_t * __restrict local) {
  if (!spatial_node_valid(space, handle) || !local) return;
  glm_vec3_copy((float *)local->position, space->local_positions[handle.index]);
  space->local_positions[handle.index][3] = 0.0f;
  glm_quat_copy((float *)local->rotation, space->local_rotations[handle.index]);
  glm_vec3_copy((float *)local->scale,    space->local_scales[handle.index]);
  space->local_scales[handle.index][3]    = 0.0f;
  space->flags[handle.index] |= SPATIAL_NODE_DIRTY_LOCAL;
  spatial__push_dirty(space, handle);
}

SPATIAL_EXPORT
bool
spatial_node_get_local(const spatial_space_t * __restrict space,
                       spatial_node_t                    handle,
                       spatial_pose_t       * __restrict out) {
  if (!spatial_node_valid(space, handle) || !out) return false;
  glm_vec3_copy(space->local_positions[handle.index], out->position);
  glm_quat_copy(space->local_rotations[handle.index], out->rotation);
  glm_vec3_copy(space->local_scales[handle.index],    out->scale);
  return true;
}

SPATIAL_EXPORT
void
spatial_node_set_world_physics(spatial_space_t      * __restrict space,
                               spatial_node_t                    handle,
                               const spatial_pose_t * __restrict world) {
  if (!spatial_node_valid(space, handle) || !world) return;
  glm_vec3_copy((float *)world->position, space->world_positions[handle.index]);
  space->world_positions[handle.index][3] = 0.0f;
  glm_quat_copy((float *)world->rotation, space->world_rotations[handle.index]);
  glm_vec3_copy((float *)world->scale,    space->world_scales[handle.index]);
  space->world_scales[handle.index][3]    = 0.0f;
  space->flags[handle.index] |= SPATIAL_NODE_DIRTY_WORLD | SPATIAL_NODE_PHYSICS_OWNS;
  spatial__push_dirty(space, handle);
}

SPATIAL_EXPORT
bool
spatial_node_get_world(const spatial_space_t * __restrict space,
                       spatial_node_t                    handle,
                       spatial_pose_t       * __restrict out) {
  if (!spatial_node_valid(space, handle) || !out) return false;
  glm_vec3_copy(space->world_positions[handle.index], out->position);
  glm_quat_copy(space->world_rotations[handle.index], out->rotation);
  glm_vec3_copy(space->world_scales[handle.index],    out->scale);
  return true;
}

SPATIAL_EXPORT
bool
spatial_node_get_world_matrix(const spatial_space_t * __restrict space,
                              spatial_node_t                    handle,
                              mat4                              out) {
  if (!spatial_node_valid(space, handle)) return false;
  glm_mat4_copy((vec4 *)space->world_matrices[handle.index], out);
  return true;
}

SPATIAL_EXPORT
uint32_t
spatial_node_get_flags(const spatial_space_t *space, spatial_node_t handle) {
  if (!spatial_node_valid(space, handle)) return 0;
  return space->flags[handle.index];
}

SPATIAL_EXPORT
uint32_t
spatial_node_get_version(const spatial_space_t *space, spatial_node_t handle) {
  if (!spatial_node_valid(space, handle)) return 0;
  return space->versions[handle.index];
}

SPATIAL_EXPORT
spatial_node_t
spatial_node_get_parent(const spatial_space_t *space, spatial_node_t handle) {
  if (!spatial_node_valid(space, handle)) return SPATIAL_NODE_NULL;
  return space->parents[handle.index];
}

SPATIAL_EXPORT
void
spatial_node_set_user(spatial_space_t *space, spatial_node_t handle, void *user) {
  if (!spatial_node_valid(space, handle)) return;
  space->users[handle.index] = user;
}

SPATIAL_EXPORT
void *
spatial_node_get_user(const spatial_space_t *space, spatial_node_t handle) {
  if (!spatial_node_valid(space, handle)) return NULL;
  return space->users[handle.index];
}

SPATIAL_EXPORT
void
spatial_node_set_matrix(spatial_space_t * __restrict space,
                        spatial_node_t                handle,
                        const mat4                    local) {
  spatial_pose_t pose;
  if (!spatial_node_valid(space, handle)) return;

  if (!space->matrix_overrides[handle.index]) {
    space->matrix_overrides[handle.index] = calloc(1, sizeof(spatial_matrix_override_t));
  }
  glm_mat4_copy((vec4 *)local, space->matrix_overrides[handle.index]->local);

  /* best-effort pose decomposition for readers that ignore matrix path */
  spatial_mat4_to_pose(local, &pose);
  glm_vec3_copy(pose.position, space->local_positions[handle.index]);
  space->local_positions[handle.index][3] = 0.0f;
  glm_quat_copy(pose.rotation, space->local_rotations[handle.index]);
  glm_vec3_copy(pose.scale,    space->local_scales[handle.index]);
  space->local_scales[handle.index][3]    = 0.0f;

  space->flags[handle.index] |= SPATIAL_NODE_HAS_MATRIX | SPATIAL_NODE_DIRTY_LOCAL;
  spatial__push_dirty(space, handle);
}

SPATIAL_EXPORT
void
spatial_node_clear_matrix(spatial_space_t *space, spatial_node_t handle) {
  if (!spatial_node_valid(space, handle)) return;
  if (space->matrix_overrides[handle.index]) {
    free(space->matrix_overrides[handle.index]);
    space->matrix_overrides[handle.index] = NULL;
  }
  space->flags[handle.index] &= ~SPATIAL_NODE_HAS_MATRIX;
  space->flags[handle.index] |= SPATIAL_NODE_DIRTY_LOCAL;
  spatial__push_dirty(space, handle);
}

/* ============================================================== */
/* Update traversal (iterative, explicit stack).                    */
/* ============================================================== */

static
void
spatial__traverse_iter(spatial_space_t      *space,
                       spatial_node_t        start,
                       spatial__trav_ctx_t  *ctx) {
  spatial__trav_t *stack;
  uint32_t         sp = 0;
  const uint32_t   dirty_mask = SPATIAL_NODE_DIRTY_LOCAL | SPATIAL_NODE_DIRTY_WORLD;

  if (!spatial_node_valid(space, start)) return;

  /* initial stack size: depth bounded by node count */
  spatial__ctx_grow(ctx, space->capacity);
  stack = ctx->stack;
  stack[sp++] = (spatial__trav_t){ start, 0u };

  while (sp > 0) {
    spatial__trav_t e     = stack[--sp];
    uint32_t        idx   = e.node.index;
    uint32_t        flags = space->flags[idx];
    bool            changed;
    spatial_node_t  child;

    if (SPATIAL_UNLIKELY(!(flags & SPATIAL_NODE_ALIVE))) continue;

    if (SPATIAL_UNLIKELY(flags & SPATIAL_NODE_HAS_MATRIX)) {
      mat4 *local_m = &space->matrix_overrides[idx]->local;
      spatial_node_t parent = space->parents[idx];
      if (spatial_node_is_null(parent)) {
        glm_mat4_copy(*local_m, space->world_matrices[idx]);
      } else {
        glm_mat4_mul(space->world_matrices[parent.index], *local_m,
                     space->world_matrices[idx]);
      }
      /* best-effort decompose world mat into world pose */
      {
        spatial_pose_t wp;
        spatial_mat4_to_pose(space->world_matrices[idx], &wp);
        glm_vec3_copy(wp.position, space->world_positions[idx]);
        space->world_positions[idx][3] = 0.0f;
        glm_quat_copy(wp.rotation, space->world_rotations[idx]);
        glm_vec3_copy(wp.scale,    space->world_scales[idx]);
        space->world_scales[idx][3]    = 0.0f;
      }
      space->versions[idx]++;
      changed = true;
    } else if (flags & SPATIAL_NODE_PHYSICS_OWNS) {
      /* world written externally. Refresh matrix from world pose. */
      spatial__pose_slots_to_mat4(space->world_positions[idx],
                                  space->world_rotations[idx],
                                  space->world_scales[idx],
                                  space->world_matrices[idx]);
      space->versions[idx]++;
      changed = true;
    } else {
      vec4   new_p, new_s;
      versor new_q;
      spatial_node_t parent = space->parents[idx];

      if (spatial_node_is_null(parent)) {
        glm_vec4_copy(space->local_positions[idx], new_p);
        glm_quat_copy(space->local_rotations[idx], new_q);
        glm_vec4_copy(space->local_scales[idx],    new_s);
      } else {
        spatial__compose_slots(space->world_positions[parent.index],
                               space->world_rotations[parent.index],
                               space->world_scales[parent.index],
                               space->local_positions[idx],
                               space->local_rotations[idx],
                               space->local_scales[idx],
                               new_p, new_q, new_s);
      }

      changed = e.parent_changed
             || (flags & dirty_mask)
             || spatial__pose_slots_differ(space->world_positions[idx],
                                           space->world_rotations[idx],
                                           space->world_scales[idx],
                                           new_p, new_q, new_s);

      if (changed) {
        glm_vec4_copy(new_p, space->world_positions[idx]);
        glm_quat_copy(new_q, space->world_rotations[idx]);
        glm_vec4_copy(new_s, space->world_scales[idx]);
        spatial__pose_slots_to_mat4(new_p, new_q, new_s, space->world_matrices[idx]);
        space->versions[idx]++;
      }
    }

    space->flags[idx] = flags & ~dirty_mask;

    /* push children */
    child = space->first_children[idx];
    while (!spatial_node_is_null(child)) {
      if (SPATIAL_UNLIKELY(sp >= ctx->capacity)) {
        spatial__ctx_grow(ctx, ctx->capacity * 2);
        stack = ctx->stack;
      }
      stack[sp++] = (spatial__trav_t){ child, changed ? 1u : 0u };
      child = space->next_siblings[child.index];
    }
  }
}

/* ============================================================== */
/* Thread pool for parallel dirty-root traversal.                   */
/* Dirty roots are disjoint subtrees (guaranteed by push_dirty       */
/* ancestor dedupe), so threads can traverse them independently.    */
/* ============================================================== */

#if SPATIAL_HAS_THREADS

#define SPATIAL_PARALLEL_THRESHOLD 2   /* only dispatch when >=2 roots */

typedef struct spatial__worker_t {
  pthread_t           thread;
  spatial__trav_ctx_t ctx;
  struct spatial__pool_t *pool;
} spatial__worker_t;

typedef struct spatial__pool_t {
  spatial__worker_t *workers;
  uint32_t           worker_count;

  pthread_mutex_t    mutex;
  pthread_cond_t     work_cv;
  pthread_cond_t     done_cv;

  uint32_t           next_work;     /* next dirty_root index to claim */
  uint32_t           work_total;    /* dirty_count this dispatch      */
  uint32_t           workers_done;  /* items completed                */
  bool               shutdown;

  spatial_space_t   *space;
} spatial__pool_t;

static
void *
spatial__worker_main(void *arg) {
  spatial__worker_t *w    = (spatial__worker_t *)arg;
  spatial__pool_t   *pool = w->pool;

  for (;;) {
    uint32_t       work_id;
    spatial_node_t root;

    pthread_mutex_lock(&pool->mutex);
    while (!pool->shutdown && pool->next_work >= pool->work_total) {
      pthread_cond_wait(&pool->work_cv, &pool->mutex);
    }
    if (pool->shutdown) {
      pthread_mutex_unlock(&pool->mutex);
      break;
    }
    work_id = pool->next_work++;
    pthread_mutex_unlock(&pool->mutex);

    root = pool->space->dirty_roots[work_id];
    spatial__traverse_iter(pool->space, root, &w->ctx);

    pthread_mutex_lock(&pool->mutex);
    pool->workers_done++;
    if (pool->workers_done >= pool->work_total) {
      pthread_cond_signal(&pool->done_cv);
    }
    pthread_mutex_unlock(&pool->mutex);
  }
  return NULL;
}

static
spatial__pool_t *
spatial__pool_create(spatial_space_t *space, uint32_t n) {
  spatial__pool_t *p;
  uint32_t         i;

  p = calloc(1, sizeof(*p));
  p->worker_count = n;
  p->space        = space;
  p->workers      = calloc(n, sizeof(spatial__worker_t));
  pthread_mutex_init(&p->mutex, NULL);
  pthread_cond_init(&p->work_cv, NULL);
  pthread_cond_init(&p->done_cv, NULL);

  for (i = 0; i < n; i++) {
    p->workers[i].pool = p;
    pthread_create(&p->workers[i].thread, NULL, spatial__worker_main, &p->workers[i]);
  }
  return p;
}

static
void
spatial__pool_destroy(spatial__pool_t *p) {
  uint32_t i;
  if (!p) return;

  pthread_mutex_lock(&p->mutex);
  p->shutdown = true;
  pthread_cond_broadcast(&p->work_cv);
  pthread_mutex_unlock(&p->mutex);

  for (i = 0; i < p->worker_count; i++) {
    pthread_join(p->workers[i].thread, NULL);
    free(p->workers[i].ctx.stack);
  }
  pthread_mutex_destroy(&p->mutex);
  pthread_cond_destroy(&p->work_cv);
  pthread_cond_destroy(&p->done_cv);
  free(p->workers);
  free(p);
}

static
void
spatial__pool_dispatch(spatial__pool_t *p, uint32_t total) {
  if (total == 0) return;

  pthread_mutex_lock(&p->mutex);
  p->next_work    = 0;
  p->work_total   = total;
  p->workers_done = 0;
  pthread_cond_broadcast(&p->work_cv);
  while (p->workers_done < total) {
    pthread_cond_wait(&p->done_cv, &p->mutex);
  }
  pthread_mutex_unlock(&p->mutex);
}

SPATIAL_EXPORT
void
spatial_space_enable_parallel(spatial_space_t *space, uint32_t thread_count) {
  if (!space || space->pool) return;
  if (thread_count == 0) {
    long n = sysconf(_SC_NPROCESSORS_ONLN);
    thread_count = (n > 1) ? (uint32_t)n : 2;
  }
  space->pool = spatial__pool_create(space, thread_count);
}

#else  /* !SPATIAL_HAS_THREADS */

SPATIAL_EXPORT
void
spatial_space_enable_parallel(spatial_space_t *space, uint32_t thread_count) {
  (void)space; (void)thread_count;
  /* threads disabled on this platform; spatial_update stays sequential */
}

#endif

SPATIAL_EXPORT
void
spatial_update(spatial_space_t * __restrict space) {
  spatial__trav_ctx_t main_ctx;
  uint32_t            i;

  if (!space) return;

  if (space->dirty_count == 0) {
    space->update_version++;
    return;
  }

#if SPATIAL_HAS_THREADS
  if (space->pool && space->dirty_count >= SPATIAL_PARALLEL_THRESHOLD) {
    spatial__pool_dispatch((spatial__pool_t *)space->pool, space->dirty_count);
    space->dirty_count = 0;
    space->update_version++;
    return;
  }
#endif

  /* Sequential path. Reuse space->trav_stack as main_ctx storage. */
  main_ctx.stack    = (spatial__trav_t *)space->trav_stack;
  main_ctx.capacity = space->trav_capacity;

  for (i = 0; i < space->dirty_count; i++) {
    spatial__traverse_iter(space, space->dirty_roots[i], &main_ctx);
  }

  /* reclaim possibly-grown storage into space */
  space->trav_stack    = main_ctx.stack;
  space->trav_capacity = main_ctx.capacity;

  space->dirty_count = 0;
  space->update_version++;
}

/* ================================================================ */
/* 2D implementation. Parallel SoA, parallel API, suffix _2.         */
/* ================================================================ */

static
void
spatial__grow_arrays2(spatial_space2_t *space, uint32_t new_cap) {
  uint32_t old_cap = space->capacity;

  space->generations     = realloc(space->generations,     sizeof(uint32_t)       * new_cap);
  space->parents         = realloc(space->parents,         sizeof(spatial_node_t) * new_cap);
  space->first_children  = realloc(space->first_children,  sizeof(spatial_node_t) * new_cap);
  space->next_siblings   = realloc(space->next_siblings,   sizeof(spatial_node_t) * new_cap);
  space->local_positions = realloc(space->local_positions, sizeof(vec2)           * new_cap);
  space->local_rotations = realloc(space->local_rotations, sizeof(spatial_rot2_t) * new_cap);
  space->local_scales    = realloc(space->local_scales,    sizeof(vec2)           * new_cap);
  space->world_positions = realloc(space->world_positions, sizeof(vec2)           * new_cap);
  space->world_rotations = realloc(space->world_rotations, sizeof(spatial_rot2_t) * new_cap);
  space->world_scales    = realloc(space->world_scales,    sizeof(vec2)           * new_cap);
  space->world_matrices  = realloc(space->world_matrices,  sizeof(mat3)           * new_cap);
  space->flags           = realloc(space->flags,           sizeof(uint32_t)       * new_cap);
  space->versions        = realloc(space->versions,        sizeof(uint32_t)       * new_cap);
  space->users           = realloc(space->users,           sizeof(void *)         * new_cap);

  if (new_cap > old_cap) {
    uint32_t delta = new_cap - old_cap;
    memset(space->generations    + old_cap, 0, sizeof(uint32_t)       * delta);
    memset(space->parents        + old_cap, 0, sizeof(spatial_node_t) * delta);
    memset(space->first_children + old_cap, 0, sizeof(spatial_node_t) * delta);
    memset(space->next_siblings  + old_cap, 0, sizeof(spatial_node_t) * delta);
    memset(space->flags          + old_cap, 0, sizeof(uint32_t)       * delta);
    memset(space->versions       + old_cap, 0, sizeof(uint32_t)       * delta);
    memset(space->users          + old_cap, 0, sizeof(void *)         * delta);
  }

  space->capacity = new_cap;
}

static
uint32_t
spatial__alloc_slot2(spatial_space2_t *space) {
  uint32_t slot;
  if (space->free_head != 0) {
    slot = space->free_head;
    space->free_head = space->parents[slot].index;
  } else {
    if (SPATIAL_UNLIKELY(space->count >= space->capacity)) {
      spatial__grow_arrays2(space, space->capacity * 2);
    }
    slot = space->count++;
  }
  return slot;
}

static
void
spatial__push_dirty2(spatial_space2_t *space, spatial_node_t handle) {
  spatial_node_t a;
  const uint32_t dirty_mask = SPATIAL_NODE_DIRTY_LOCAL | SPATIAL_NODE_DIRTY_WORLD;

  a = space->parents[handle.index];
  while (!spatial_node_is_null(a)) {
    if (a.index >= space->capacity) break;
    if (space->flags[a.index] & dirty_mask) return;
    a = space->parents[a.index];
  }

  if (SPATIAL_UNLIKELY(space->dirty_count >= space->dirty_capacity)) {
    uint32_t nc = space->dirty_capacity * 2;
    space->dirty_roots    = realloc(space->dirty_roots, sizeof(spatial_node_t) * nc);
    space->dirty_capacity = nc;
  }
  space->dirty_roots[space->dirty_count++] = handle;
}

static
void
spatial__ensure_trav_capacity2(spatial_space2_t *space, uint32_t want) {
  if (space->trav_capacity < want) {
    uint32_t nc = space->trav_capacity ? space->trav_capacity : SPATIAL_INITIAL_TRAV_CAPACITY;
    while (nc < want) nc *= 2;
    space->trav_stack    = realloc(space->trav_stack, sizeof(spatial__trav_t) * nc);
    space->trav_capacity = nc;
  }
}

SPATIAL_EXPORT
spatial_space2_t *
spatial_space2_create(uint32_t initial_capacity) {
  spatial_space2_t *space;
  uint32_t          slot;

  if (initial_capacity < 2) initial_capacity = 16;

  space = calloc(1, sizeof(spatial_space2_t));
  space->count     = 1;
  space->free_head = 0;
  spatial__grow_arrays2(space, initial_capacity);

  space->dirty_roots    = calloc(SPATIAL_INITIAL_DIRTY_CAPACITY, sizeof(spatial_node_t));
  space->dirty_capacity = SPATIAL_INITIAL_DIRTY_CAPACITY;

  slot = spatial__alloc_slot2(space);
  space->parents[slot]        = SPATIAL_NODE_NULL;
  space->first_children[slot] = SPATIAL_NODE_NULL;
  space->next_siblings[slot]  = SPATIAL_NODE_NULL;
  space->local_positions[slot][0] = 0; space->local_positions[slot][1] = 0;
  space->local_rotations[slot]    = (spatial_rot2_t){ .c = 1.0f, .s = 0.0f };
  space->local_scales[slot][0]    = 1; space->local_scales[slot][1]    = 1;
  space->world_positions[slot][0] = 0; space->world_positions[slot][1] = 0;
  space->world_rotations[slot]    = (spatial_rot2_t){ .c = 1.0f, .s = 0.0f };
  space->world_scales[slot][0]    = 1; space->world_scales[slot][1]    = 1;
  glm_mat3_identity(space->world_matrices[slot]);
  space->flags[slot]       = SPATIAL_NODE_ALIVE;
  space->versions[slot]    = 1;
  space->users[slot]       = NULL;
  space->generations[slot] = 1;
  space->root = (spatial_node_t){ .index = slot, .generation = 1 };

  return space;
}

SPATIAL_EXPORT
void
spatial_space2_destroy(spatial_space2_t *space) {
  if (!space) return;
  free(space->generations);
  free(space->parents);
  free(space->first_children);
  free(space->next_siblings);
  free(space->local_positions);
  free(space->local_rotations);
  free(space->local_scales);
  free(space->world_positions);
  free(space->world_rotations);
  free(space->world_scales);
  free(space->world_matrices);
  free(space->flags);
  free(space->versions);
  free(space->users);
  free(space->dirty_roots);
  free(space->trav_stack);
  free(space);
}

SPATIAL_EXPORT
bool
spatial_node2_valid(const spatial_space2_t *space, spatial_node_t handle) {
  if (SPATIAL_UNLIKELY(handle.index == 0 || handle.index >= space->capacity)) return false;
  if (SPATIAL_UNLIKELY(space->generations[handle.index] != handle.generation)) return false;
  return (space->flags[handle.index] & SPATIAL_NODE_ALIVE) != 0;
}

SPATIAL_EXPORT
spatial_node_t
spatial_node2_create(spatial_space2_t       *space,
                     spatial_node_t          parent,
                     const spatial_pose2_t  *local) {
  uint32_t       slot;
  spatial_node_t handle;

  if (spatial_node_is_null(parent)) parent = space->root;
  if (SPATIAL_UNLIKELY(!spatial_node2_valid(space, parent))) return SPATIAL_NODE_NULL;

  slot = spatial__alloc_slot2(space);

  space->parents[slot]          = parent;
  space->first_children[slot]   = SPATIAL_NODE_NULL;
  space->next_siblings[slot]    = space->first_children[parent.index];
  space->first_children[parent.index] = (spatial_node_t){ .index = slot, .generation = 0 };

  if (local) {
    space->local_positions[slot][0] = local->position[0];
    space->local_positions[slot][1] = local->position[1];
    space->local_rotations[slot]    = local->rotation;
    space->local_scales[slot][0]    = local->scale[0];
    space->local_scales[slot][1]    = local->scale[1];
  } else {
    space->local_positions[slot][0] = 0; space->local_positions[slot][1] = 0;
    space->local_rotations[slot]    = (spatial_rot2_t){ .c = 1.0f, .s = 0.0f };
    space->local_scales[slot][0]    = 1; space->local_scales[slot][1]    = 1;
  }
  space->world_positions[slot][0] = 0; space->world_positions[slot][1] = 0;
  space->world_rotations[slot]    = (spatial_rot2_t){ .c = 1.0f, .s = 0.0f };
  space->world_scales[slot][0]    = 1; space->world_scales[slot][1]    = 1;
  glm_mat3_identity(space->world_matrices[slot]);
  space->flags[slot]    = SPATIAL_NODE_ALIVE | SPATIAL_NODE_DIRTY_LOCAL;
  space->versions[slot] = 1;
  space->users[slot]    = NULL;

  space->generations[slot]++;
  if (SPATIAL_UNLIKELY(space->generations[slot] == 0)) space->generations[slot] = 1;
  handle = (spatial_node_t){ .index = slot, .generation = space->generations[slot] };
  space->first_children[parent.index] = handle;

  spatial__push_dirty2(space, handle);
  return handle;
}

static
void
spatial__unlink_from_parent2(spatial_space2_t *space, spatial_node_t handle) {
  spatial_node_t parent, cur;

  parent = space->parents[handle.index];
  if (spatial_node_is_null(parent)) return;

  if (spatial_node_eq(space->first_children[parent.index], handle)) {
    space->first_children[parent.index] = space->next_siblings[handle.index];
  } else {
    cur = space->first_children[parent.index];
    while (!spatial_node_is_null(cur)) {
      if (spatial_node_eq(space->next_siblings[cur.index], handle)) {
        space->next_siblings[cur.index] = space->next_siblings[handle.index];
        break;
      }
      cur = space->next_siblings[cur.index];
    }
  }
  space->next_siblings[handle.index] = SPATIAL_NODE_NULL;
}

SPATIAL_EXPORT
void
spatial_node2_destroy(spatial_space2_t *space, spatial_node_t handle) {
  spatial_node_t child;

  if (!spatial_node2_valid(space, handle)) return;
  if (spatial_node_eq(handle, space->root)) return;

  child = space->first_children[handle.index];
  while (!spatial_node_is_null(child)) {
    spatial_node_t next = space->next_siblings[child.index];
    spatial_node2_destroy(space, child);
    child = next;
  }

  spatial__unlink_from_parent2(space, handle);

  space->flags[handle.index] = 0;
  space->parents[handle.index] = (spatial_node_t){ .index = space->free_head, .generation = 0 };
  space->free_head = handle.index;
  space->generations[handle.index]++;
  if (SPATIAL_UNLIKELY(space->generations[handle.index] == 0)) space->generations[handle.index] = 1;
}

static
bool
spatial__is_descendant2(const spatial_space2_t *space,
                        spatial_node_t          root,
                        spatial_node_t          candidate) {
  spatial_node_t cur = candidate;
  while (!spatial_node_is_null(cur)) {
    if (spatial_node_eq(cur, root)) return true;
    if (cur.index >= space->capacity) return false;
    cur = space->parents[cur.index];
  }
  return false;
}

SPATIAL_EXPORT
bool
spatial_node2_attach(spatial_space2_t *space,
                     spatial_node_t    child,
                     spatial_node_t    new_parent) {
  if (!spatial_node2_valid(space, child))      return false;
  if (spatial_node_is_null(new_parent))        new_parent = space->root;
  if (!spatial_node2_valid(space, new_parent)) return false;
  if (spatial_node_eq(child, new_parent))      return false;
  if (spatial__is_descendant2(space, child, new_parent)) return false;

  spatial__unlink_from_parent2(space, child);

  space->parents[child.index]             = new_parent;
  space->next_siblings[child.index]       = space->first_children[new_parent.index];
  space->first_children[new_parent.index] = child;

  space->flags[child.index] |= SPATIAL_NODE_DIRTY_LOCAL;
  spatial__push_dirty2(space, child);
  return true;
}

SPATIAL_EXPORT
void
spatial_node2_set_local(spatial_space2_t       *space,
                        spatial_node_t          handle,
                        const spatial_pose2_t  *local) {
  if (!spatial_node2_valid(space, handle) || !local) return;
  space->local_positions[handle.index][0] = local->position[0];
  space->local_positions[handle.index][1] = local->position[1];
  space->local_rotations[handle.index]    = local->rotation;
  space->local_scales[handle.index][0]    = local->scale[0];
  space->local_scales[handle.index][1]    = local->scale[1];
  space->flags[handle.index] |= SPATIAL_NODE_DIRTY_LOCAL;
  spatial__push_dirty2(space, handle);
}

SPATIAL_EXPORT
bool
spatial_node2_get_local(const spatial_space2_t *space,
                        spatial_node_t          handle,
                        spatial_pose2_t        *out) {
  if (!spatial_node2_valid(space, handle) || !out) return false;
  out->position[0] = space->local_positions[handle.index][0];
  out->position[1] = space->local_positions[handle.index][1];
  out->rotation    = space->local_rotations[handle.index];
  out->scale[0]    = space->local_scales[handle.index][0];
  out->scale[1]    = space->local_scales[handle.index][1];
  return true;
}

SPATIAL_EXPORT
void
spatial_node2_set_world_physics(spatial_space2_t       *space,
                                spatial_node_t          handle,
                                const spatial_pose2_t  *world) {
  if (!spatial_node2_valid(space, handle) || !world) return;
  space->world_positions[handle.index][0] = world->position[0];
  space->world_positions[handle.index][1] = world->position[1];
  space->world_rotations[handle.index]    = world->rotation;
  space->world_scales[handle.index][0]    = world->scale[0];
  space->world_scales[handle.index][1]    = world->scale[1];
  space->flags[handle.index] |= SPATIAL_NODE_DIRTY_WORLD | SPATIAL_NODE_PHYSICS_OWNS;
  spatial__push_dirty2(space, handle);
}

SPATIAL_EXPORT
bool
spatial_node2_get_world(const spatial_space2_t *space,
                        spatial_node_t          handle,
                        spatial_pose2_t        *out) {
  if (!spatial_node2_valid(space, handle) || !out) return false;
  out->position[0] = space->world_positions[handle.index][0];
  out->position[1] = space->world_positions[handle.index][1];
  out->rotation    = space->world_rotations[handle.index];
  out->scale[0]    = space->world_scales[handle.index][0];
  out->scale[1]    = space->world_scales[handle.index][1];
  return true;
}

SPATIAL_EXPORT
bool
spatial_node2_get_world_matrix(const spatial_space2_t *space,
                               spatial_node_t          handle,
                               mat3                    out) {
  if (!spatial_node2_valid(space, handle)) return false;
  glm_mat3_copy((vec3 *)space->world_matrices[handle.index], out);
  return true;
}

SPATIAL_EXPORT
uint32_t
spatial_node2_get_flags(const spatial_space2_t *space, spatial_node_t handle) {
  if (!spatial_node2_valid(space, handle)) return 0;
  return space->flags[handle.index];
}

SPATIAL_EXPORT
uint32_t
spatial_node2_get_version(const spatial_space2_t *space, spatial_node_t handle) {
  if (!spatial_node2_valid(space, handle)) return 0;
  return space->versions[handle.index];
}

SPATIAL_EXPORT
spatial_node_t
spatial_node2_get_parent(const spatial_space2_t *space, spatial_node_t handle) {
  if (!spatial_node2_valid(space, handle)) return SPATIAL_NODE_NULL;
  return space->parents[handle.index];
}

static SPATIAL_INLINE
bool
spatial__pose2_slots_differ(const vec2 pa, spatial_rot2_t ra, const vec2 sa,
                            const vec2 pb, spatial_rot2_t rb, const vec2 sb) {
  const float eps = 1e-7f;
  return fabsf(pa[0] - pb[0]) > eps
      || fabsf(pa[1] - pb[1]) > eps
      || fabsf(sa[0] - sb[0]) > eps
      || fabsf(sa[1] - sb[1]) > eps
      || fabsf(ra.c - rb.c)   > eps
      || fabsf(ra.s - rb.s)   > eps;
}

static SPATIAL_INLINE
void
spatial__pose2_slots_to_mat3(const vec2 p, spatial_rot2_t r, const vec2 s, mat3 out) {
  out[0][0] =  r.c * s[0]; out[0][1] =  r.s * s[0]; out[0][2] = 0.0f;
  out[1][0] = -r.s * s[1]; out[1][1] =  r.c * s[1]; out[1][2] = 0.0f;
  out[2][0] =  p[0];       out[2][1] =  p[1];       out[2][2] = 1.0f;
}

static
void
spatial__traverse_iter2(spatial_space2_t *space, spatial_node_t start) {
  spatial__trav_t *stack;
  uint32_t         sp = 0;
  const uint32_t   dirty_mask = SPATIAL_NODE_DIRTY_LOCAL | SPATIAL_NODE_DIRTY_WORLD;

  if (!spatial_node2_valid(space, start)) return;

  spatial__ensure_trav_capacity2(space, space->capacity);
  stack = (spatial__trav_t *)space->trav_stack;
  stack[sp++] = (spatial__trav_t){ start, 0u };

  while (sp > 0) {
    spatial__trav_t e     = stack[--sp];
    uint32_t        idx   = e.node.index;
    uint32_t        flags = space->flags[idx];
    bool            changed;
    spatial_node_t  child;

    if (SPATIAL_UNLIKELY(!(flags & SPATIAL_NODE_ALIVE))) continue;

    if (flags & SPATIAL_NODE_PHYSICS_OWNS) {
      spatial__pose2_slots_to_mat3(space->world_positions[idx],
                                   space->world_rotations[idx],
                                   space->world_scales[idx],
                                   space->world_matrices[idx]);
      space->versions[idx]++;
      changed = true;
    } else {
      vec2           new_p, new_s, scaled, rotated;
      spatial_rot2_t new_r;
      spatial_node_t parent = space->parents[idx];

      if (spatial_node_is_null(parent)) {
        new_p[0] = space->local_positions[idx][0];
        new_p[1] = space->local_positions[idx][1];
        new_r    = space->local_rotations[idx];
        new_s[0] = space->local_scales[idx][0];
        new_s[1] = space->local_scales[idx][1];
      } else {
        uint32_t p = parent.index;
        scaled[0] = space->world_scales[p][0] * space->local_positions[idx][0];
        scaled[1] = space->world_scales[p][1] * space->local_positions[idx][1];
        spatial_rot2_rotatev(space->world_rotations[p], scaled, rotated);
        new_p[0] = space->world_positions[p][0] + rotated[0];
        new_p[1] = space->world_positions[p][1] + rotated[1];
        spatial_rot2_mul(space->world_rotations[p], space->local_rotations[idx], &new_r);
        new_s[0] = space->world_scales[p][0] * space->local_scales[idx][0];
        new_s[1] = space->world_scales[p][1] * space->local_scales[idx][1];
      }

      changed = e.parent_changed
             || (flags & dirty_mask)
             || spatial__pose2_slots_differ(space->world_positions[idx],
                                            space->world_rotations[idx],
                                            space->world_scales[idx],
                                            new_p, new_r, new_s);

      if (changed) {
        space->world_positions[idx][0] = new_p[0];
        space->world_positions[idx][1] = new_p[1];
        space->world_rotations[idx]    = new_r;
        space->world_scales[idx][0]    = new_s[0];
        space->world_scales[idx][1]    = new_s[1];
        spatial__pose2_slots_to_mat3(new_p, new_r, new_s, space->world_matrices[idx]);
        space->versions[idx]++;
      }
    }

    space->flags[idx] = flags & ~dirty_mask;

    child = space->first_children[idx];
    while (!spatial_node_is_null(child)) {
      if (SPATIAL_UNLIKELY(sp >= space->trav_capacity)) {
        spatial__ensure_trav_capacity2(space, space->trav_capacity * 2);
        stack = (spatial__trav_t *)space->trav_stack;
      }
      stack[sp++] = (spatial__trav_t){ child, changed ? 1u : 0u };
      child = space->next_siblings[child.index];
    }
  }
}

SPATIAL_EXPORT
void
spatial_update2(spatial_space2_t *space) {
  uint32_t i;
  if (!space) return;

  for (i = 0; i < space->dirty_count; i++) {
    spatial__traverse_iter2(space, space->dirty_roots[i]);
  }

  space->dirty_count = 0;
  space->update_version++;
}
