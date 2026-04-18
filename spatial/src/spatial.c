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

#define SPATIAL_INITIAL_DIRTY_CAPACITY 16

/* --------------------------------------------------------------- */
/* Pose conversions and composition.                                */
/* --------------------------------------------------------------- */

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

/* 2D math. */

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
  vec2 scaled, rotated;
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

SPATIAL_EXPORT
void
spatial_pose_compose(const spatial_pose_t * __restrict a,
                  const spatial_pose_t * __restrict b,
                  spatial_pose_t       * __restrict out) {
  vec3 scaled, rotated;
  versor rot;
  vec3 pos, scl;

  /* position = a.pos + a.rot * (a.scale * b.pos) */
  glm_vec3_mul((float *)a->scale, (float *)b->position, scaled);
  glm_quat_rotatev((float *)a->rotation, scaled, rotated);
  glm_vec3_add((float *)a->position, rotated, pos);

  /* rotation = a.rot * b.rot */
  glm_quat_mul((float *)a->rotation, (float *)b->rotation, rot);

  /* scale = a.scale * b.scale */
  glm_vec3_mul((float *)a->scale, (float *)b->scale, scl);

  glm_vec3_copy(pos, out->position);
  glm_quat_copy(rot, out->rotation);
  glm_vec3_copy(scl, out->scale);
}

/* --------------------------------------------------------------- */
/* Space and node storage.                                          */
/* --------------------------------------------------------------- */

static
uint32_t
spatial__alloc_slot(spatial_space_t * __restrict space) {
  uint32_t slot;

  if (space->free_head != 0) {
    slot = space->free_head;
    /* free list is threaded through parent.index of freed nodes */
    space->free_head = space->nodes[slot].parent.index;
  } else {
    if (space->count >= space->capacity) {
      uint32_t new_cap = space->capacity * 2;
      space->nodes       = realloc(space->nodes,
                                   sizeof(spatial_node_data_t) * new_cap);
      space->generations = realloc(space->generations,
                                   sizeof(uint32_t) * new_cap);
      memset(space->nodes + space->capacity, 0,
             sizeof(spatial_node_data_t) * (new_cap - space->capacity));
      memset(space->generations + space->capacity, 0,
             sizeof(uint32_t) * (new_cap - space->capacity));
      space->capacity = new_cap;
    }
    slot = space->count++;
  }

  return slot;
}

static
void
spatial__push_dirty(spatial_space_t * __restrict space, spatial_node_t node) {
  if (space->dirty_count >= space->dirty_capacity) {
    uint32_t new_cap = space->dirty_capacity * 2;
    space->dirty_roots    = realloc(space->dirty_roots,
                                    sizeof(spatial_node_t) * new_cap);
    space->dirty_capacity = new_cap;
  }
  space->dirty_roots[space->dirty_count++] = node;
}

SPATIAL_EXPORT
spatial_space_t *
spatial_space_create(uint32_t initial_capacity) {
  spatial_space_t *space;

  if (initial_capacity < 2) initial_capacity = 16;

  space = calloc(1, sizeof(spatial_space_t));
  space->nodes       = calloc(initial_capacity, sizeof(spatial_node_data_t));
  space->generations = calloc(initial_capacity, sizeof(uint32_t));
  space->capacity    = initial_capacity;
  space->count       = 1;  /* reserve slot 0 for SPATIAL_NODE_NULL */
  space->free_head   = 0;

  space->dirty_roots    = calloc(SPATIAL_INITIAL_DIRTY_CAPACITY,
                                 sizeof(spatial_node_t));
  space->dirty_capacity = SPATIAL_INITIAL_DIRTY_CAPACITY;

  /* create the root node at slot 1 */
  {
    uint32_t slot = spatial__alloc_slot(space);
    spatial_node_data_t *n = &space->nodes[slot];

    n->parent       = SPATIAL_NODE_NULL;
    n->first_child  = SPATIAL_NODE_NULL;
    n->next_sibling = SPATIAL_NODE_NULL;
    n->local        = SPATIAL_POSE_IDENTITY;
    n->world        = SPATIAL_POSE_IDENTITY;
    glm_mat4_identity(n->world_matrix);
    n->flags        = SPATIAL_NODE_ALIVE;
    n->version      = 1;

    space->generations[slot] = 1;
    space->root = (spatial_node_t){ .index = slot, .generation = 1 };
  }

  return space;
}

SPATIAL_EXPORT
void
spatial_space_destroy(spatial_space_t * __restrict space) {
  if (!space) return;
  free(space->nodes);
  free(space->generations);
  free(space->dirty_roots);
  free(space);
}

SPATIAL_EXPORT
bool
spatial_node_valid(const spatial_space_t * __restrict space, spatial_node_t handle) {
  if (handle.index == 0 || handle.index >= space->capacity) return false;
  if (space->generations[handle.index] != handle.generation) return false;
  return (space->nodes[handle.index].flags & SPATIAL_NODE_ALIVE) != 0;
}

SPATIAL_EXPORT
spatial_node_data_t *
spatial_node_get(spatial_space_t * __restrict space, spatial_node_t handle) {
  if (!spatial_node_valid(space, handle)) return NULL;
  return &space->nodes[handle.index];
}

SPATIAL_EXPORT
spatial_node_t
spatial_node_create(spatial_space_t      * __restrict space,
                 spatial_node_t                    parent,
                 const spatial_pose_t * __restrict local) {
  uint32_t          slot;
  spatial_node_data_t *n;
  spatial_node_t       handle;

  if (spatial_node_is_null(parent)) parent = space->root;
  if (!spatial_node_valid(space, parent)) return SPATIAL_NODE_NULL;

  slot = spatial__alloc_slot(space);
  n    = &space->nodes[slot];

  n->parent       = parent;
  n->first_child  = SPATIAL_NODE_NULL;
  n->next_sibling = SPATIAL_NODE_NULL;
  n->local        = local ? *local : SPATIAL_POSE_IDENTITY;
  n->world        = SPATIAL_POSE_IDENTITY;
  glm_mat4_identity(n->world_matrix);
  n->flags        = SPATIAL_NODE_ALIVE | SPATIAL_NODE_DIRTY_LOCAL;
  n->version      = 1;
  n->user         = NULL;

  space->generations[slot]++;
  if (space->generations[slot] == 0) space->generations[slot] = 1;
  handle = (spatial_node_t){ .index = slot, .generation = space->generations[slot] };

  /* link under parent */
  {
    spatial_node_data_t *p = &space->nodes[parent.index];
    n->next_sibling = p->first_child;
    p->first_child  = handle;
  }

  spatial__push_dirty(space, handle);
  return handle;
}

static
void
spatial__unlink_from_parent(spatial_space_t * __restrict space, spatial_node_t handle) {
  spatial_node_data_t *n, *p;
  spatial_node_t       cur;

  n = &space->nodes[handle.index];
  if (spatial_node_is_null(n->parent)) return;

  p = &space->nodes[n->parent.index];
  if (spatial_node_eq(p->first_child, handle)) {
    p->first_child = n->next_sibling;
  } else {
    cur = p->first_child;
    while (!spatial_node_is_null(cur)) {
      spatial_node_data_t *cd = &space->nodes[cur.index];
      if (spatial_node_eq(cd->next_sibling, handle)) {
        cd->next_sibling = n->next_sibling;
        break;
      }
      cur = cd->next_sibling;
    }
  }
  n->next_sibling = SPATIAL_NODE_NULL;
}

SPATIAL_EXPORT
void
spatial_node_destroy(spatial_space_t * __restrict space, spatial_node_t handle) {
  spatial_node_data_t *n;
  spatial_node_t       child;

  if (!spatial_node_valid(space, handle)) return;
  if (spatial_node_eq(handle, space->root)) return;  /* root is permanent */

  n = &space->nodes[handle.index];

  /* destroy children first (depth-first) */
  child = n->first_child;
  while (!spatial_node_is_null(child)) {
    spatial_node_t next = space->nodes[child.index].next_sibling;
    spatial_node_destroy(space, child);
    child = next;
  }

  spatial__unlink_from_parent(space, handle);

  if (n->matrix_override) {
    free(n->matrix_override);
    n->matrix_override = NULL;
  }

  n->flags = 0;  /* clear ALIVE */
  n->parent.index = space->free_head;  /* thread free list */
  space->free_head = handle.index;
  space->generations[handle.index]++;
  if (space->generations[handle.index] == 0) space->generations[handle.index] = 1;
}

/* returns true iff candidate descends from root (including root itself). */
static
bool
spatial__is_descendant(const spatial_space_t *space,
                       spatial_node_t         root,
                       spatial_node_t         candidate) {
  spatial_node_t cur = candidate;
  while (!spatial_node_is_null(cur)) {
    if (spatial_node_eq(cur, root)) return true;
    if (cur.index >= space->capacity) return false;
    cur = space->nodes[cur.index].parent;
  }
  return false;
}

SPATIAL_EXPORT
bool
spatial_node_attach(spatial_space_t * __restrict space,
                 spatial_node_t                child,
                 spatial_node_t                new_parent) {
  spatial_node_data_t *c, *p;

  if (!spatial_node_valid(space, child))      return false;
  if (spatial_node_is_null(new_parent))       new_parent = space->root;
  if (!spatial_node_valid(space, new_parent)) return false;
  if (spatial_node_eq(child, new_parent))     return false;

  /* cycle check: new_parent must not descend from child */
  if (spatial__is_descendant(space, child, new_parent)) return false;

  spatial__unlink_from_parent(space, child);

  c = &space->nodes[child.index];
  p = &space->nodes[new_parent.index];
  c->parent       = new_parent;
  c->next_sibling = p->first_child;
  p->first_child  = child;

  c->flags |= SPATIAL_NODE_DIRTY_LOCAL;
  spatial__push_dirty(space, child);
  return true;
}

SPATIAL_EXPORT
void
spatial_node_set_local(spatial_space_t      * __restrict space,
                    spatial_node_t                    handle,
                    const spatial_pose_t * __restrict local) {
  spatial_node_data_t *n = spatial_node_get(space, handle);
  if (!n || !local) return;
  n->local = *local;
  n->flags |= SPATIAL_NODE_DIRTY_LOCAL;
  spatial__push_dirty(space, handle);
}

SPATIAL_EXPORT
void
spatial_node_set_world_physics(spatial_space_t      * __restrict space,
                            spatial_node_t                    handle,
                            const spatial_pose_t * __restrict world) {
  spatial_node_data_t *n = spatial_node_get(space, handle);
  if (!n || !world) return;
  n->world = *world;
  n->flags |= SPATIAL_NODE_DIRTY_WORLD | SPATIAL_NODE_PHYSICS_OWNS;
  spatial__push_dirty(space, handle);
}

SPATIAL_EXPORT
bool
spatial_node_get_world(const spatial_space_t * __restrict space,
                    spatial_node_t                    handle,
                    spatial_pose_t        * __restrict out) {
  if (!spatial_node_valid(space, handle) || !out) return false;
  *out = space->nodes[handle.index].world;
  return true;
}

SPATIAL_EXPORT
void
spatial_node_set_matrix(spatial_space_t * __restrict space,
                        spatial_node_t                handle,
                        const mat4                    local_matrix) {
  spatial_node_data_t *n = spatial_node_get(space, handle);
  if (!n) return;

  if (!n->matrix_override) {
    n->matrix_override = calloc(1, sizeof(spatial_matrix_override_t));
  }
  glm_mat4_copy((vec4 *)local_matrix, n->matrix_override->local);

  /* best-effort decompose into pose for readers that don't know matrix path */
  spatial_mat4_to_pose(local_matrix, &n->local);

  n->flags |= SPATIAL_NODE_HAS_MATRIX | SPATIAL_NODE_DIRTY_LOCAL;
  spatial__push_dirty(space, handle);
}

SPATIAL_EXPORT
void
spatial_node_clear_matrix(spatial_space_t * __restrict space,
                          spatial_node_t                handle) {
  spatial_node_data_t *n = spatial_node_get(space, handle);
  if (!n) return;
  if (n->matrix_override) {
    free(n->matrix_override);
    n->matrix_override = NULL;
  }
  n->flags &= ~SPATIAL_NODE_HAS_MATRIX;
  n->flags |= SPATIAL_NODE_DIRTY_LOCAL;
  spatial__push_dirty(space, handle);
}

/* --------------------------------------------------------------- */
/* Update traversal. See spec/update.md.                            */
/* --------------------------------------------------------------- */

static
bool
spatial__pose_differs(const spatial_pose_t *a, const spatial_pose_t *b) {
  const float eps = 1e-7f;
  int i;

  for (i = 0; i < 3; i++) {
    if (fabsf(a->position[i] - b->position[i]) > eps) return true;
    if (fabsf(a->scale[i]    - b->scale[i])    > eps) return true;
  }
  for (i = 0; i < 4; i++) {
    if (fabsf(a->rotation[i] - b->rotation[i]) > eps) return true;
  }
  return false;
}

static
void
spatial__traverse(spatial_space_t * __restrict space, spatial_node_t handle, bool parent_changed) {
  spatial_node_data_t *n;
  spatial_pose_t       new_world;
  bool              changed;

  if (!spatial_node_valid(space, handle)) return;
  n = &space->nodes[handle.index];

  if (n->flags & SPATIAL_NODE_HAS_MATRIX) {
    /* Matrix-override path. world_matrix is authoritative; pose is a
     * best-effort view for readers that don't know about matrix mode. */
    if (spatial_node_is_null(n->parent)) {
      glm_mat4_copy(n->matrix_override->local, n->world_matrix);
    } else {
      spatial_node_data_t *p = &space->nodes[n->parent.index];
      glm_mat4_mul(p->world_matrix, n->matrix_override->local, n->world_matrix);
    }
    spatial_mat4_to_pose(n->world_matrix, &n->world);
    changed = true;
    n->version++;
  } else {
    if (n->flags & SPATIAL_NODE_PHYSICS_OWNS) {
      new_world = n->world;
    } else if (spatial_node_is_null(n->parent)) {
      new_world = n->local;
    } else {
      spatial_node_data_t *p = &space->nodes[n->parent.index];
      spatial_pose_compose(&p->world, &n->local, &new_world);
    }

    changed = parent_changed
           || (n->flags & (SPATIAL_NODE_DIRTY_LOCAL | SPATIAL_NODE_DIRTY_WORLD))
           || spatial__pose_differs(&new_world, &n->world);

    if (changed) {
      n->world = new_world;
      spatial_pose_to_mat4(&n->world, n->world_matrix);
      n->version++;
    }
  }

  n->flags &= ~(SPATIAL_NODE_DIRTY_LOCAL | SPATIAL_NODE_DIRTY_WORLD);

  /* recurse into children */
  {
    spatial_node_t child = n->first_child;
    while (!spatial_node_is_null(child)) {
      spatial_node_data_t *cd;
      spatial_node_t next;
      if (!spatial_node_valid(space, child)) break;
      cd   = &space->nodes[child.index];
      next = cd->next_sibling;
      spatial__traverse(space, child, changed);
      child = next;
    }
  }
}

SPATIAL_EXPORT
void
spatial_update(spatial_space_t * __restrict space) {
  uint32_t i;

  if (!space) return;

  for (i = 0; i < space->dirty_count; i++) {
    spatial__traverse(space, space->dirty_roots[i], false);
  }

  space->dirty_count = 0;
  space->update_version++;
}

/* ================================================================== */
/* 2D implementation. Mirrors the 3D API with suffix _2.              */
/* ================================================================== */

static
uint32_t
spatial__alloc_slot2(spatial_space2_t * __restrict space) {
  uint32_t slot;
  if (space->free_head != 0) {
    slot = space->free_head;
    space->free_head = space->nodes[slot].parent.index;
  } else {
    if (space->count >= space->capacity) {
      uint32_t new_cap = space->capacity * 2;
      space->nodes       = realloc(space->nodes,
                                   sizeof(spatial_node_data2_t) * new_cap);
      space->generations = realloc(space->generations,
                                   sizeof(uint32_t) * new_cap);
      memset(space->nodes + space->capacity, 0,
             sizeof(spatial_node_data2_t) * (new_cap - space->capacity));
      memset(space->generations + space->capacity, 0,
             sizeof(uint32_t) * (new_cap - space->capacity));
      space->capacity = new_cap;
    }
    slot = space->count++;
  }
  return slot;
}

static
void
spatial__push_dirty2(spatial_space2_t * __restrict space, spatial_node_t node) {
  if (space->dirty_count >= space->dirty_capacity) {
    uint32_t new_cap = space->dirty_capacity * 2;
    space->dirty_roots    = realloc(space->dirty_roots,
                                    sizeof(spatial_node_t) * new_cap);
    space->dirty_capacity = new_cap;
  }
  space->dirty_roots[space->dirty_count++] = node;
}

SPATIAL_EXPORT
spatial_space2_t *
spatial_space2_create(uint32_t initial_capacity) {
  spatial_space2_t *space;

  if (initial_capacity < 2) initial_capacity = 16;

  space = calloc(1, sizeof(spatial_space2_t));
  space->nodes       = calloc(initial_capacity, sizeof(spatial_node_data2_t));
  space->generations = calloc(initial_capacity, sizeof(uint32_t));
  space->capacity    = initial_capacity;
  space->count       = 1;
  space->free_head   = 0;

  space->dirty_roots    = calloc(SPATIAL_INITIAL_DIRTY_CAPACITY,
                                 sizeof(spatial_node_t));
  space->dirty_capacity = SPATIAL_INITIAL_DIRTY_CAPACITY;

  {
    uint32_t slot = spatial__alloc_slot2(space);
    spatial_node_data2_t *n = &space->nodes[slot];
    n->parent       = SPATIAL_NODE_NULL;
    n->first_child  = SPATIAL_NODE_NULL;
    n->next_sibling = SPATIAL_NODE_NULL;
    n->local        = SPATIAL_POSE2_IDENTITY;
    n->world        = SPATIAL_POSE2_IDENTITY;
    glm_mat3_identity(n->world_matrix);
    n->flags        = SPATIAL_NODE_ALIVE;
    n->version      = 1;
    space->generations[slot] = 1;
    space->root = (spatial_node_t){ .index = slot, .generation = 1 };
  }

  return space;
}

SPATIAL_EXPORT
void
spatial_space2_destroy(spatial_space2_t * space) {
  if (!space) return;
  free(space->nodes);
  free(space->generations);
  free(space->dirty_roots);
  free(space);
}

SPATIAL_EXPORT
bool
spatial_node2_valid(const spatial_space2_t *space, spatial_node_t handle) {
  if (handle.index == 0 || handle.index >= space->capacity) return false;
  if (space->generations[handle.index] != handle.generation) return false;
  return (space->nodes[handle.index].flags & SPATIAL_NODE_ALIVE) != 0;
}

SPATIAL_EXPORT
spatial_node_data2_t *
spatial_node2_get(spatial_space2_t *space, spatial_node_t handle) {
  if (!spatial_node2_valid(space, handle)) return NULL;
  return &space->nodes[handle.index];
}

SPATIAL_EXPORT
spatial_node_t
spatial_node2_create(spatial_space2_t       * __restrict space,
                     spatial_node_t                       parent,
                     const spatial_pose2_t  * __restrict  local) {
  uint32_t              slot;
  spatial_node_data2_t *n;
  spatial_node_t        handle;

  if (spatial_node_is_null(parent)) parent = space->root;
  if (!spatial_node2_valid(space, parent)) return SPATIAL_NODE_NULL;

  slot = spatial__alloc_slot2(space);
  n    = &space->nodes[slot];

  n->parent       = parent;
  n->first_child  = SPATIAL_NODE_NULL;
  n->next_sibling = SPATIAL_NODE_NULL;
  n->local        = local ? *local : SPATIAL_POSE2_IDENTITY;
  n->world        = SPATIAL_POSE2_IDENTITY;
  glm_mat3_identity(n->world_matrix);
  n->flags        = SPATIAL_NODE_ALIVE | SPATIAL_NODE_DIRTY_LOCAL;
  n->version      = 1;
  n->user         = NULL;

  space->generations[slot]++;
  if (space->generations[slot] == 0) space->generations[slot] = 1;
  handle = (spatial_node_t){ .index = slot, .generation = space->generations[slot] };

  {
    spatial_node_data2_t *p = &space->nodes[parent.index];
    n->next_sibling = p->first_child;
    p->first_child  = handle;
  }

  spatial__push_dirty2(space, handle);
  return handle;
}

static
void
spatial__unlink_from_parent2(spatial_space2_t *space, spatial_node_t handle) {
  spatial_node_data2_t *n, *p;
  spatial_node_t        cur;

  n = &space->nodes[handle.index];
  if (spatial_node_is_null(n->parent)) return;

  p = &space->nodes[n->parent.index];
  if (spatial_node_eq(p->first_child, handle)) {
    p->first_child = n->next_sibling;
  } else {
    cur = p->first_child;
    while (!spatial_node_is_null(cur)) {
      spatial_node_data2_t *cd = &space->nodes[cur.index];
      if (spatial_node_eq(cd->next_sibling, handle)) {
        cd->next_sibling = n->next_sibling;
        break;
      }
      cur = cd->next_sibling;
    }
  }
  n->next_sibling = SPATIAL_NODE_NULL;
}

SPATIAL_EXPORT
void
spatial_node2_destroy(spatial_space2_t *space, spatial_node_t handle) {
  spatial_node_data2_t *n;
  spatial_node_t        child;

  if (!spatial_node2_valid(space, handle)) return;
  if (spatial_node_eq(handle, space->root)) return;

  n = &space->nodes[handle.index];

  child = n->first_child;
  while (!spatial_node_is_null(child)) {
    spatial_node_t next = space->nodes[child.index].next_sibling;
    spatial_node2_destroy(space, child);
    child = next;
  }

  spatial__unlink_from_parent2(space, handle);

  n->flags = 0;
  n->parent.index = space->free_head;
  space->free_head = handle.index;
  space->generations[handle.index]++;
  if (space->generations[handle.index] == 0) space->generations[handle.index] = 1;
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
    cur = space->nodes[cur.index].parent;
  }
  return false;
}

SPATIAL_EXPORT
bool
spatial_node2_attach(spatial_space2_t *space,
                     spatial_node_t    child,
                     spatial_node_t    new_parent) {
  spatial_node_data2_t *c, *p;

  if (!spatial_node2_valid(space, child))      return false;
  if (spatial_node_is_null(new_parent))        new_parent = space->root;
  if (!spatial_node2_valid(space, new_parent)) return false;
  if (spatial_node_eq(child, new_parent))      return false;
  if (spatial__is_descendant2(space, child, new_parent)) return false;

  spatial__unlink_from_parent2(space, child);

  c = &space->nodes[child.index];
  p = &space->nodes[new_parent.index];
  c->parent       = new_parent;
  c->next_sibling = p->first_child;
  p->first_child  = child;

  c->flags |= SPATIAL_NODE_DIRTY_LOCAL;
  spatial__push_dirty2(space, child);
  return true;
}

SPATIAL_EXPORT
void
spatial_node2_set_local(spatial_space2_t       * __restrict space,
                        spatial_node_t                       handle,
                        const spatial_pose2_t  * __restrict  local) {
  spatial_node_data2_t *n = spatial_node2_get(space, handle);
  if (!n || !local) return;
  n->local = *local;
  n->flags |= SPATIAL_NODE_DIRTY_LOCAL;
  spatial__push_dirty2(space, handle);
}

SPATIAL_EXPORT
void
spatial_node2_set_world_physics(spatial_space2_t       * __restrict space,
                                spatial_node_t                       handle,
                                const spatial_pose2_t  * __restrict  world) {
  spatial_node_data2_t *n = spatial_node2_get(space, handle);
  if (!n || !world) return;
  n->world = *world;
  n->flags |= SPATIAL_NODE_DIRTY_WORLD | SPATIAL_NODE_PHYSICS_OWNS;
  spatial__push_dirty2(space, handle);
}

SPATIAL_EXPORT
bool
spatial_node2_get_world(const spatial_space2_t *space,
                        spatial_node_t         handle,
                        spatial_pose2_t       *out) {
  if (!spatial_node2_valid(space, handle) || !out) return false;
  *out = space->nodes[handle.index].world;
  return true;
}

static
bool
spatial__pose2_differs(const spatial_pose2_t *a, const spatial_pose2_t *b) {
  const float eps = 1e-7f;
  if (fabsf(a->position[0] - b->position[0]) > eps) return true;
  if (fabsf(a->position[1] - b->position[1]) > eps) return true;
  if (fabsf(a->scale[0]    - b->scale[0])    > eps) return true;
  if (fabsf(a->scale[1]    - b->scale[1])    > eps) return true;
  if (fabsf(a->rotation.c  - b->rotation.c)  > eps) return true;
  if (fabsf(a->rotation.s  - b->rotation.s)  > eps) return true;
  return false;
}

static
void
spatial__traverse2(spatial_space2_t *space,
                   spatial_node_t    handle,
                   bool              parent_changed) {
  spatial_node_data2_t *n;
  spatial_pose2_t       new_world;
  bool                  changed;

  if (!spatial_node2_valid(space, handle)) return;
  n = &space->nodes[handle.index];

  if (n->flags & SPATIAL_NODE_PHYSICS_OWNS) {
    new_world = n->world;
  } else if (spatial_node_is_null(n->parent)) {
    new_world = n->local;
  } else {
    spatial_node_data2_t *p = &space->nodes[n->parent.index];
    spatial_pose2_compose(&p->world, &n->local, &new_world);
  }

  changed = parent_changed
         || (n->flags & (SPATIAL_NODE_DIRTY_LOCAL | SPATIAL_NODE_DIRTY_WORLD))
         || spatial__pose2_differs(&new_world, &n->world);

  if (changed) {
    n->world = new_world;
    spatial_pose2_to_mat3(&n->world, n->world_matrix);
    n->version++;
  }

  n->flags &= ~(SPATIAL_NODE_DIRTY_LOCAL | SPATIAL_NODE_DIRTY_WORLD);

  {
    spatial_node_t child = n->first_child;
    while (!spatial_node_is_null(child)) {
      spatial_node_data2_t *cd;
      spatial_node_t next;
      if (!spatial_node2_valid(space, child)) break;
      cd   = &space->nodes[child.index];
      next = cd->next_sibling;
      spatial__traverse2(space, child, changed);
      child = next;
    }
  }
}

SPATIAL_EXPORT
void
spatial_update2(spatial_space2_t * __restrict space) {
  uint32_t i;
  if (!space) return;

  for (i = 0; i < space->dirty_count; i++) {
    spatial__traverse2(space, space->dirty_roots[i], false);
  }

  space->dirty_count = 0;
  space->update_version++;
}
