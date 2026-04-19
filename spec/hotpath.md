# hot-path access

Render loops, culling jobs, and physics solvers read and write
transforms tens of thousands of times per frame. This document
describes zero-copy access patterns that keep spatial out of the
critical path.

## Zero-copy reads — render, culling, command recording

The inline pointer accessors return `const`-qualified pointers
directly into the SoA arrays. No memcpy, no function-call overhead
after inlining. Pass the pointer straight to graphics / GPU APIs.

```c
/* Vulkan command recording, no intermediate buffer. */
for (uint32_t i = first; i < last; i++) {
    const mat4 *m = spatial_node_world_matrix(space, handles[i]);
    vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_VERTEX_BIT,
                       0, sizeof(mat4), m);
    vkCmdDrawIndexed(cmd, draws[i].index_count, 1, 0, 0, 0);
}
```

```c
/* Culling — read position + radius, no copies. */
const vec4 *pos = spatial_node_world_position(space, handle);
float dist2 = (*pos)[0]*(*pos)[0] + (*pos)[1]*(*pos)[1] + (*pos)[2]*(*pos)[2];
if (dist2 > cull_radius * cull_radius) skip(handle);
```

### Accessor cheat sheet

| function                              | returns              | use                               |
|---------------------------------------|----------------------|-----------------------------------|
| `spatial_node_world_matrix(s, h)`     | `const mat4 *`       | vertex transforms, skin bind     |
| `spatial_node_world_position(s, h)`   | `const vec4 *`       | culling, AABB center, audio       |
| `spatial_node_world_rotation(s, h)`   | `const versor *`     | directional audio, oriented tests |
| `spatial_node_world_scale(s, h)`      | `const vec4 *`       | bounding volume scaling           |
| `spatial_node_local_*`                | same                 | animation read-back, editors      |

Pointers remain valid until the next `spatial_node_create` that grows
the arrays, or until `spatial_space_destroy`. Call
`spatial_space_reserve` upfront to pin pointers for the lifetime of
the space.

These accessors perform **no handle validation** — caller must ensure
the handle is live. For cold paths or untrusted handles, use the copy
variants (`spatial_node_get_world_matrix`, …) which validate and
return `false` on bogus handles.

## Pre-reserving capacity

Growth is the only thing that can move SoA storage. Call
`spatial_space_reserve` to a known upper bound before the frame
loop and cached pointers stay valid for the lifetime of the space:

```c
spatial_space_t *space = spatial_space_create(1024);
spatial_space_reserve(space, MAX_SCENE_NODES);  /* pin */
/* build scene, cache handles and pointers, render … */
```

A future `spatial_space_set_growable(space, false)` could make growth
an assert if that ever becomes desirable; the current implementation
trusts the caller.

## Zero-copy writes — physics, animation, direct authority

The reference implementation stores nodes as parallel SoA arrays
(`space->world_positions`, `space->flags`, …) and exposes these as
the hot-path write surface. Alternative implementations **MAY**
choose a different layout so long as the same zero-copy semantics are
offered through some mechanism; the examples below target the
reference layout.

Write paths skip the safe setters by mutating SoA slots directly and
setting the dirty flag. One cache line touched per write.

```c
/* Physics solver hot loop. */
for (uint32_t i = 0; i < body_count; i++) {
    uint32_t idx = bodies[i].handle.index;
    space->world_positions[idx][0] = bodies[i].x;
    space->world_positions[idx][1] = bodies[i].y;
    space->world_positions[idx][2] = bodies[i].z;
    glm_quat_copy(bodies[i].q, space->world_rotations[idx]);
    space->flags[idx] |= SPATIAL_NODE_DIRTY_WORLD | SPATIAL_NODE_PHYSICS_OWNS;
}
/* ... one spatial_update at end of frame propagates to descendants. */
```

```c
/* Animation bone writer. */
for (uint32_t i = 0; i < bone_count; i++) {
    uint32_t idx = bones[i].handle.index;
    glm_vec3_copy(bones[i].pos, space->local_positions[idx]);
    space->local_positions[idx][3] = 0.0f;
    glm_quat_copy(bones[i].rot, space->local_rotations[idx]);
    glm_vec3_copy(bones[i].scl, space->local_scales[idx]);
    space->local_scales[idx][3] = 0.0f;
    space->flags[idx] |= SPATIAL_NODE_DIRTY_LOCAL;
}
```

For each animation and physics frame, the caller also appends the
mutated handles to `space->dirty_roots` or calls
`spatial_node_mark_dirty_mt` (multi-threaded, see [parallel.md](parallel.md)).

## Correctness rules

Direct SoA access is unsafe across structural changes. The contract:

1. **Do not cache pointers across `spatial_node_create` calls unless
   capacity is reserved.** Growth reallocates; cached pointers are
   dangling.
2. **Do not cache pointers across `spatial_node_destroy`.** The memory
   is still valid but the slot may have been recycled into a different
   node with a new generation.
3. **Do not share cached pointers across threads without external
   synchronization.** spatial is not lock-free on mutating calls
   unless documented otherwise; see [parallel.md](parallel.md).

## Thread safety summary

| phase / path                                    | safety                                  |
|-------------------------------------------------|-----------------------------------------|
| Multiple threads **read** world poses / matrices | lock-free, always safe                  |
| Multiple threads **write** disjoint nodes        | safe; use `spatial_node_mark_dirty_mt`  |
| Multiple threads **write** same node             | unsafe; engine must serialize ownership |
| Concurrent **read** and **write** of same node   | unsafe; separate with phase barriers    |

## Summary

Read once per frame via handle — from then on, access SoA arrays
directly. Reserve upfront to pin pointers. For writes, skip the safe
setters on the hot path; set the flag and let `spatial_update` do the
propagation.

For multi-threaded writers (physics / animation job systems), see
[parallel.md](parallel.md) — `spatial_node_mark_dirty_mt` is the
lock-free per-node dirty push backed by atomic fetch-or / fetch-add.
