# hot-path access

Physics solvers iterate bodies thousands of times per simulation step.
Per-iteration handle validation and slot indexing adds unnecessary
overhead. This document describes the recommended pattern for caching
direct pointers into spatial node storage during hot loops.

## The slow path

```c
for (int iter = 0; iter < N; iter++) {
  spatial_node_data_t *n = spatial_node_get(space, body->node);
  if (!n) continue;
  /* read / write n->world */
}
```

Each iteration does:
- handle validity check (2 comparisons + array read)
- array index (pointer arithmetic)
- null-check on the returned pointer

For 10k bodies × 20 solver iterations × 60 Hz, that is 12M redundant
validations per second. Avoidable.

## The fast path

```c
/* once, before the solver loop */
spatial_node_data_t *body_data[N];
for (int i = 0; i < N; i++) {
  body_data[i] = spatial_node_get(space, bodies[i].node);  /* may be NULL */
}

/* hot loop */
for (int iter = 0; iter < N; iter++) {
  spatial_node_data_t *n = body_data[iter];
  if (!n) continue;
  /* direct access */
  n->world.position[0] += velocity[iter][0] * dt;
}
```

Handle validation happens once per body per frame, not per solver
iteration.

## Correctness requirements

A cached `spatial_node_data_t *` is valid only while:

1. **No node is destroyed** in the space. `spatial_node_destroy()`
   marks the slot free and bumps its generation. The pointer still
   points to valid memory, but the slot may be reused.
2. **No node is created beyond capacity**. `spatial_node_create()`
   may `realloc()` the underlying array, invalidating all cached
   pointers.
3. **The space is not destroyed.**

If any of the above may occur during the hot loop, either refresh the
cache or pre-reserve capacity.

## Pre-reserving capacity

To guarantee no `realloc` during the simulation step, create the space
with sufficient capacity upfront:

```c
spatial_space_t *space = spatial_space_create(expected_max_nodes);
```

The space grows by doubling; starting at a known upper bound avoids
mid-frame reallocation.

A future `spatial_space_reserve(space, cap)` function would make this
explicit; for now, pass the target capacity at creation time.

## Matching the solver frame boundary

A safe pattern is to refresh pointer caches at the start of each
simulation step, after any structural changes (creates / destroys)
from game code have been applied:

```c
frame_begin(space);          /* game code creates / destroys nodes */
spatial_update(space);       /* propagate from game logic */

refresh_body_pointer_cache(); /* cache fresh pointers */

for (substep = 0; substep < N; substep++) {
  physics_integrate(body_data);   /* fast path */
  physics_solve_constraints();    /* fast path */
}

/* write back dirty flags so spatial_update will propagate children */
for (i = 0; i < count; i++) {
  body_data[i]->flags |= SPATIAL_NODE_DIRTY_WORLD | SPATIAL_NODE_PHYSICS_OWNS;
}
spatial_update(space);
```

## Direct dirty-flag writes

Inside the hot loop, setting `SPATIAL_NODE_DIRTY_WORLD` directly on
the cached pointer is safe. The flag will be picked up by the next
`spatial_update()`. Bypassing `spatial_node_set_world_physics()`
avoids a handle lookup and a second dirty-list push per write.

If the node is not already in `space->dirty_roots`, push it once at
the end of the substep loop via the public API, or append the handle
directly (implementation-dependent).

## Sibling / child traversal in hot paths

Traversing children via cached pointers is also fine, but each hop
costs a lookup because `n->first_child` is a `spatial_node_t` handle,
not a pointer. If a physics engine needs a flat list of bodies under a
subtree, precompute that list once per frame rather than walking the
hierarchy on every iteration.

## What not to do

- Do not cache pointers across `spatial_node_create()` calls unless
  capacity is reserved.
- Do not cache pointers across `spatial_node_destroy()` calls. The
  memory is still valid, but the slot may have been recycled into an
  unrelated node with a new generation.
- Do not share cached pointers across threads without external
  synchronization (spatial is not thread-safe by default).
- Do not skip the initial validation. A `NULL` from `spatial_node_get`
  must be respected on first cache; after that, cached pointers can be
  trusted for the scope defined above.

## Summary

Validate once per frame, access directly thereafter. Reserve capacity
to prevent `realloc`. Never cache across node destroys. This pattern
lets spatial underlie a physics solver with effectively zero overhead
versus a dedicated physics transform store.

For multi-threaded writers (physics / animation job systems), see
[parallel.md](parallel.md) and `spatial_node_mark_dirty_mt` —
lock-free per-node dirty push backed by atomic fetch-or / fetch-add.

## Zero-copy reads for graphics

For render loops, command recording, and culling jobs, use the
inline pointer accessors. They return `const`-qualified pointers
directly into the SoA arrays — no memcpy, no function-call overhead
after inlining.

```c
/* Vulkan command recording, no copies. */
for (uint32_t i = first; i < last; i++) {
    const mat4 *m = spatial_node_world_matrix(space, handles[i]);
    vkCmdPushConstants(cmd, layout, stage, 0, sizeof(mat4), m);
}
```

```c
/* Culling — read position + bounds, no copies. */
const vec4 *pos = spatial_node_world_position(space, handle);
float dist2 = (*pos)[0]*(*pos)[0] + (*pos)[1]*(*pos)[1] + (*pos)[2]*(*pos)[2];
```

Available zero-copy accessors:

| function                               | returns              |
|----------------------------------------|----------------------|
| `spatial_node_world_matrix(s, h)`      | `const mat4 *`       |
| `spatial_node_world_position(s, h)`    | `const vec4 *`       |
| `spatial_node_world_rotation(s, h)`    | `const versor *`     |
| `spatial_node_world_scale(s, h)`       | `const vec4 *`       |
| `spatial_node_local_position(s, h)`    | `const vec4 *`       |
| `spatial_node_local_rotation(s, h)`    | `const versor *`     |
| `spatial_node_local_scale(s, h)`       | `const vec4 *`       |

Pointers remain valid until the next `spatial_node_create` that
grows the arrays, or until `spatial_space_destroy`. Call
`spatial_space_reserve` upfront to pin pointers for the lifetime of
the space.

These accessors perform **no handle validation** — caller must
ensure the handle is live. For cold paths or untrusted handles, use
the copy variants (`spatial_node_get_world_matrix`, etc.) which
validate and return `false` on bogus handles.

For **writes** in physics hot loops, write directly to the SoA arrays:

```c
space->world_positions[idx][0] = x;
space->world_positions[idx][1] = y;
space->world_positions[idx][2] = z;
space->flags[idx] |= SPATIAL_NODE_DIRTY_WORLD | SPATIAL_NODE_PHYSICS_OWNS;
```

The safe copy setters (`spatial_node_set_world_physics`) are
provided for cold-path use.
