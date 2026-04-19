#  🔭 spatial - WIP / Experimental

**spatial** is a shared spatial kernel for engines.

It is not a graphics engine, physics engine, animation engine, or audio engine.  
It is the common layer beneath them, defining how "where something is" is represented, updated, and shared.

```
  graphics engine    physics engine    animation engine    audio engine
        |                  |                    |                |
        +------------------+--------------------+----------------+
                                   |
                                spatial   ← shared spatial kernel
                                   |
                                 cglm
```

## What it does

- Scene graph (parent / child / sibling hierarchy)
- Local and world pose (position + rotation + scale), cached world matrix
- Matrix override on individual nodes for non-TRS transforms
  (look-at, skew, CSS-style matrix stacks)
- Dirty propagation and world matrix update traversal
- Common spatial state that multiple systems can read and write

## What it does not do

- Rigid body simulation
- Rendering
- Animation playback, blending, state machines, or IK
- Audio mixing
- Physics constraints or solver

## Why

Every engine eventually re-implements "where is this object and who is its parent."  
When graphics, physics, animation, and audio share one spatial model, integration becomes simpler:

- physics writes world pose → spatial
- animation writes local pose → spatial
- graphics reads world matrix ← spatial  
- audio reads world position ← spatial
- editor authors local transform → spatial

The goal is to reduce duplicate transform code, reduce sync cost, and define one spatial contract.

More specifically, `spatial` exists to standardize:

- a shared pose / transform ABI
- shared pose semantics (`local`, `world`, hierarchy, authority)
- a shared update contract between systems

That matters because the real cost is often not the transform math itself, but repeated conversion, duplicate storage, cache rebuilds, and sync bugs between frameworks.

## Authority

`spatial` does not force one system to own all transforms.  
Different systems can be authoritative at different times:

| content type         | authoritative system     |
|----------------------|--------------------------|
| static world geometry | spatial / editor         |
| animated objects      | animation system         |
| dynamic rigid bodies  | physics engine           |
| render output         | reads from spatial       |
| audio emitter/listener| reads from spatial       |


## Quick Start

```c
#include <spatial/spatial.h>

spatial_space_t *space = spatial_space_create(32);

spatial_pose_t a_local = SPATIAL_POSE_IDENTITY;
spatial_pose_t b_local = SPATIAL_POSE_IDENTITY;
spatial_pose_t b_world;

a_local.position[0] = 1.0f;
b_local.position[0] = 2.0f;

spatial_node_t a = spatial_node_create(space, SPATIAL_NODE_NULL, &a_local);
spatial_node_t b = spatial_node_create(space, a, &b_local);

spatial_update(space);
spatial_node_get_world(space, b, &b_world);

/* b_world.position[0] == 3.0f */

spatial_space_destroy(space);
```

## Zero-overhead hot paths

`spatial` is designed so that render and physics hot loops pay nothing
beyond a single memory load. Inline pointer accessors return directly
into the SoA arrays — no memcpy, no function-call overhead after
inlining.

### Render: Vulkan push-constants with zero copy

```c
/* After spatial_update, read the live world matrix pointer. */
spatial_space_reserve(space, MAX_DRAWABLES);       /* pin pointers once */

for (uint32_t i = 0; i < draw_count; i++) {
    const mat4 *m = spatial_node_world_matrix(space, draws[i].handle);
    vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_VERTEX_BIT,
                       0, sizeof(mat4), m);  /* pointer passed straight */
    vkCmdDrawIndexed(cmd, draws[i].index_count, 1, 0, 0, 0);
}
```

Measured on an M1 Max: `3.5 ns / node` for the pointer accessor — same
as hand-written `&space->world_matrices[h.index]`. The safe copy variant
(`spatial_node_get_world_matrix`) runs at `12.6 ns` due to the 64-byte
memcpy and handle validation.

### Physics: solver writes straight into SoA

```c
/* Physics hot loop. One cache line touched per body. */
for (uint32_t i = 0; i < body_count; i++) {
    uint32_t idx = bodies[i].handle.index;
    space->world_positions[idx][0] = bodies[i].x;
    space->world_positions[idx][1] = bodies[i].y;
    space->world_positions[idx][2] = bodies[i].z;
    glm_quat_copy(bodies[i].q, space->world_rotations[idx]);
    space->flags[idx] |= SPATIAL_NODE_DIRTY_WORLD | SPATIAL_NODE_PHYSICS_OWNS;
}
```

Measured: `2.9 ns / write` vs `21.3 ns` for the safe setter — 7× faster.
After the solver step, one `spatial_update(space)` propagates to
descendants (render transforms, attached effects).

### Parallel jobs writing disjoint node ranges

```c
spatial_space_reserve_dirty(space, MAX_BODIES);    /* size the ring once */

/* Job N, running on worker thread N: */
for (uint32_t i = start; i < end; i++) {
    uint32_t idx = bodies[i].handle.index;
    space->world_positions[idx][0] = new_x;        /* disjoint → no race */
    space->flags[idx] |= SPATIAL_NODE_PHYSICS_OWNS;
    spatial_node_mark_dirty_mt(space, bodies[i].handle, SPATIAL_NODE_DIRTY_WORLD);
}

/* Main thread after job barrier: */
spatial_update(space);   /* sequential or parallel traversal of dirty subtrees */
```

`spatial_node_mark_dirty_mt` uses atomic fetch-or + fetch-add against
the dirty-roots ring, so any number of writer threads can mark nodes
dirty without a lock. The existing `spatial_update` compaction pass
drops duplicates, so callers never need to check ancestor state in the
MT path.

### Accessor cheat sheet

| use case                     | API                                                   | cost          |
|------------------------------|-------------------------------------------------------|---------------|
| Render read (hot)            | `spatial_node_world_matrix(s, h)` → `const mat4 *`    | 3.5 ns, inline|
| Render read (safe, cold)     | `spatial_node_get_world_matrix(s, h, out)`            | 12.6 ns, copy |
| Physics write (hot)          | `s->world_positions[idx][k] = v;`                     | 2.9 ns        |
| Physics write (safe)         | `spatial_node_set_world_physics(s, h, &pose)`         | 21.3 ns, copy |
| Physics MT dirty (lock-free) | `spatial_node_mark_dirty_mt(s, h, flag)`              | atomic        |
| Pin pointers for a frame     | `spatial_space_reserve(s, N)`                         | one-time      |

## Dependencies

- [cglm](https://github.com/recp/cglm)

## Ecosystem

`spatial` is designed to be the shared spatial substrate for this set
of projects. Each one plugs into it through the roles described above:

| project                                              | role                    | how it uses spatial                          |
|------------------------------------------------------|-------------------------|----------------------------------------------|
| [cglm](https://github.com/recp/cglm)                 | math                    | dependency — vectors, quats, matrices        |
| [AssetKit](https://github.com/recp/AssetKit)         | asset I/O (glTF/COLLADA)| imports node trees into `spatial_space_t`    |
| [anim](https://github.com/recp/anim)                 | animation               | writes bone `local` poses                    |
| [phy](https://github.com/recp/phy)                   | physics                 | writes rigid-body `world` poses (`PHYSICS_OWNS`) |
| [gpu](https://github.com/recp/gpu)                   | pure GPU abstraction    | underlying layer for the render engine       |
| [rend](https://github.com/recp/rend)                 | render engine           | reads `world_matrix` for draw submissions    |
| [rays](https://github.com/recp/rays)                 | ray tracing             | reads `world_matrix` for acceleration structures |
| [reality](https://github.com/recp/reality)           | AR                      | reads camera / anchor poses                  |
| [UniversalShading](https://github.com/UniversalShading) | shading language     | consumes world transforms via `rend` shader bindings |

Integrations are in progress; API changes in `spatial` are expected
as real use exposes rough edges.

## Status

Early development. API is not stable.
