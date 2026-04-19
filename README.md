#  🔭 spatial

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
- Local and world transform, cached as mat4
- Transform item chains: translate, rotate, scale, quaternion, look-at, skew, matrix
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

No per-integration sync adapters. One spatial contract.

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

## Dependencies

- [cglm](https://github.com/recp/cglm)

## Status

Early development. API is not stable.
