# authority

Spatial does not force one system to own all transforms. Different
systems write at different times. The flags on a node declare who is
authoritative for that node during a given frame.

## Flags

```c
enum {
  SPATIAL_NODE_ALIVE        = 1 << 0,
  SPATIAL_NODE_DIRTY_LOCAL  = 1 << 1,
  SPATIAL_NODE_DIRTY_WORLD  = 1 << 2,

  SPATIAL_NODE_HAS_EXPR     = 1 << 3,
  SPATIAL_NODE_HAS_MATRIX   = 1 << 4,

  SPATIAL_NODE_PHYSICS_OWNS = 1 << 5,
  SPATIAL_NODE_KINEMATIC    = 1 << 6,
  SPATIAL_NODE_STATIC       = 1 << 7,
};
```

## Write rules

A system may write to a node's `local` pose iff:

- `SPATIAL_NODE_PHYSICS_OWNS` is clear, and
- the system holds the conventional authority for the node, as
  documented below, and
- it sets `SPATIAL_NODE_DIRTY_LOCAL` after the write.

A system may write to a node's `world` pose iff:

- `SPATIAL_NODE_PHYSICS_OWNS` is set (physics engine only), and
- it sets `SPATIAL_NODE_DIRTY_WORLD` after the write, so children
  re-propagate on the next update.

Writing `world` directly without `SPATIAL_NODE_PHYSICS_OWNS` set is
**implementation-defined** and **SHOULD** be treated as an error.

## Read rules

Any system **MAY** read `world`, `local`, and `world_matrix` at any
time. Reads during `spatial_update()` **MAY** return torn or pre-update
values if the space is being mutated concurrently; external
synchronization is the caller's responsibility unless the
implementation documents lock-free reads.

## Conventional authority

| content type           | authoritative system | flag combo                           |
|------------------------|----------------------|--------------------------------------|
| static world geometry  | editor               | `STATIC`                             |
| animated objects       | animation system     | (none; writes local)                 |
| dynamic rigid bodies   | physics engine       | `PHYSICS_OWNS`                       |
| kinematic bodies       | game code            | `PHYSICS_OWNS | KINEMATIC`           |
| camera / audio listener| game / animation     | (none; reads world)                  |

## Physics integration

When `SPATIAL_NODE_PHYSICS_OWNS` is set:

1. Physics engine steps the simulation.
2. For each active body, physics writes `world.position` and
   `world.rotation` into the node.
3. Physics sets `SPATIAL_NODE_DIRTY_WORLD`.
4. Caller invokes `spatial_update(space)`.
5. `spatial_update()` propagates the new world to children and
   refreshes `world_matrix`.

Physics **SHOULD NOT** read or write the expression chain on
physics-owned nodes. Physics **SHOULD** treat `local` as derived
when it owns the node.

## Graphics integration

Graphics **MUST NOT** write to `world` unless it holds authority
(e.g. root camera rig). Graphics reads `world_matrix` after
`spatial_update()` has completed for the frame.

## Audio integration

Audio reads `world.position` and optionally `world.rotation` (for
directional emitters). Audio **MUST NOT** write spatial state.

## Ownership transfer

To hand ownership from one system to another (e.g. a body becoming
dynamic after being kinematic):

1. Ensure a full `spatial_update()` has completed.
2. Clear or set `SPATIAL_NODE_PHYSICS_OWNS` as appropriate.
3. Optionally reset velocities in the physics engine separately.

Ownership transfer during a frame **SHOULD** be avoided.

## Multi-threaded writer phase

When physics or animation runs as a job system, multiple worker
threads can write disjoint node ranges concurrently. The write path:

1. Each worker assigned a disjoint set of node indices.
2. Worker writes directly to SoA arrays for its own indices
   (pose components, non-dirty flags).
3. Worker calls `spatial_node_mark_dirty_mt(space, handle, dirty_flag)`
   to atomically OR the dirty flag and append to the pre-reserved
   dirty-roots ring.
4. Main thread joins all workers.
5. Main thread calls `spatial_update(space)`.

The caller **MUST** call `spatial_space_reserve_dirty(space, N)` before
the parallel phase, sizing `N` to the worst-case concurrent push count.

Threads **MUST NOT** target the same node from more than one writer.
If multiple systems legitimately need to write the same node
(e.g. partial-ragdoll blending), they **MUST** run serially in
distinct phases, not concurrently.

See [parallel.md](parallel.md) for the full dispatch model.
