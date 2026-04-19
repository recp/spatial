# parallel

`spatial_update` may run in parallel across independent dirty subtrees,
and writer threads may mark nodes dirty concurrently. This document is
normative: implementations that claim parallel support must follow it.

## Mental model

```
Phase 1  Input              — single thread
Phase 2  Sim / Animation    — multiple writer threads, DISJOINT nodes
Phase 3  Barrier            — joins all writer threads
Phase 4  spatial_update     — may dispatch readers across threads
Phase 5  Render             — multiple reader threads, READ-ONLY
```

Two parallel surfaces:

1. **Readers in Phase 5** — render, culling, command recording, GPU
   command buffer workers. Lock-free by construction; no API needed
   beyond the zero-copy accessors. This is the common case and
   works automatically.
2. **Writers in Phase 2** — physics / animation jobs that set poses on
   disjoint node ranges. Opt in with `spatial_node_mark_dirty_mt`.

`spatial_update` in Phase 4 is itself either sequential or parallel,
orthogonal to the above.

## Enabling the pool

```c
SPATIAL_EXPORT void spatial_space_enable_parallel(spatial_space_t *space,
                                                  uint32_t         thread_count);
```

- Lazy; must be called once before the first multi-root update
- `thread_count == 0` picks hardware concurrency (at least 2)
- No-op if already enabled
- Pool is torn down by `spatial_space_destroy`

If never called, `spatial_update` stays sequential.

## Dispatch threshold

```c
SPATIAL_EXPORT void spatial_space_set_parallel_threshold(spatial_space_t *space,
                                                         uint32_t         min_dirty_roots);
```

Below the threshold, `spatial_update` runs sequentially even when a
pool is enabled. Dispatch overhead dominates for tiny work.

- Default (implementation-recommended): **128** dirty roots
- Callers may tune down for deep-subtree scenes, up for shallow ones
- Setting `< 2` is clamped to `2`

Measured crossover on flat hierarchies: ~200–500 nodes.
Scenes with 100+ descendants per dirty root cross over much lower.

## Update dispatch algorithm

```
spatial_update(space):
  if dirty_count == 0:
    update_version++; return
  compact_dirty(space)                  # remove descendants of dirty roots
  if pool && dirty_count >= threshold:
    parallel_dispatch()
  else:
    sequential_traverse()
  dirty_count = 0
  update_version++
```

`compact_dirty` removes any dirty_root whose ancestor chain also has a
dirty flag set. This is required so parallel workers never process the
same subtree twice. It runs **before** dispatch.

## Parallel worker contract

Each worker owns a private traversal stack. Workers pull a **chunk** of
dirty-root indices per mutex lock to amortize contention. Within a
chunk, workers traverse each subtree single-threadedly using the
iterative algorithm from [update.md](update.md).

Workers never touch nodes outside their assigned subtree. This is safe
because `compact_dirty` guarantees dirty roots are disjoint.

## Reservation

Before MT writes begin, the caller **MUST** reserve the dirty-roots
ring:

```c
SPATIAL_EXPORT void spatial_space_reserve_dirty(spatial_space_t *space,
                                                uint32_t         n);
```

Sizing: worst-case concurrent pushes. For a physics solver that may
mark every body, pass the body count.

Failure to reserve enough is **undefined behavior** (the current
implementation aborts).

## Thread-safe dirty mark

```c
SPATIAL_EXPORT void spatial_node_mark_dirty_mt(spatial_space_t *space,
                                               spatial_node_t   handle,
                                               uint32_t         dirty_flag);
```

Thread-safe. Atomic fetch-or on the dirty flag; the thread whose
fetch-or transitions the flag from 0 to set is the unique publisher
of the ring entry (atomic fetch-add). Concurrent threads marking the
same node only OR in additional dirty bits and skip the append, so
the ring contains at most one entry per node. Ancestor dedupe is
handled by `compact_dirty` at dispatch time.

Typical use:

```c
/* physics job N */
for (body in my_range):
  space->world_positions[body.idx][0] = x
  space->world_positions[body.idx][1] = y
  space->world_positions[body.idx][2] = z
  space->flags[body.idx] |= SPATIAL_NODE_PHYSICS_OWNS  /* OR'd safely; only this thread owns body */
  spatial_node_mark_dirty_mt(space, body.handle, SPATIAL_NODE_DIRTY_WORLD)
```

Pose writes are safe because each thread owns a disjoint index range;
no cross-thread node is touched.

Setting non-dirty flags (`PHYSICS_OWNS`, `HAS_MATRIX`) from multiple
threads is **safe** only if the threads OR'ing them agree — or if each
thread OR'ing a given node is the sole owner of that node.

## Read side (Phase 5)

The lock-free read surface is what most multi-threaded engines
actually need. Render, culling, GPU command recording, and SSBO
upload workers can all run in parallel reading the same spatial
arrays.

Readers during Phase 5 do not need any special API. Every zero-copy
accessor is `const`-returning and plain memory access — no locks, no
atomics, no fences.

```c
/* Vulkan command-recording worker, one of many */
const mat4 *m = spatial_node_world_matrix(space, handles[i]);
vkCmdPushConstants(cmd, layout, stage, 0, sizeof(mat4), m);
```

`spatial_update` must have completed before readers start. Once it
returns, mutex-release barriers make all writes visible to all threads.
No further synchronization is needed between readers themselves.

## Unsupported

- Concurrent writer + reader without a barrier between — **not
  supported**. Use phase discipline or add double-buffering externally.
- Concurrent writers targeting **the same node** — **not supported**;
  the engine must assign each node to a single owner per frame.

## Conformance

An implementation is parallel-conformant iff it:

- treats `spatial_node_mark_dirty_mt` atomically and dedupes at push
  time so each node appears at most once in `dirty_roots`
- runs `compact_dirty` before any dispatch
- guarantees each dirty-root subtree is traversed by exactly one thread
- never invalidates a cached pointer during `spatial_update`
- returns from `spatial_update` only after every worker has finished
  processing the current dispatch (workers may persist and block on a
  condition variable between dispatches — they do not need to be
  `pthread_join`ed per call)
