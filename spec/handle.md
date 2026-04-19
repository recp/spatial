# handle

## spatial_node_t

```c
typedef struct spatial_node_t {
  uint32_t index;       /* slot in space->nodes   */
  uint32_t generation;  /* incremented on free    */
} spatial_node_t;
```

Generational handle. 8 bytes. Used as a stable reference to a node
across insertions, deletions, and storage reallocations.

## Null handle

```c
#define SPATIAL_NODE_NULL ((spatial_node_t){ .index = 0, .generation = 0 })
```

Slot 0 is reserved as the null sentinel. No live node occupies it.

## Validation

A handle is valid iff:

- `index` is less than `space->capacity`
- `space->generations[index] == handle.generation`
- `space->nodes[index].flags & SPATIAL_NODE_ALIVE` is set

Implementations **MUST** provide:

```c
bool spatial_node_valid(const spatial_space_t *space, spatial_node_t handle);
```

All public functions that accept a handle **MUST** validate it and
return an error (or a defined no-op) on invalid input. Passing an
invalid handle **MUST NOT** cause undefined behavior.

## Generation semantics

- On `spatial_node_create`, the space allocates a slot (from free list or
  by growing), sets `ALIVE`, and returns `{ index, generations[index] }`.
- On `spatial_node_destroy`, the space clears `ALIVE`, increments
  `generations[index]`, and links the slot into the free list.
- After destroy, any existing handle to that slot fails validation
  because its `generation` no longer matches.

Generation **MUST** wrap on overflow. Wrap-around is rare in practice
(2^32 reuses of one slot) but implementations **MUST NOT** treat
wrap-around as undefined behavior.

## Thread safety

Handles themselves are plain data and safe to copy across threads.
Concurrent access to `spatial_space_t` is implementation-defined and
**SHOULD** be documented per implementation.

A minimum conformant implementation **MAY** require external
synchronization for all mutating calls. A higher-tier implementation
**MAY** offer lock-free read of pose data while the update runs.

## Stability guarantees

- A handle returned from `spatial_node_create` **MUST** remain valid
  until `spatial_node_destroy` is called on that same handle.
- Node storage reallocation **MUST NOT** invalidate handles.
  (Handles reference `index`, not a pointer.)
- Across `spatial_update()` calls, handle values **MUST NOT** change.

## Why handles and not pointers

- Storage can grow without invalidating references.
- Safe to share across systems (physics, graphics, audio, networking)
  without ownership coupling.
- Detectable use-after-free via generation mismatch.
- Matches the modern consensus (Jolt `BodyID`, Rapier
  `RigidBodyHandle`, Box2D v3 `b2BodyId`).

Pointer-based parent/child/sibling links inside node data are **not
exposed** to callers. Internally an implementation **MAY** use
pointers if the node array is not reallocated, but the public API
**MUST** use `spatial_node_t`.
