# update

`spatial_update()` is the single function that makes spatial useful. It
resolves dirty state into consistent world poses and cached matrices.

## Signature

```c
void spatial_update(spatial_space_t *space);
```

## Preconditions

- `space` is non-null and initialized.
- All handles referenced by nodes' `parent`, `first_child`,
  `next_sibling` are either `SPATIAL_NODE_NULL` or valid.
- No other thread mutates the space during this call (unless the
  implementation documents otherwise).

## Postconditions

- For every node `n` with `SPATIAL_NODE_ALIVE`:
  - `n.world` equals `parent.world ∘ n.local` (or equals the
    externally written `world` if `PHYSICS_OWNS`).
  - `n.world_matrix` equals `spatial_pose_to_mat4(n.world)`.
  - `SPATIAL_NODE_DIRTY_LOCAL` and `SPATIAL_NODE_DIRTY_WORLD` are clear.
  - `n.version` has incremented iff `n.world` changed this call.
- `space->update_version` has incremented by 1.
- `space->dirty_roots` is empty and `dirty_count == 0`.

## Algorithm

The pseudocode below is recursive for readability; conformant
implementations **SHOULD** use an iterative stack-based traversal to
avoid stack overflow on deep hierarchies.

A post-mutation compaction pass **MUST** run before dispatch to remove
any dirty_root whose ancestor chain also has a dirty flag set. Without
this step, parallel workers could process the same subtree twice
(e.g., after an `attach` reparented a dirty node under another dirty
subtree).

```
1. compact_dirty(space)
     for i in dirty_roots:
       if any ancestor of i has DIRTY_LOCAL or DIRTY_WORLD: drop i
2. for each root in space->dirty_roots:
     traverse(root)

3. clear space->dirty_roots
4. space->update_version += 1

traverse(node):
  data = space->nodes[node.index]

  if data.flags & HAS_EXPR:
    evaluate expression chain → data.local
  if data.flags & HAS_MATRIX:
    data.world_matrix = matrix override
    goto children

  if data.flags & PHYSICS_OWNS:
    # world was written externally; do not recompute from local
    pose = data.world
  else if data.parent == NULL:
    pose = data.local
  else:
    parent_data = space->nodes[data.parent.index]
    pose = compose(parent_data.world, data.local)

  if pose != data.world:
    data.world = pose
    data.version += 1

  pose_to_mat4(data.world, data.world_matrix)

  clear DIRTY_LOCAL, DIRTY_WORLD

  for child in children(node):
    traverse(child)
```

## Dirty propagation

When a caller modifies `local` on node `n`:

1. Set `n.flags |= DIRTY_LOCAL`.
2. Add the first dirty ancestor (or `n` itself if no ancestor is
   dirty) to `space->dirty_roots`.

When physics writes `world` on node `n` (with `PHYSICS_OWNS`):

1. Set `n.flags |= DIRTY_WORLD`.
2. Add `n` to `space->dirty_roots`.

Implementations **SHOULD** dedupe `dirty_roots` to avoid traversing
the same subtree twice.

## Cached matrix

`world_matrix` is derived. An implementation **MAY** skip updating
it for nodes whose descendants do not need a matrix (e.g. audio
emitters). In that case the implementation **MUST** document which
nodes have live `world_matrix` and which do not.

A conformant default: update `world_matrix` for every visited node.

## Version counter

`node.version` allows cheap change detection:

```c
if (node.version != cached_version) {
    rebuild_render_data(node);
    cached_version = node.version;
}
```

`space->update_version` allows frame-level change detection
(e.g. "did anything change this frame?").

## Traversal order

Implementations **MUST** visit parents before children. Sibling
order is implementation-defined. Consistent order is **RECOMMENDED**
for deterministic builds.

## Performance

- Cost is proportional to the number of dirty nodes and their
  descendants, not the total node count.
- Implementations **SHOULD** avoid recursing into subtrees with no
  dirty flags.
- Implementations **MAY** parallelize traversal across independent
  dirty roots. See [parallel.md](parallel.md) for the dispatch model,
  worker contract, and threshold semantics.
- Push-to-dirty-roots **MUST** dedupe at push time: if the node or any
  ancestor already has a dirty flag, skip pushing a new ring entry and
  rely on the ancestor's traversal to cover the descendant. This keeps
  the ring O(unique-subtrees) rather than O(writes).

## Non-goals

- `spatial_update()` does not simulate physics.
- `spatial_update()` does not render.
- `spatial_update()` does not evaluate animation curves beyond the
  expression chain (animation systems run before update and write
  `local`).
