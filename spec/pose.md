# pose

## spatial_transform_t — canonical physics-compatible pose

```c
typedef struct spatial_transform_t {
  vec3   position;   /* 12 bytes */
  versor rotation;   /* 16 bytes, unit quaternion */
} spatial_transform_t;  /* 28 bytes, 16-byte aligned (implementation pads) */
```

### Invariants

- `rotation` **MUST** be a unit quaternion (||q|| == 1 ± epsilon).
- `position` **MUST NOT** contain NaN or infinity.
- The struct layout **MUST** place `position` before `rotation`.
- The struct **SHOULD** be 16-byte aligned.

### Rationale

Matches `PxTransform` (PhysX), Jolt `Body` pose, Rapier `Isometry3`,
and Havok runtime bodies. Physics engines can alias this type onto
their own with zero conversion.

### Non-goals

- Scale is not part of this type. Scale belongs on `spatial_pose_t` or
  on shape-level data.
- Matrix form is not stored. Callers derive mat4 via
  `spatial_transform_to_mat4()`.

## spatial_pose_t — scaled pose for graphics and editor

```c
typedef struct spatial_pose_t {
  vec3   position;
  versor rotation;
  vec3   scale;
} spatial_pose_t;  /* 40 bytes */
```

### Invariants

- Same invariants as `spatial_transform_t` for position and rotation.
- `scale` components **SHOULD** be positive and nonzero on nodes that
  physics or collision systems will read.
- Non-uniform scale **MAY** be used on graphics-only nodes.

### Composition

Pose composition for `world = parent.world ∘ local`:

```
world.position = parent.position
               + parent.rotation * (parent.scale * local.position)
world.rotation = parent.rotation * local.rotation
world.scale    = parent.scale * local.scale
```

Non-uniform scale on a parent **MAY** produce non-orthogonal world
transforms for children. Implementations **SHOULD** detect this and
warn when a physics-authoritative node is descended from a
non-uniformly scaled parent.

## spatial_dtransform_t — large-world variant

```c
#ifdef SPATIAL_DOUBLE_PRECISION
typedef struct spatial_dtransform_t {
  double position[3];
  versor rotation;
} spatial_dtransform_t;
#endif
```

Matches Jolt `RVec3` and Rapier double-precision mode. Position is
double, rotation stays float. Only compiled when
`SPATIAL_DOUBLE_PRECISION` is defined.

## Conversions

Implementations **MUST** provide:

```c
void spatial_transform_to_mat4(const spatial_transform_t *t, mat4 out);
void spatial_pose_to_mat4     (const spatial_pose_t      *p, mat4 out);
void spatial_mat4_to_transform(const mat4 m, spatial_transform_t *out);
void spatial_mat4_to_pose     (const mat4 m, spatial_pose_t *out);
```

`mat4_to_*` functions may lose information if the input matrix
contains shear or non-uniform components that do not decompose
cleanly. Behavior in that case is implementation-defined but
**MUST NOT** crash.

## Identity

```c
#define SPATIAL_TRANSFORM_IDENTITY ((spatial_transform_t){ \
    .position = {0, 0, 0},                           \
    .rotation = {0, 0, 0, 1}                         \
})

#define SPATIAL_POSE_IDENTITY ((spatial_pose_t){           \
    .position = {0, 0, 0},                           \
    .rotation = {0, 0, 0, 1},                        \
    .scale    = {1, 1, 1}                            \
})
```

## Internal storage note

The public types above describe the ABI for `spatial_pose_t` and
`spatial_transform_t`. Internally a conformant implementation **MAY**
store positions and scales in wider cells (e.g. `vec4` / 16-byte
aligned) to enable aligned SIMD loads. The w-lane of such storage is
ignored semantically; accessors copy or expose only the 3-lane value.

Implementations **SHOULD** expose zero-copy read accessors that return
`const` pointers directly into storage for hot paths:

```c
const mat4   *spatial_node_world_matrix  (const spatial_space_t*, spatial_node_t);
const vec4   *spatial_node_world_position(const spatial_space_t*, spatial_node_t);
const versor *spatial_node_world_rotation(const spatial_space_t*, spatial_node_t);
const vec4   *spatial_node_world_scale   (const spatial_space_t*, spatial_node_t);
/* matching spatial_node_local_*                                                  */
```

These accessors **MUST NOT** validate handles (caller's responsibility)
and **MUST** be inline. The safe copy variants
(`spatial_node_get_world`, `spatial_node_get_world_matrix`) continue to
validate and are the cold-path API.
