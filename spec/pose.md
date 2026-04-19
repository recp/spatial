# pose

## spatial_pose_t — primary pose type

This is the pose type most systems use: scene graphs, editors,
animation, rendering. Carries translation, rotation, and scale.

```c
typedef struct spatial_pose_t {
  vec3   position;
  versor rotation;
  vec3   scale;
} spatial_pose_t;  /* 40 bytes */
```

### Invariants

- `rotation` **MUST** be a unit quaternion (||q|| == 1 ± epsilon).
- `position` and `scale` **MUST NOT** contain NaN or infinity.
- `scale` components **SHOULD** be nonzero on any node whose world
  transform will be read for rendering or physics.
- Non-uniform scale **MAY** be used on graphics-only nodes but cascades
  into children's world transforms (see below).

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
warn when a node with a physics owner is descended from a
non-uniformly scaled parent.

### Identity

```c
#define SPATIAL_POSE_IDENTITY ((spatial_pose_t){ \
    .position = {0, 0, 0},                       \
    .rotation = {0, 0, 0, 1},                    \
    .scale    = {1, 1, 1}                        \
})
```

## spatial_transform_t — compact pose (no scale)

A narrower type for systems that never need scale on a body — most
commonly physics (where non-unit scale breaks inertia) but equally
useful for camera poses, audio emitters, and simple scene anchors.

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

The layout `{ vec3 position; versor rotation; }` matches what most
physics engines already use internally (`PxTransform` in PhysX, Jolt's
`Body` pose, Rapier's `Isometry3`, Havok runtime body). This is a
side-effect of the design, not its purpose: the same shape is also
optimal for a camera rig or a static scene anchor. Scale is kept on
`spatial_pose_t` so systems that do not need it pay nothing.

### Identity

```c
#define SPATIAL_TRANSFORM_IDENTITY ((spatial_transform_t){ \
    .position = {0, 0, 0},                                 \
    .rotation = {0, 0, 0, 1}                               \
})
```

## spatial_dtransform_t — large-world variant

```c
#ifdef SPATIAL_DOUBLE_PRECISION
typedef struct spatial_dtransform_t {
  double position[3];
  versor rotation;
} spatial_dtransform_t;
#endif
```

Opt-in double-precision position for open-world scenes where float
breaks down (typically beyond ~16 km from origin). Rotation stays
float; rotational precision is already sufficient. Compiled only when
`SPATIAL_DOUBLE_PRECISION` is defined.

## Conversions

Implementations **MUST** provide:

```c
void spatial_pose_to_mat4     (const spatial_pose_t      *p, mat4 out);
void spatial_transform_to_mat4(const spatial_transform_t *t, mat4 out);
void spatial_mat4_to_pose     (const mat4 m, spatial_pose_t      *out);
void spatial_mat4_to_transform(const mat4 m, spatial_transform_t *out);
```

`mat4_to_*` functions may lose information if the input matrix
contains shear or non-uniform components that do not decompose
cleanly. Behavior in that case is implementation-defined but
**MUST NOT** crash.

## Zero-copy accessors for hot loops

Render loops, culling jobs, and command recording read world transforms
every frame. Implementations **SHOULD** expose inline accessors that
return `const` pointers directly into SoA storage — no memcpy, no
function-call overhead after inlining:

```c
const mat4   *spatial_node_world_matrix  (const spatial_space_t*, spatial_node_t);
const vec4   *spatial_node_world_position(const spatial_space_t*, spatial_node_t);
const versor *spatial_node_world_rotation(const spatial_space_t*, spatial_node_t);
const vec4   *spatial_node_world_scale   (const spatial_space_t*, spatial_node_t);
/* matching spatial_node_local_* accessors                                        */
```

These accessors **MUST NOT** validate handles (caller's responsibility)
and **MUST** be inline. The safe copy variants (`spatial_node_get_world`,
`spatial_node_get_world_matrix`, …) continue to validate and are the
cold-path API.

See [hotpath.md](hotpath.md) for usage patterns and benchmarks.

## Internal storage note

The public types above describe the ABI. Internally a conformant
implementation **MAY** store positions and scales in wider cells
(e.g. `vec4` / 16-byte aligned) to enable aligned SIMD loads. The
w-lane of such storage is ignored semantically; accessors copy or
expose only the 3-lane value through the declared return type.
