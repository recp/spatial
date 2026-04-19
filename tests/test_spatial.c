/*
 * Spatial unit tests. Compile & run:
 *   clang -I<cglm>/include -Ispatial/include tests/test_spatial.c \
 *         spatial/src/spatial.c -o test && ./test
 */

#include <spatial/spatial.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

static int g_tests  = 0;
static int g_passed = 0;

#define TEST(name) static void test_##name(void)
#define RUN(name)  do {                    \
    g_tests++;                             \
    printf("[  RUN ] %s\n", #name);        \
    test_##name();                         \
    g_passed++;                            \
    printf("[  OK  ] %s\n", #name);        \
  } while (0)

#define CHECK(cond) do {                                              \
    if (!(cond)) {                                                    \
      fprintf(stderr, "  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return;                                                         \
    }                                                                 \
  } while (0)

#define CHECK_FLOAT(a, b) CHECK(fabsf((float)(a) - (float)(b)) < 1e-4f)

/* -------------------------------------------------------------- */

TEST(basic_hierarchy) {
  spatial_space_t *s = spatial_space_create(32);
  spatial_pose_t   p = SPATIAL_POSE_IDENTITY;
  spatial_pose_t   q = SPATIAL_POSE_IDENTITY;
  spatial_pose_t   wb;
  spatial_node_t   a, b;

  p.position[0] = 1.0f;
  q.position[0] = 2.0f;
  a = spatial_node_create(s, SPATIAL_NODE_NULL, &p);
  b = spatial_node_create(s, a, &q);

  spatial_update(s);
  spatial_node_get_world(s, b, &wb);
  CHECK_FLOAT(wb.position[0], 3.0f);

  spatial_space_destroy(s);
}

TEST(physics_authority) {
  spatial_space_t *s = spatial_space_create(8);
  spatial_node_t   a = spatial_node_create(s, SPATIAL_NODE_NULL, NULL);
  spatial_pose_t   w = SPATIAL_POSE_IDENTITY;
  spatial_pose_t   out;

  w.position[0] = 100.0f;
  spatial_node_set_world_physics(s, a, &w);
  spatial_update(s);

  spatial_node_get_world(s, a, &out);
  CHECK_FLOAT(out.position[0], 100.0f);

  spatial_space_destroy(s);
}

TEST(cycle_detection) {
  spatial_space_t *s = spatial_space_create(8);
  spatial_node_t   a = spatial_node_create(s, SPATIAL_NODE_NULL, NULL);
  spatial_node_t   b = spatial_node_create(s, a, NULL);
  spatial_node_t   c = spatial_node_create(s, b, NULL);
  spatial_node_t   d;

  /* Try to attach a under c. This would create a cycle (c -> b -> a -> c). */
  CHECK(!spatial_node_attach(s, a, c));

  /* a should still be under the root. */
  CHECK(spatial_node_eq(spatial_node_get_parent(s, a), s->root));

  /* Legit reparent should work. */
  d = spatial_node_create(s, SPATIAL_NODE_NULL, NULL);
  CHECK(spatial_node_attach(s, c, d));

  spatial_space_destroy(s);
}

TEST(generation_reuse) {
  spatial_space_t *s = spatial_space_create(8);
  spatial_node_t   a, b;
  uint32_t         old_index, old_gen;

  a = spatial_node_create(s, SPATIAL_NODE_NULL, NULL);
  old_index = a.index;
  old_gen   = a.generation;

  spatial_node_destroy(s, a);
  CHECK(!spatial_node_valid(s, a));

  b = spatial_node_create(s, SPATIAL_NODE_NULL, NULL);
  CHECK(b.index == old_index);
  CHECK(b.generation != old_gen);
  CHECK(spatial_node_valid(s, b));
  CHECK(!spatial_node_valid(s, a));

  spatial_space_destroy(s);
}

TEST(deep_hierarchy) {
  spatial_space_t *s;
  spatial_pose_t   step   = SPATIAL_POSE_IDENTITY;
  spatial_pose_t   w;
  spatial_node_t   parent = SPATIAL_NODE_NULL;
  spatial_node_t   leaf   = SPATIAL_NODE_NULL;
  const int        depth  = 200;
  int              i;

  s = spatial_space_create(8);  /* force growth */
  step.position[0] = 1.0f;

  for (i = 0; i < depth; i++) {
    parent = spatial_node_create(s, parent, &step);
    leaf   = parent;
  }

  spatial_update(s);
  spatial_node_get_world(s, leaf, &w);
  CHECK_FLOAT(w.position[0], (float)depth);

  spatial_space_destroy(s);
}

TEST(invalid_handle_safe) {
  spatial_space_t *s     = spatial_space_create(8);
  spatial_node_t   bogus = { .index = 99999, .generation = 1 };
  spatial_pose_t   p     = SPATIAL_POSE_IDENTITY;
  spatial_pose_t   out;

  CHECK(!spatial_node_valid(s, bogus));

  /* All of these should be no-ops, not crashes. */
  spatial_node_set_local(s, bogus, &p);
  spatial_node_set_world_physics(s, bogus, &p);
  spatial_node_destroy(s, bogus);

  CHECK(!spatial_node_get_world(s, bogus, &out));
  CHECK(spatial_node_get_flags(s, bogus) == 0);
  CHECK(spatial_node_is_null(spatial_node_get_parent(s, bogus)));

  spatial_space_destroy(s);
}

TEST(matrix_override) {
  spatial_space_t *s      = spatial_space_create(8);
  spatial_pose_t   base   = SPATIAL_POSE_IDENTITY;
  spatial_node_t   parent, child;
  mat4             skew, world;
  uint32_t         flags;

  base.position[1] = 10.0f;
  parent = spatial_node_create(s, SPATIAL_NODE_NULL, &base);

  glm_mat4_identity(skew);
  skew[3][0] = 5.0f;  /* translate X by 5 */
  skew[1][0] = 0.5f;  /* xy shear */

  child = spatial_node_create(s, parent, NULL);
  spatial_node_set_matrix(s, child, skew);

  spatial_update(s);

  flags = spatial_node_get_flags(s, child);
  CHECK(flags & SPATIAL_NODE_HAS_MATRIX);
  CHECK(s->matrix_overrides[child.index] != NULL);

  spatial_node_get_world_matrix(s, child, world);
  CHECK_FLOAT(world[3][0], 5.0f);
  CHECK_FLOAT(world[3][1], 10.0f);
  CHECK_FLOAT(world[1][0], 0.5f);

  /* Clear override -> falls back to pose path. */
  spatial_node_clear_matrix(s, child);
  spatial_update(s);
  CHECK(!(spatial_node_get_flags(s, child) & SPATIAL_NODE_HAS_MATRIX));
  CHECK(s->matrix_overrides[child.index] == NULL);

  spatial_space_destroy(s);
}

TEST(update_version_bumps) {
  spatial_space_t *s  = spatial_space_create(8);
  uint64_t         v0 = s->update_version;

  spatial_update(s);
  CHECK(s->update_version == v0 + 1);

  spatial_update(s);
  CHECK(s->update_version == v0 + 2);

  spatial_space_destroy(s);
}

TEST(node_version_bumps_on_change) {
  spatial_space_t *s = spatial_space_create(8);
  spatial_node_t   a = spatial_node_create(s, SPATIAL_NODE_NULL, NULL);
  spatial_pose_t   p = SPATIAL_POSE_IDENTITY;
  uint32_t         v0;

  spatial_update(s);
  v0 = spatial_node_get_version(s, a);

  p.position[0] = 42.0f;
  spatial_node_set_local(s, a, &p);
  spatial_update(s);

  CHECK(spatial_node_get_version(s, a) > v0);

  spatial_space_destroy(s);
}

TEST(dirty_dedupe) {
  spatial_space_t *s = spatial_space_create(8);
  spatial_node_t   a = spatial_node_create(s, SPATIAL_NODE_NULL, NULL);
  spatial_node_t   b = spatial_node_create(s, a, NULL);
  spatial_pose_t   p = SPATIAL_POSE_IDENTITY;

  spatial_update(s);  /* clears initial dirties */

  /* Set parent dirty first. */
  p.position[0] = 1.0f;
  spatial_node_set_local(s, a, &p);
  /* Then set child dirty — should be deduped, ancestor covers it. */
  spatial_node_set_local(s, b, &p);

  /* Only one dirty root should be queued (a). b is deduped. */
  CHECK(s->dirty_count == 1);
  CHECK(spatial_node_eq(s->dirty_roots[0], a));

  spatial_space_destroy(s);
}

TEST(reserve_keeps_pointers_stable) {
  spatial_space_t *s = spatial_space_create(8);
  spatial_node_t   a;
  vec4            *p_before;
  int              i;

  spatial_space_reserve(s, 10000);

  a = spatial_node_create(s, SPATIAL_NODE_NULL, NULL);
  p_before = &s->world_positions[a.index];

  /* Create many more nodes. With reserve(10000), no realloc happens. */
  for (i = 0; i < 500; i++) {
    spatial_node_create(s, SPATIAL_NODE_NULL, NULL);
  }

  /* Pointer still valid. */
  CHECK(p_before == &s->world_positions[a.index]);

  spatial_space_destroy(s);
}

TEST(parallel_update) {
  /* Build many disjoint dirty-root subtrees and verify parallel dispatch
   * produces the same world poses as sequential. */
  spatial_space_t *s = spatial_space_create(256);
  spatial_pose_t   step = SPATIAL_POSE_IDENTITY;
  spatial_node_t   roots[64];
  spatial_node_t   children[64];
  const int        n = 64;
  spatial_pose_t   w;
  int              i;

  spatial_space_enable_parallel(s, 4);

  step.position[0] = 1.0f;
  for (i = 0; i < n; i++) {
    spatial_pose_t rp = SPATIAL_POSE_IDENTITY;
    rp.position[1] = (float)i;
    roots[i]    = spatial_node_create(s, SPATIAL_NODE_NULL, &rp);
    children[i] = spatial_node_create(s, roots[i], &step);
  }

  spatial_update(s);

  /* every child should have world = (1, i, 0) */
  for (i = 0; i < n; i++) {
    spatial_node_get_world(s, children[i], &w);
    CHECK_FLOAT(w.position[0], 1.0f);
    CHECK_FLOAT(w.position[1], (float)i);
  }

  /* mutate all roots, expect children to follow */
  for (i = 0; i < n; i++) {
    spatial_pose_t rp = SPATIAL_POSE_IDENTITY;
    rp.position[0] = 10.0f;
    rp.position[1] = (float)i * 2.0f;
    spatial_node_set_local(s, roots[i], &rp);
  }
  spatial_update(s);
  for (i = 0; i < n; i++) {
    spatial_node_get_world(s, children[i], &w);
    CHECK_FLOAT(w.position[0], 11.0f);
    CHECK_FLOAT(w.position[1], (float)i * 2.0f);
  }

  spatial_space_destroy(s);
}

TEST(soa_direct_access) {
  /* Data-oriented hot path: physics writes directly to SoA arrays. */
  spatial_space_t *s = spatial_space_create(8);
  spatial_node_t   a = spatial_node_create(s, SPATIAL_NODE_NULL, NULL);
  uint32_t         i = a.index;

  s->world_positions[i][0] = 42.0f;
  s->world_positions[i][1] =  7.0f;
  s->flags[i] |= SPATIAL_NODE_DIRTY_WORLD | SPATIAL_NODE_PHYSICS_OWNS;
  /* Skip push_dirty, manually add */
  s->dirty_roots[s->dirty_count++] = a;

  spatial_update(s);

  CHECK_FLOAT(s->world_positions[i][0], 42.0f);
  CHECK_FLOAT(s->world_positions[i][1],  7.0f);
  CHECK_FLOAT(s->world_matrices[i][3][0], 42.0f);

  spatial_space_destroy(s);
}

/* -------------------------------------------------------------- */
/* 2D tests                                                       */
/* -------------------------------------------------------------- */

TEST(basic_hierarchy_2d) {
  spatial_space2_t *s = spatial_space2_create(16);
  spatial_pose2_t   p = SPATIAL_POSE2_IDENTITY;
  spatial_pose2_t   q = SPATIAL_POSE2_IDENTITY;
  spatial_pose2_t   wb;
  spatial_node_t    a, b;

  p.position[0] = 1.0f; p.position[1] = 2.0f;
  q.position[0] = 3.0f; q.position[1] = 4.0f;
  a = spatial_node2_create(s, SPATIAL_NODE_NULL, &p);
  b = spatial_node2_create(s, a, &q);

  spatial_update2(s);
  spatial_node2_get_world(s, b, &wb);
  CHECK_FLOAT(wb.position[0], 4.0f);
  CHECK_FLOAT(wb.position[1], 6.0f);

  spatial_space2_destroy(s);
}

TEST(rotation_2d) {
  spatial_space2_t *s = spatial_space2_create(8);
  spatial_pose2_t   parent_pose = SPATIAL_POSE2_IDENTITY;
  spatial_pose2_t   child_pose  = SPATIAL_POSE2_IDENTITY;
  spatial_pose2_t   wb;
  spatial_node_t    a, b;

  parent_pose.rotation.c = 0.0f;
  parent_pose.rotation.s = 1.0f;
  child_pose.position[0] = 1.0f;
  a = spatial_node2_create(s, SPATIAL_NODE_NULL, &parent_pose);
  b = spatial_node2_create(s, a, &child_pose);

  spatial_update2(s);
  spatial_node2_get_world(s, b, &wb);
  CHECK_FLOAT(wb.position[0], 0.0f);
  CHECK_FLOAT(wb.position[1], 1.0f);

  spatial_space2_destroy(s);
}

TEST(rot2_mul_composition) {
  spatial_rot2_t a   = { .c = cosf(M_PI/6), .s = sinf(M_PI/6) };
  spatial_rot2_t b   = { .c = cosf(M_PI/3), .s = sinf(M_PI/3) };
  spatial_rot2_t out;
  spatial_rot2_mul(a, b, &out);

  CHECK_FLOAT(out.c, 0.0f);
  CHECK_FLOAT(out.s, 1.0f);
}

/* -------------------------------------------------------------- */

int main(void) {
  RUN(basic_hierarchy);
  RUN(physics_authority);
  RUN(cycle_detection);
  RUN(generation_reuse);
  RUN(deep_hierarchy);
  RUN(invalid_handle_safe);
  RUN(matrix_override);
  RUN(update_version_bumps);
  RUN(node_version_bumps_on_change);
  RUN(dirty_dedupe);
  RUN(soa_direct_access);
  RUN(reserve_keeps_pointers_stable);
  RUN(parallel_update);
  RUN(basic_hierarchy_2d);
  RUN(rotation_2d);
  RUN(rot2_mul_composition);

  printf("\n%d/%d tests passed\n", g_passed, g_tests);
  return (g_passed == g_tests) ? 0 : 1;
}
