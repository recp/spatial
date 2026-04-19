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

/* Multi-threaded writer test uses pthread directly. Skipped on
 * platforms without pthread (Windows). The same pattern is available
 * via the internal spatial_thread_* abstraction; the test deliberately
 * avoids that so it stays runnable without exposing internal headers. */
#ifndef _WIN32
#include <pthread.h>
#define SPATIAL_TEST_HAS_PTHREAD 1
#else
#define SPATIAL_TEST_HAS_PTHREAD 0
#endif

#if SPATIAL_TEST_HAS_PTHREAD
typedef struct mt_writer_arg_t {
  spatial_space_t *s;
  spatial_node_t  *handles;
  uint32_t         start;
  uint32_t         end;
  uint32_t         k;
} mt_writer_arg_t;

static void *mt_writer_main(void *arg) {
  mt_writer_arg_t *a = (mt_writer_arg_t *)arg;
  uint32_t         i;
  for (i = a->start; i < a->end; i++) {
    uint32_t idx = a->handles[i].index;
    a->s->world_positions[idx][0] = (float)(i + a->k);
    a->s->flags[idx] |= SPATIAL_NODE_PHYSICS_OWNS;
    spatial_node_mark_dirty_mt(a->s, a->handles[i], SPATIAL_NODE_DIRTY_WORLD);
  }
  return NULL;
}

TEST(mt_writer_disjoint) {
  /* Simulate physics jobs: N threads write disjoint body ranges in
   * parallel via spatial_node_mark_dirty_mt, then spatial_update. */
  spatial_space_t *s = spatial_space_create(16);
  const uint32_t   n = 400;
  spatial_node_t   handles[400];
  pthread_t        threads[4];
  mt_writer_arg_t  args[4];
  uint32_t         i;
  int              t;

  spatial_space_reserve(s, n + 16);
  spatial_space_reserve_dirty(s, n + 16);

  for (i = 0; i < n; i++) {
    handles[i] = spatial_node_create(s, SPATIAL_NODE_NULL, NULL);
  }
  spatial_update(s);

  for (t = 0; t < 4; t++) {
    args[t].s       = s;
    args[t].handles = handles;
    args[t].start   = t * (n / 4);
    args[t].end     = (t + 1) * (n / 4);
    args[t].k       = 100u;
    pthread_create(&threads[t], NULL, mt_writer_main, &args[t]);
  }
  for (t = 0; t < 4; t++) pthread_join(threads[t], NULL);

  spatial_update(s);

  for (i = 0; i < n; i++) {
    spatial_pose_t w;
    spatial_node_get_world(s, handles[i], &w);
    CHECK_FLOAT(w.position[0], (float)(i + 100));
  }

  spatial_space_destroy(s);
}
#endif /* SPATIAL_TEST_HAS_PTHREAD */

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

TEST(authority_transfer) {
  spatial_space_t *s = spatial_space_create(8);
  spatial_pose_t   p = SPATIAL_POSE_IDENTITY;
  spatial_pose_t   w = SPATIAL_POSE_IDENTITY;
  spatial_pose_t   out;
  spatial_node_t   a;

  /* Phase 1: local-authored. */
  p.position[0] = 1.0f;
  a = spatial_node_create(s, SPATIAL_NODE_NULL, &p);
  spatial_update(s);
  spatial_node_get_world(s, a, &out);
  CHECK_FLOAT(out.position[0], 1.0f);

  /* Phase 2: physics takes ownership, writes world. */
  spatial_node_set_physics_authority(s, a, true);
  CHECK(spatial_node_get_flags(s, a) & SPATIAL_NODE_PHYSICS_OWNS);
  w.position[0] = 99.0f;
  spatial_node_set_world_physics(s, a, &w);
  spatial_update(s);
  spatial_node_get_world(s, a, &out);
  CHECK_FLOAT(out.position[0], 99.0f);

  /* Phase 3: hand authority back to local. Local should drive world. */
  spatial_node_set_physics_authority(s, a, false);
  CHECK(!(spatial_node_get_flags(s, a) & SPATIAL_NODE_PHYSICS_OWNS));
  p.position[0] = 7.0f;
  spatial_node_set_local(s, a, &p);
  spatial_update(s);
  spatial_node_get_world(s, a, &out);
  CHECK_FLOAT(out.position[0], 7.0f);

  spatial_space_destroy(s);
}

TEST(mt_dedupe_same_node) {
  /* Calling mark_dirty_mt on the same node multiple times MUST land
   * exactly one entry in dirty_roots. */
  spatial_space_t *s = spatial_space_create(8);
  spatial_node_t   a;

  spatial_space_reserve_dirty(s, 32);
  a = spatial_node_create(s, SPATIAL_NODE_NULL, NULL);
  spatial_update(s);  /* clear initial dirty */

  spatial_node_mark_dirty_mt(s, a, SPATIAL_NODE_DIRTY_WORLD);
  spatial_node_mark_dirty_mt(s, a, SPATIAL_NODE_DIRTY_WORLD);
  spatial_node_mark_dirty_mt(s, a, SPATIAL_NODE_DIRTY_WORLD);

  CHECK(s->dirty_count == 1);
  CHECK(spatial_node_eq(s->dirty_roots[0], a));

  spatial_space_destroy(s);
}

TEST(mat4_to_transform_conversion) {
  mat4                 m;
  spatial_transform_t  t;

  glm_mat4_identity(m);
  m[3][0] = 10.0f; m[3][1] = 20.0f; m[3][2] = 30.0f;

  spatial_mat4_to_transform(m, &t);
  CHECK_FLOAT(t.position[0], 10.0f);
  CHECK_FLOAT(t.position[1], 20.0f);
  CHECK_FLOAT(t.position[2], 30.0f);
  CHECK_FLOAT(t.rotation[3], 1.0f);  /* identity quat w */
}

TEST(zero_copy_accessors) {
  spatial_space_t *s = spatial_space_create(8);
  spatial_pose_t   p = SPATIAL_POSE_IDENTITY;
  spatial_node_t   a;
  const mat4      *mp;
  const vec4      *pp;
  const versor    *qp;

  p.position[0] = 7.0f;
  p.position[1] = 8.0f;
  p.position[2] = 9.0f;
  a = spatial_node_create(s, SPATIAL_NODE_NULL, &p);
  spatial_update(s);

  mp = spatial_node_world_matrix(s, a);
  pp = spatial_node_world_position(s, a);
  qp = spatial_node_world_rotation(s, a);

  /* Pointers must alias the SoA arrays (no copy). */
  CHECK(mp == (const mat4 *)&s->world_matrices[a.index]);
  CHECK(pp == (const vec4 *)&s->world_positions[a.index]);
  CHECK(qp == (const versor *)&s->world_rotations[a.index]);

  /* Values readable via pointer. */
  CHECK_FLOAT((*pp)[0], 7.0f);
  CHECK_FLOAT((*pp)[1], 8.0f);
  CHECK_FLOAT((*pp)[2], 9.0f);
  CHECK_FLOAT((*mp)[3][0], 7.0f);

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
  RUN(authority_transfer);
  RUN(mt_dedupe_same_node);
  RUN(mat4_to_transform_conversion);
  RUN(zero_copy_accessors);
  RUN(soa_direct_access);
  RUN(reserve_keeps_pointers_stable);
#if SPATIAL_TEST_HAS_PTHREAD
  RUN(mt_writer_disjoint);
#endif
  RUN(parallel_update);
  RUN(basic_hierarchy_2d);
  RUN(rotation_2d);
  RUN(rot2_mul_composition);

  printf("\n%d/%d tests passed\n", g_passed, g_tests);
  return (g_passed == g_tests) ? 0 : 1;
}
