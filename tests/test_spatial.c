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
#define RUN(name)  do { \
    g_tests++; \
    printf("[  RUN ] %s\n", #name); \
    test_##name(); \
    g_passed++; \
    printf("[  OK  ] %s\n", #name); \
  } while (0)

#define CHECK(cond) do { \
    if (!(cond)) { \
      fprintf(stderr, "  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return; \
    } \
  } while (0)

#define CHECK_FLOAT(a, b) CHECK(fabsf((float)(a) - (float)(b)) < 1e-4f)

/* -------------------------------------------------------------- */

TEST(basic_hierarchy) {
  spatial_space_t *s = spatial_space_create(32);

  spatial_pose_t p = SPATIAL_POSE_IDENTITY;
  p.position[0] = 1.0f;
  spatial_node_t a = spatial_node_create(s, SPATIAL_NODE_NULL, &p);

  spatial_pose_t q = SPATIAL_POSE_IDENTITY;
  q.position[0] = 2.0f;
  spatial_node_t b = spatial_node_create(s, a, &q);

  spatial_update(s);

  spatial_pose_t wb;
  spatial_node_get_world(s, b, &wb);
  CHECK_FLOAT(wb.position[0], 3.0f);

  spatial_space_destroy(s);
}

TEST(physics_authority) {
  spatial_space_t *s = spatial_space_create(8);
  spatial_node_t a = spatial_node_create(s, SPATIAL_NODE_NULL, NULL);

  spatial_pose_t w = SPATIAL_POSE_IDENTITY;
  w.position[0] = 100.0f;
  spatial_node_set_world_physics(s, a, &w);
  spatial_update(s);

  spatial_pose_t out;
  spatial_node_get_world(s, a, &out);
  CHECK_FLOAT(out.position[0], 100.0f);

  spatial_space_destroy(s);
}

TEST(cycle_detection) {
  spatial_space_t *s = spatial_space_create(8);
  spatial_node_t a = spatial_node_create(s, SPATIAL_NODE_NULL, NULL);
  spatial_node_t b = spatial_node_create(s, a, NULL);
  spatial_node_t c = spatial_node_create(s, b, NULL);

  /* Try to attach a under c. This would create a cycle (c -> b -> a -> c). */
  CHECK(!spatial_node_attach(s, a, c));

  /* a should still be under the root (not under c). */
  spatial_node_data_t *na = spatial_node_get(s, a);
  CHECK(spatial_node_eq(na->parent, s->root));

  /* Legit reparent should work. */
  spatial_node_t d = spatial_node_create(s, SPATIAL_NODE_NULL, NULL);
  CHECK(spatial_node_attach(s, c, d));

  spatial_space_destroy(s);
}

TEST(generation_reuse) {
  spatial_space_t *s = spatial_space_create(8);
  spatial_node_t a = spatial_node_create(s, SPATIAL_NODE_NULL, NULL);
  uint32_t old_index = a.index;
  uint32_t old_gen   = a.generation;

  spatial_node_destroy(s, a);
  CHECK(!spatial_node_valid(s, a));  /* stale handle fails */

  /* New node should reuse the slot but with bumped generation. */
  spatial_node_t b = spatial_node_create(s, SPATIAL_NODE_NULL, NULL);
  CHECK(b.index == old_index);
  CHECK(b.generation != old_gen);
  CHECK(spatial_node_valid(s, b));
  CHECK(!spatial_node_valid(s, a));  /* still stale */

  spatial_space_destroy(s);
}

TEST(deep_hierarchy) {
  spatial_space_t *s;
  spatial_pose_t   step = SPATIAL_POSE_IDENTITY;
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
  spatial_space_t *s = spatial_space_create(8);

  spatial_node_t bogus = { .index = 99999, .generation = 1 };
  CHECK(!spatial_node_valid(s, bogus));

  /* All of these should be no-ops, not crashes. */
  spatial_pose_t p = SPATIAL_POSE_IDENTITY;
  spatial_node_set_local(s, bogus, &p);
  spatial_node_set_world_physics(s, bogus, &p);
  spatial_node_destroy(s, bogus);

  spatial_pose_t out;
  CHECK(!spatial_node_get_world(s, bogus, &out));

  spatial_space_destroy(s);
}

TEST(matrix_override) {
  spatial_space_t *s = spatial_space_create(8);

  spatial_pose_t base = SPATIAL_POSE_IDENTITY;
  base.position[1] = 10.0f;
  spatial_node_t parent = spatial_node_create(s, SPATIAL_NODE_NULL, &base);

  /* Skew-like matrix on child (non-decomposable ideally — here just a translate). */
  mat4 skew;
  glm_mat4_identity(skew);
  skew[3][0] = 5.0f;  /* translate X by 5 */
  skew[1][0] = 0.5f;  /* xy shear */

  spatial_node_t child = spatial_node_create(s, parent, NULL);
  spatial_node_set_matrix(s, child, skew);

  spatial_update(s);

  spatial_node_data_t *nc = spatial_node_get(s, child);
  CHECK(nc->flags & SPATIAL_NODE_HAS_MATRIX);
  CHECK(nc->matrix_override != NULL);

  /* world_matrix = parent.world_matrix * skew_local.
     parent world is translate Y=10, so world[3] = (5, 10, 0). */
  CHECK_FLOAT(nc->world_matrix[3][0], 5.0f);
  CHECK_FLOAT(nc->world_matrix[3][1], 10.0f);
  /* shear should survive */
  CHECK_FLOAT(nc->world_matrix[1][0], 0.5f);

  /* Clear override -> falls back to pose path. */
  spatial_node_clear_matrix(s, child);
  spatial_update(s);
  CHECK(!(nc->flags & SPATIAL_NODE_HAS_MATRIX));
  CHECK(nc->matrix_override == NULL);

  spatial_space_destroy(s);
}

TEST(update_version_bumps) {
  spatial_space_t *s = spatial_space_create(8);
  uint64_t v0 = s->update_version;

  spatial_update(s);
  CHECK(s->update_version == v0 + 1);

  spatial_update(s);
  CHECK(s->update_version == v0 + 2);

  spatial_space_destroy(s);
}

TEST(node_version_bumps_on_change) {
  spatial_space_t *s = spatial_space_create(8);
  spatial_node_t a = spatial_node_create(s, SPATIAL_NODE_NULL, NULL);
  spatial_update(s);

  spatial_node_data_t *n = spatial_node_get(s, a);
  uint32_t v0 = n->version;

  spatial_pose_t p = SPATIAL_POSE_IDENTITY;
  p.position[0] = 42.0f;
  spatial_node_set_local(s, a, &p);
  spatial_update(s);

  CHECK(n->version > v0);

  spatial_space_destroy(s);
}

/* -------------------------------------------------------------- */
/* 2D tests                                                       */
/* -------------------------------------------------------------- */

TEST(basic_hierarchy_2d) {
  spatial_space2_t *s = spatial_space2_create(16);

  spatial_pose2_t p = SPATIAL_POSE2_IDENTITY;
  p.position[0] = 1.0f; p.position[1] = 2.0f;
  spatial_node_t a = spatial_node2_create(s, SPATIAL_NODE_NULL, &p);

  spatial_pose2_t q = SPATIAL_POSE2_IDENTITY;
  q.position[0] = 3.0f; q.position[1] = 4.0f;
  spatial_node_t b = spatial_node2_create(s, a, &q);

  spatial_update2(s);

  spatial_pose2_t wb;
  spatial_node2_get_world(s, b, &wb);
  CHECK_FLOAT(wb.position[0], 4.0f);
  CHECK_FLOAT(wb.position[1], 6.0f);

  spatial_space2_destroy(s);
}

TEST(rotation_2d) {
  spatial_space2_t *s = spatial_space2_create(8);

  /* Parent rotated 90 degrees CCW. Child at local (1, 0). World should be (0, 1). */
  spatial_pose2_t parent_pose = SPATIAL_POSE2_IDENTITY;
  parent_pose.rotation.c = 0.0f;
  parent_pose.rotation.s = 1.0f;
  spatial_node_t a = spatial_node2_create(s, SPATIAL_NODE_NULL, &parent_pose);

  spatial_pose2_t child_pose = SPATIAL_POSE2_IDENTITY;
  child_pose.position[0] = 1.0f;
  spatial_node_t b = spatial_node2_create(s, a, &child_pose);

  spatial_update2(s);

  spatial_pose2_t wb;
  spatial_node2_get_world(s, b, &wb);
  CHECK_FLOAT(wb.position[0], 0.0f);
  CHECK_FLOAT(wb.position[1], 1.0f);

  spatial_space2_destroy(s);
}

TEST(rot2_mul_composition) {
  /* 30 deg * 60 deg = 90 deg */
  spatial_rot2_t a = { .c = cosf(M_PI/6), .s = sinf(M_PI/6) };
  spatial_rot2_t b = { .c = cosf(M_PI/3), .s = sinf(M_PI/3) };
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
  RUN(basic_hierarchy_2d);
  RUN(rotation_2d);
  RUN(rot2_mul_composition);

  printf("\n%d/%d tests passed\n", g_passed, g_tests);
  return (g_passed == g_tests) ? 0 : 1;
}
