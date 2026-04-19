/*
 * Stress test: random mix of create/destroy/attach/set/update with
 * handle validation. Checks no crash, no leak, state consistency.
 * Combine with ASan/TSan via CMake options.
 */

#include <spatial/spatial.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define POOL_SIZE 512
#define OPS       50000
#define SEED_DEFAULT 1337u

int main(int argc, char **argv) {
  spatial_space_t *s;
  spatial_node_t   pool[POOL_SIZE];
  uint32_t         alive = 0;
  uint32_t         i, seed;
  int              op;

  seed = (argc > 1) ? (uint32_t)atoi(argv[1]) : SEED_DEFAULT;
  srand(seed);

  s = spatial_space_create(64);
  spatial_space_reserve(s, POOL_SIZE * 2);
  spatial_space_enable_parallel(s, 0);

  for (i = 0; i < POOL_SIZE; i++) pool[i] = SPATIAL_NODE_NULL;

  for (op = 0; op < OPS; op++) {
    uint32_t idx = (uint32_t)rand() % POOL_SIZE;
    int      act = rand() % 10;

    switch (act) {
      case 0: case 1: case 2:
        /* create if empty */
        if (spatial_node_is_null(pool[idx])) {
          spatial_pose_t p = SPATIAL_POSE_IDENTITY;
          p.position[0] = (float)(rand() % 1000) * 0.1f;
          pool[idx] = spatial_node_create(s, SPATIAL_NODE_NULL, &p);
          if (!spatial_node_is_null(pool[idx])) alive++;
        }
        break;
      case 3:
        /* destroy if alive */
        if (!spatial_node_is_null(pool[idx])) {
          spatial_node_destroy(s, pool[idx]);
          pool[idx] = SPATIAL_NODE_NULL;
          alive--;
        }
        break;
      case 4: case 5: {
        /* set_local */
        spatial_pose_t p = SPATIAL_POSE_IDENTITY;
        p.position[0] = (float)(rand() % 100);
        p.position[1] = (float)(rand() % 100);
        spatial_node_set_local(s, pool[idx], &p);
        break;
      }
      case 6: {
        /* attach: random other handle as parent */
        uint32_t jdx = (uint32_t)rand() % POOL_SIZE;
        if (!spatial_node_is_null(pool[idx]) && !spatial_node_is_null(pool[jdx])
            && idx != jdx) {
          spatial_node_attach(s, pool[idx], pool[jdx]);
        }
        break;
      }
      case 7: {
        /* physics write */
        spatial_pose_t p = SPATIAL_POSE_IDENTITY;
        p.position[0] = (float)(rand() % 1000);
        spatial_node_set_world_physics(s, pool[idx], &p);
        break;
      }
      case 8: {
        /* get_world consistency check */
        spatial_pose_t w;
        if (spatial_node_get_world(s, pool[idx], &w)) {
          /* quat magnitude should be ~1 */
          float m = w.rotation[0]*w.rotation[0] + w.rotation[1]*w.rotation[1]
                  + w.rotation[2]*w.rotation[2] + w.rotation[3]*w.rotation[3];
          if (m < 0.9f || m > 1.1f) {
            fprintf(stderr, "FAIL: quat drift m=%.4f at op=%d\n", m, op);
            return 1;
          }
        }
        break;
      }
      case 9:
        spatial_update(s);
        break;
    }

    if ((op % 500) == 0) spatial_update(s);
  }

  spatial_update(s);

  /* cleanup */
  for (i = 0; i < POOL_SIZE; i++) {
    if (!spatial_node_is_null(pool[i])) spatial_node_destroy(s, pool[i]);
  }
  spatial_update(s);
  spatial_space_destroy(s);

  printf("stress: %d ops, seed=%u, final_alive=%u — OK\n", OPS, seed, alive);
  return 0;
}
