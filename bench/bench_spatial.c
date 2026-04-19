/*
 * Minimal benchmark suite. Measures update throughput across hierarchy
 * shapes and parallel vs sequential dispatch.
 *
 * Build:
 *   cmake -B build -DSPATIAL_BUILD_BENCH=ON && cmake --build build
 *   ./build/bench_spatial
 */

#include <spatial/spatial.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

#if defined(_WIN32)
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#endif

static double
now_ms(void) {
#if defined(_WIN32)
  static LARGE_INTEGER freq;
  LARGE_INTEGER        counter;

  if (freq.QuadPart == 0) {
    QueryPerformanceFrequency(&freq);
  }

  QueryPerformanceCounter(&counter);
  return (double)counter.QuadPart * 1000.0 / (double)freq.QuadPart;
#else
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
#endif
}

static void
bench_flat(uint32_t n, int iters, bool parallel) {
  spatial_space_t *s = spatial_space_create(n + 16);
  spatial_node_t  *handles;
  spatial_pose_t   p = SPATIAL_POSE_IDENTITY;
  double           t0, t1;
  uint32_t         i;
  int              k;

  spatial_space_reserve(s, n + 16);
  if (parallel) spatial_space_enable_parallel(s, 0);

  handles = malloc(sizeof(spatial_node_t) * n);
  for (i = 0; i < n; i++) {
    p.position[0] = (float)i;
    handles[i] = spatial_node_create(s, SPATIAL_NODE_NULL, &p);
  }
  spatial_update(s);  /* warmup */

  t0 = now_ms();
  for (k = 0; k < iters; k++) {
    for (i = 0; i < n; i++) {
      p.position[0] = (float)(i + k);
      spatial_node_set_local(s, handles[i], &p);
    }
    spatial_update(s);
  }
  t1 = now_ms();

  printf("  flat n=%u iters=%d %s: %.2f ms (%.2f us/update, %.2f ns/node)\n",
         n, iters, parallel ? "parallel" : "sequential",
         t1 - t0,
         (t1 - t0) * 1000.0 / iters,
         (t1 - t0) * 1e6 / ((double)iters * n));

  free(handles);
  spatial_space_destroy(s);
}

static void
bench_deep(uint32_t depth, int iters) {
  spatial_space_t *s = spatial_space_create(depth + 16);
  spatial_node_t   leaf = SPATIAL_NODE_NULL;
  spatial_pose_t   step = SPATIAL_POSE_IDENTITY;
  double           t0, t1;
  uint32_t         i;
  int              k;

  spatial_space_reserve(s, depth + 16);
  step.position[0] = 1.0f;

  for (i = 0; i < depth; i++) {
    leaf = spatial_node_create(s, leaf, &step);
  }
  spatial_update(s);  /* warmup */

  t0 = now_ms();
  for (k = 0; k < iters; k++) {
    step.position[1] = (float)k;
    /* force full re-propagation by dirtying the root chain */
    spatial_node_set_local(s, leaf, &step);
    spatial_update(s);
  }
  t1 = now_ms();

  printf("  deep depth=%u iters=%d: %.2f ms (%.2f us/update)\n",
         depth, iters, t1 - t0, (t1 - t0) * 1000.0 / iters);

  spatial_space_destroy(s);
}

/* Simulates a render loop reading world_matrix for each drawable.
 * Compares zero-copy pointer accessor vs the safe-copy variant. */
static void
bench_render_read(uint32_t n, int iters) {
  spatial_space_t *s = spatial_space_create(n + 16);
  spatial_node_t  *handles;
  volatile float   sink = 0.0f;   /* prevent dead-code elimination */
  double           t0, t1;
  uint32_t         i;
  int              k;

  spatial_space_reserve(s, n + 16);
  handles = malloc(sizeof(spatial_node_t) * n);
  for (i = 0; i < n; i++) {
    spatial_pose_t p = SPATIAL_POSE_IDENTITY;
    p.position[0] = (float)i;
    handles[i] = spatial_node_create(s, SPATIAL_NODE_NULL, &p);
  }
  spatial_update(s);

  /* --- copy accessor --- */
  t0 = now_ms();
  for (k = 0; k < iters; k++) {
    for (i = 0; i < n; i++) {
      mat4 m;
      spatial_node_get_world_matrix(s, handles[i], m);
      sink += m[3][0];
    }
  }
  t1 = now_ms();
  printf("  render-read n=%u iters=%d COPY: %.2f ms (%.2f ns/node)\n",
         n, iters, t1 - t0, (t1 - t0) * 1e6 / ((double)iters * n));

  /* --- zero-copy accessor --- */
  t0 = now_ms();
  for (k = 0; k < iters; k++) {
    for (i = 0; i < n; i++) {
      const mat4 *m = spatial_node_world_matrix(s, handles[i]);
      sink += (*m)[3][0];
    }
  }
  t1 = now_ms();
  printf("  render-read n=%u iters=%d PTR:  %.2f ms (%.2f ns/node)\n",
         n, iters, t1 - t0, (t1 - t0) * 1e6 / ((double)iters * n));

  /* --- direct SoA --- */
  t0 = now_ms();
  for (k = 0; k < iters; k++) {
    for (i = 0; i < n; i++) {
      sink += s->world_matrices[handles[i].index][3][0];
    }
  }
  t1 = now_ms();
  printf("  render-read n=%u iters=%d SoA:  %.2f ms (%.2f ns/node)\n",
         n, iters, t1 - t0, (t1 - t0) * 1e6 / ((double)iters * n));

  (void)sink;
  free(handles);
  spatial_space_destroy(s);
}

static void
bench_physics_write(uint32_t n, int iters) {
  /* Measures just the write path (no propagation) for copy-setter vs
   * direct-SoA. spatial_update is not called inside the timed loops. */
  spatial_space_t *s = spatial_space_create(n + 16);
  spatial_node_t  *handles;
  double           t0, t1;
  uint32_t         i;
  int              k;

  spatial_space_reserve(s, n + 16);
  handles = malloc(sizeof(spatial_node_t) * n);
  for (i = 0; i < n; i++) {
    handles[i] = spatial_node_create(s, SPATIAL_NODE_NULL, NULL);
  }
  spatial_update(s);

  /* --- copy setter (spatial_node_set_world_physics) --- */
  t0 = now_ms();
  for (k = 0; k < iters; k++) {
    spatial_pose_t w = SPATIAL_POSE_IDENTITY;
    w.position[0] = (float)k;
    for (i = 0; i < n; i++) {
      spatial_node_set_world_physics(s, handles[i], &w);
    }
    s->dirty_count = 0;  /* reset without update for pure write measurement */
    for (i = 0; i < n; i++) s->flags[handles[i].index] &= ~(SPATIAL_NODE_DIRTY_WORLD);
  }
  t1 = now_ms();
  printf("  physics-write n=%u iters=%d SET: %.2f ms (%.2f ns/write)\n",
         n, iters, t1 - t0, (t1 - t0) * 1e6 / ((double)iters * n));

  /* --- direct SoA write --- */
  t0 = now_ms();
  for (k = 0; k < iters; k++) {
    for (i = 0; i < n; i++) {
      uint32_t idx = handles[i].index;
      s->world_positions[idx][0] = (float)k;
      s->flags[idx] |= SPATIAL_NODE_DIRTY_WORLD | SPATIAL_NODE_PHYSICS_OWNS;
    }
    s->dirty_count = 0;
    for (i = 0; i < n; i++) s->flags[handles[i].index] &= ~(SPATIAL_NODE_DIRTY_WORLD);
  }
  t1 = now_ms();
  printf("  physics-write n=%u iters=%d SoA: %.2f ms (%.2f ns/write)\n",
         n, iters, t1 - t0, (t1 - t0) * 1e6 / ((double)iters * n));

  free(handles);
  spatial_space_destroy(s);
}

static void
bench_physics_hotpath(uint32_t n, int iters) {
  /* Write + propagate (spatial_update). */
  spatial_space_t *s = spatial_space_create(n + 16);
  spatial_node_t  *handles;
  double           t0, t1;
  uint32_t         i;
  int              k;

  spatial_space_reserve(s, n + 16);
  handles = malloc(sizeof(spatial_node_t) * n);
  for (i = 0; i < n; i++) {
    handles[i] = spatial_node_create(s, SPATIAL_NODE_NULL, NULL);
    s->flags[handles[i].index] |= SPATIAL_NODE_PHYSICS_OWNS;
  }
  spatial_update(s);

  t0 = now_ms();
  for (k = 0; k < iters; k++) {
    for (i = 0; i < n; i++) {
      uint32_t idx = handles[i].index;
      s->world_positions[idx][0] = (float)(i + k);
      s->flags[idx] |= SPATIAL_NODE_DIRTY_WORLD;
    }
    for (i = 0; i < n; i++) s->dirty_roots[i] = handles[i];
    s->dirty_count = n;
    spatial_update(s);
  }
  t1 = now_ms();

  printf("  physics-hotpath n=%u iters=%d: %.2f ms (%.2f ns/write+propagate)\n",
         n, iters, t1 - t0,
         (t1 - t0) * 1e6 / ((double)iters * n));

  free(handles);
  spatial_space_destroy(s);
}

int main(void) {
  printf("spatial benchmarks\n");

  printf("\n[flat hierarchy — sequential vs parallel crossover]\n");
  bench_flat(10,    10000, false);
  bench_flat(10,    10000, true);
  bench_flat(50,    10000, false);
  bench_flat(50,    10000, true);
  bench_flat(100,   10000, false);
  bench_flat(100,   10000, true);
  bench_flat(500,   1000,  false);
  bench_flat(500,   1000,  true);
  bench_flat(1000,  1000,  false);
  bench_flat(1000,  1000,  true);
  bench_flat(10000, 200,   false);
  bench_flat(10000, 200,   true);

  printf("\n[deep chain]\n");
  bench_deep(100,  10000);
  bench_deep(1000, 1000);

  printf("\n[render-read: copy vs zero-copy vs direct SoA]\n");
  bench_render_read(10000, 1000);

  printf("\n[physics write: copy-setter vs direct SoA]\n");
  bench_physics_write(10000, 1000);

  printf("\n[physics hot-path: write + propagate]\n");
  bench_physics_hotpath(10000, 200);

  return 0;
}
