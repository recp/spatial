/*
 * Copyright (C) 2026 Recep Aslantas
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef spatial_thread_h
#define spatial_thread_h

#include "../../include/spatial/common.h"

#if defined(_WIN32) || defined(_MSC_VER)
#  include "../win/thread.h"
#else
#  include "../posix/thread.h"
#endif

/* Common thread abstraction. All implementations provide the types
 * spatial_thread_t, spatial_thread_mutex_t, spatial_thread_cond_t
 * through their platform header. */

SPATIAL_HIDE spatial_thread_t *spatial_thread_new(void (*func)(void *), void *arg);
SPATIAL_HIDE void              spatial_thread_join(spatial_thread_t *t);
SPATIAL_HIDE void              spatial_thread_free(spatial_thread_t *t);

SPATIAL_HIDE void spatial_thread_mutex_init   (spatial_thread_mutex_t *m);
SPATIAL_HIDE void spatial_thread_mutex_destroy(spatial_thread_mutex_t *m);
SPATIAL_HIDE void spatial_thread_mutex_lock   (spatial_thread_mutex_t *m);
SPATIAL_HIDE void spatial_thread_mutex_unlock (spatial_thread_mutex_t *m);

SPATIAL_HIDE void spatial_thread_cond_init     (spatial_thread_cond_t *c);
SPATIAL_HIDE void spatial_thread_cond_destroy  (spatial_thread_cond_t *c);
SPATIAL_HIDE void spatial_thread_cond_wait     (spatial_thread_cond_t *c,
                                                spatial_thread_mutex_t *m);
SPATIAL_HIDE void spatial_thread_cond_signal   (spatial_thread_cond_t *c);
SPATIAL_HIDE void spatial_thread_cond_broadcast(spatial_thread_cond_t *c);

SPATIAL_HIDE uint32_t spatial_thread_hw_concurrency(void);

#endif /* spatial_thread_h */
