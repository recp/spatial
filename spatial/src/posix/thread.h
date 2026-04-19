/*
 * Copyright (C) 2026 Recep Aslantas
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef spatial_posix_thread_h
#define spatial_posix_thread_h

#include <pthread.h>

typedef struct spatial_thread_t {
  pthread_t id;
} spatial_thread_t;

typedef struct spatial_thread_mutex_t {
  pthread_mutex_t m;
} spatial_thread_mutex_t;

typedef struct spatial_thread_cond_t {
  pthread_cond_t c;
} spatial_thread_cond_t;

#endif /* spatial_posix_thread_h */
