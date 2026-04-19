/*
 * Copyright (C) 2026 Recep Aslantas
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef spatial_win_thread_h
#define spatial_win_thread_h

#ifndef WIN32_LEAN_AND_MEAN
#  define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

typedef struct spatial_thread_t {
  HANDLE handle;
} spatial_thread_t;

typedef struct spatial_thread_mutex_t {
  CRITICAL_SECTION cs;
} spatial_thread_mutex_t;

typedef struct spatial_thread_cond_t {
  CONDITION_VARIABLE cv;
} spatial_thread_cond_t;

#endif /* spatial_win_thread_h */
