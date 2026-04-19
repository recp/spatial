/*
 * Copyright (C) 2026 Recep Aslantas
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 */

#include "../thread/thread.h"

#include <string.h>
#include <stdlib.h>

typedef struct spatial__thread_entry_t {
  void *arg;
  void (*func)(void *);
} spatial__thread_entry_t;

static
DWORD WINAPI
spatial__thread_trampoline(LPVOID arg) {
  spatial__thread_entry_t entry;
  memcpy(&entry, arg, sizeof(entry));
  free(arg);
  entry.func(entry.arg);
  return 0;
}

SPATIAL_HIDE
spatial_thread_t *
spatial_thread_new(void (*func)(void *), void *arg) {
  spatial_thread_t        *t;
  spatial__thread_entry_t *entry;

  t     = calloc(1, sizeof(*t));
  entry = calloc(1, sizeof(*entry));
  if (!t || !entry) { free(t); free(entry); return NULL; }
  entry->func = func;
  entry->arg  = arg;

  t->handle = CreateThread(NULL, 0, spatial__thread_trampoline, entry, 0, NULL);
  if (!t->handle) {
    free(t); free(entry); return NULL;
  }
  return t;
}

SPATIAL_HIDE
void
spatial_thread_join(spatial_thread_t *t) {
  WaitForSingleObject(t->handle, INFINITE);
  CloseHandle(t->handle);
  t->handle = NULL;
}

SPATIAL_HIDE
void
spatial_thread_free(spatial_thread_t *t) {
  if (t->handle) CloseHandle(t->handle);
  free(t);
}

SPATIAL_HIDE void spatial_thread_mutex_init   (spatial_thread_mutex_t *m) { InitializeCriticalSection(&m->cs); }
SPATIAL_HIDE void spatial_thread_mutex_destroy(spatial_thread_mutex_t *m) { DeleteCriticalSection(&m->cs);     }
SPATIAL_HIDE void spatial_thread_mutex_lock   (spatial_thread_mutex_t *m) { EnterCriticalSection(&m->cs);      }
SPATIAL_HIDE void spatial_thread_mutex_unlock (spatial_thread_mutex_t *m) { LeaveCriticalSection(&m->cs);      }

SPATIAL_HIDE void spatial_thread_cond_init     (spatial_thread_cond_t *c) { InitializeConditionVariable(&c->cv); }
SPATIAL_HIDE void spatial_thread_cond_destroy  (spatial_thread_cond_t *c) { (void)c; /* no destroy on CV */    }
SPATIAL_HIDE void spatial_thread_cond_wait     (spatial_thread_cond_t *c, spatial_thread_mutex_t *m) {
  SleepConditionVariableCS(&c->cv, &m->cs, INFINITE);
}
SPATIAL_HIDE void spatial_thread_cond_signal   (spatial_thread_cond_t *c) { WakeConditionVariable(&c->cv);    }
SPATIAL_HIDE void spatial_thread_cond_broadcast(spatial_thread_cond_t *c) { WakeAllConditionVariable(&c->cv); }

SPATIAL_HIDE
uint32_t
spatial_thread_hw_concurrency(void) {
  SYSTEM_INFO si;
  GetSystemInfo(&si);
  return si.dwNumberOfProcessors > 0 ? (uint32_t)si.dwNumberOfProcessors : 1u;
}
