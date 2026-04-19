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

#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

typedef struct spatial__thread_entry_t {
  void *arg;
  void (*func)(void *);
} spatial__thread_entry_t;

static
void *
spatial__thread_trampoline(void *arg) {
  spatial__thread_entry_t entry;

  memcpy(&entry, arg, sizeof(entry));
  free(arg);

  entry.func(entry.arg);
  return NULL;
}

SPATIAL_HIDE
spatial_thread_t *
spatial_thread_new(void (*func)(void *), void *arg) {
  spatial_thread_t        *t;
  spatial__thread_entry_t *entry;
  pthread_attr_t           attr;

  t     = calloc(1, sizeof(*t));
  entry = calloc(1, sizeof(*entry));
  if (!t || !entry) { free(t); free(entry); return NULL; }
  entry->func = func;
  entry->arg  = arg;

  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
  pthread_create(&t->id, &attr, spatial__thread_trampoline, entry);
  pthread_attr_destroy(&attr);

  return t;
}

SPATIAL_HIDE
void
spatial_thread_join(spatial_thread_t *t) {
  pthread_join(t->id, NULL);
}

SPATIAL_HIDE
void
spatial_thread_free(spatial_thread_t *t) {
  free(t);
}

SPATIAL_HIDE void spatial_thread_mutex_init   (spatial_thread_mutex_t *m) { pthread_mutex_init(&m->m, NULL); }
SPATIAL_HIDE void spatial_thread_mutex_destroy(spatial_thread_mutex_t *m) { pthread_mutex_destroy(&m->m);    }
SPATIAL_HIDE void spatial_thread_mutex_lock   (spatial_thread_mutex_t *m) { pthread_mutex_lock(&m->m);       }
SPATIAL_HIDE void spatial_thread_mutex_unlock (spatial_thread_mutex_t *m) { pthread_mutex_unlock(&m->m);     }

SPATIAL_HIDE void spatial_thread_cond_init     (spatial_thread_cond_t *c) { pthread_cond_init(&c->c, NULL);  }
SPATIAL_HIDE void spatial_thread_cond_destroy  (spatial_thread_cond_t *c) { pthread_cond_destroy(&c->c);     }
SPATIAL_HIDE void spatial_thread_cond_wait     (spatial_thread_cond_t *c, spatial_thread_mutex_t *m) { pthread_cond_wait(&c->c, &m->m); }
SPATIAL_HIDE void spatial_thread_cond_signal   (spatial_thread_cond_t *c) { pthread_cond_signal(&c->c);      }
SPATIAL_HIDE void spatial_thread_cond_broadcast(spatial_thread_cond_t *c) { pthread_cond_broadcast(&c->c);   }

SPATIAL_HIDE
uint32_t
spatial_thread_hw_concurrency(void) {
  long n = sysconf(_SC_NPROCESSORS_ONLN);
  return (n > 1) ? (uint32_t)n : 1u;
}
