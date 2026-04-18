/*
 * Copyright (C) 2023 Recep Aslantas
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef spatial_common_h
#define spatial_common_h

#if defined(_MSC_VER)
#  ifdef SPATIAL_STATIC
#    define SPATIAL_EXPORT
#  elif defined(SPATIAL_EXPORTS)
#    define SPATIAL_EXPORT __declspec(dllexport)
#  else
#    define SPATIAL_EXPORT __declspec(dllimport)
#  endif
#  define SPATIAL_HIDE
#  define SPATIAL_INLINE __forceinline
#  define SPATIAL_ALIGN(X) __declspec(align(X))
#else
#  define SPATIAL_EXPORT  __attribute__((visibility("default")))
#  define SPATIAL_HIDE    __attribute__((visibility("hidden")))
#  define SPATIAL_INLINE inline __attribute((always_inline))
#  define SPATIAL_ALIGN(X) __attribute((aligned(X)))
#endif

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/* Dependency: https://github.com/recp/cglm */
#include <cglm/cglm.h>

#endif /* spatial_common_h */
