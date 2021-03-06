/*
 * Copyright (C) 2012 The Android Open Source Project
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

#ifndef __DEBUG_H__
#define __DEBUG_H__

#ifdef  NDEBUG
#undef  NDEBUG
#endif  /*NDEBUG*/

#include <assert.h>
#define ASSERT                  assert

#undef  ANDROID
#ifdef  ANDROID
//[[ANDROID

#include <log/log.h>

#define DIE(fmt, ...)       \
    do { ALOGE("[%s:%d] " fmt , __func__, __LINE__, ## __VA_ARGS__); assert(0); } while(0)

#define ERR(fmt, ...)       \
    do { ALOGE("[%s:%d] " fmt , __func__, __LINE__, ## __VA_ARGS__); } while(0)

#define INFO(fmt, ...)       \
    do { ALOGI("[%s:%d] " fmt , __func__, __LINE__, ## __VA_ARGS__); } while(0)

#define DBG(fmt, ...)       \
    do { ALOGD("[%s:%d] " fmt , __func__, __LINE__, ## __VA_ARGS__); } while(0)

#define TRACE_FUNC              \
    do { ALOGD("[TRACE_FUNC >>>>> %s:%d]", __func__, __LINE__); } while(0)

#define TRACE_LINE              \
    do { ALOGD("[TRACE_LINE >>>>> %s:%d]", __FILE__, __LINE__); } while(0)

// ]]ANDROID
#else
// [[!ANDROID

#include <stdio.h>

#ifdef  DEBUG
#define __dbg_output                __dbg_output_nonl
#else
#define __dbg_output(...)           do { } while (0)
#endif  /*DEBUG*/
#define __info_output               __info_output_nonl

#define __dbg_output_nl(fmt, ...)       \
    do { fprintf(stderr, \
            "[%s:%d] " fmt "\n", __func__, __LINE__, ## __VA_ARGS__); } while(0)
#define __dbg_output_nonl(fmt, ...)       \
    do { fprintf(stderr, \
            "[%s:%d] " fmt, __func__, __LINE__, ## __VA_ARGS__); } while(0)

#define __info_output_nl(fmt, ...)       \
    do { fprintf(stdout, fmt "\n", ## __VA_ARGS__); } while(0)

#define __info_output_nonl(fmt, ...)       \
    do { fprintf(stdout, fmt, ## __VA_ARGS__); } while(0)

#define DIE(fmt, ...)       \
    do { __dbg_output(fmt, ## __VA_ARGS__); assert(0); } while(0)

#define ERR                     __dbg_output
#define INFO                    __info_output
#define DBG                     __dbg_output

#define TRACE_FUNC              \
    do { fprintf(stderr, \
            "[TRACE_FUNC >>>>> %s:%d]\n", __func__, __LINE__); } while(0)

#define TRACE_LINE              \
    do { fprintf(stderr, \
            "[TRACE_LINE >>>>> %s:%d]\n", __FILE__, __LINE__); } while(0)

// ]]!ANDROID
#endif  /*ANDROID*/

#define ESTIMATE_START(title) \
    do { \
        unsigned int __diff_usec; \
        unsigned int __boundary = 0; \
        const char *__estimate_title = title; \
        struct timeval __s_tv, __e_tv; \
        gettimeofday(&__s_tv, (struct timezone *)0)

#define ESTIMATE_START_BOUND(title, b) \
    do { \
        unsigned int __diff_usec; \
        unsigned int __boundary = b; \
        const char *__estimate_title = title; \
        struct timeval __s_tv, __e_tv; \
        gettimeofday(&__s_tv, (struct timezone *)0)

#define ESTIMATE_STOP() \
        gettimeofday(&__e_tv, (struct timezone *)0); \
        __diff_usec = (__e_tv.tv_sec - __s_tv.tv_sec) * 1000000; \
        __diff_usec += (__e_tv.tv_usec - __s_tv.tv_usec); \
        if(__diff_usec >= __boundary) \
        DBG("ESTIMATED TIME[%s] : %d(usec)\n", \
                __estimate_title, __diff_usec); \
    } while(0)

#define __CHECK_LINE__          TRACE_LINE
#define __CHECK_FUNC__          TRACE_FUNC

#endif  /*__DEBUG_H__*/
