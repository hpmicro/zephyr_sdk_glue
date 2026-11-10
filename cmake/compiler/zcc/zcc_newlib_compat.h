/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef HPMICRO_ZCC_NEWLIB_COMPAT_H_
#define HPMICRO_ZCC_NEWLIB_COMPAT_H_

#ifdef CONFIG_POSIX_API

#include <signal.h>
#include <sys/sched.h>

#ifndef _PID_T_DECLARED
typedef __pid_t pid_t;
#define _PID_T_DECLARED
#endif

#ifndef SIGRTMIN
#define SIGRTMIN 32
#endif

#ifndef SIGRTMAX
#define SIGRTMAX (SIGRTMIN + CONFIG_POSIX_RTSIG_MAX)
#endif

#endif /* CONFIG_POSIX_API */

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_POSIX_API
int pthread_sigmask(int how, const sigset_t *set, sigset_t *oset);
#endif

#ifdef CONFIG_MINIMAL_LIBC
double cos(double x);
double round(double x);
double sin(double x);
#endif

#ifdef __cplusplus
}
#endif

#endif /* HPMICRO_ZCC_NEWLIB_COMPAT_H_ */
