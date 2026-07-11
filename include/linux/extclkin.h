/* SPDX-License-Identifier: GPL-2.0 */
/*
 * In-kernel accessor for the extclkin-gpt driver (drivers/clocksource/extclkin-gpt.c).
 *
 * Returns a (host monotonic ns, external reference clock ns) pair captured
 * atomically, safe to call from interrupt/softirq context. Falls back to a
 * stub returning -ENODEV when CONFIG_EXTCLKIN_GPT isn't reachable, so callers
 * don't need their own IS_REACHABLE() guards.
 */
#ifndef _LINUX_EXTCLKIN_H
#define _LINUX_EXTCLKIN_H

#include <linux/types.h>

#if IS_REACHABLE(CONFIG_EXTCLKIN_GPT)
int extclkin_gpt_read_raw(u64 *host_ns, u64 *ref_ns);
#else
static inline int extclkin_gpt_read_raw(u64 *host_ns, u64 *ref_ns)
{
	return -ENODEV;
}
#endif

#endif /* _LINUX_EXTCLKIN_H */
