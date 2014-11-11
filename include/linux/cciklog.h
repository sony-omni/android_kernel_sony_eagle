#ifndef __KLOG_DRIVER_H__
#define __KLOG_DRIVER_H__

#ifndef __KLOG_COMMON_H__
#include <generated/cciklog_common.h>
#endif // #ifndef __KLOG_COMMON_H__

#ifdef CONFIG_CCI_KLOG
//#define CCI_KLOG_DETAIL_LOG
#define CCI_HW_ID
#ifdef IMEM_CERT_RECORD
#define CCI_KLOG_SBL_BOOT_TIME_USE_IMEM
#endif // #ifdef IMEM_CERT_RECORD
#define CCI_KLOG_MODEM_CRASH_LOG_USE_SMEM
#define CCI_KLOG_SUPPORT_ATTRIBUTE
#include "../../drivers/staging/android/logger.h"

#define kprintk(fmt, args...)					printk(KERN_CRIT KLOG_LOG_TAG fmt, ##args)

struct klog_time
{
	struct timespec	clock;
	struct timespec	rtc;
};

void cklc_append_kernel_raw_char(unsigned char c);
void cklc_append_str(unsigned int category, unsigned char *str, size_t len);
void cklc_append_newline(unsigned int category);
void cklc_append_separator(unsigned int category);
void cklc_append_time_header(unsigned int category);
void show_android_log_to_console(void);
void cklc_append_android_log(unsigned int category, const struct logger_entry *header, const unsigned char *log_priority, const char * const log_tag, const int log_tag_bytes, const char * const log_msg, const int log_msg_bytes);
void cklc_save_magic(char *magic, int state);
void cklc_set_memory_ready(void);
int match_crash_priority(int priority);
void update_priority(void);
#ifdef CONFIG_CCI_KLOG_RECORD_RPM_VERSION
void klog_record_rpm_version(const char *str);
#endif // #ifdef CONFIG_CCI_KLOG_RECORD_RPM_VERSION
int get_fault_state(void);
void set_fault_state(int level, int type, const char* msg);
void set_kernel_log_level(int level);
void record_shutdown_time(int state);
struct timespec klog_get_kernel_clock_timestamp(void);
#ifdef CCI_KLOG_ALLOW_FORCE_PANIC
int get_force_panic_when_suspend(void);
int get_force_panic_when_power_off(void);
#endif // #ifdef CCI_KLOG_ALLOW_FORCE_PANIC

#else // #ifdef CONFIG_CCI_KLOG

#define cklc_append_kernel_raw_char(c)				do {} while (0)
#define cklc_append_str(category, str, len)			do {} while (0)
#define cklc_append_newline(category)				do {} while (0)
#define cklc_append_separator(category)				do {} while (0)
#define cklc_append_time_header(category)			do {} while (0)
#define cklc_save_magic						do {} while (0)
#define cklc_set_memory_ready					do {} while (0)
#define klog_record_rpm_version					do {} while (0)

#endif // #ifdef CONFIG_CCI_KLOG

#endif // #ifndef __KLOG_DRIVER_H__

