/* drivers/sharp/shsys/sh_sleeplog.c
 *
 * Copyright (C) 2013 SHARP CORPORATION
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/rculist.h>
#include <linux/pm_wakeup.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

#include <sharp/sh_sleeplog.h>

static spinlock_t lock;

struct alarm_count{
	int function;
	int count;
};
struct alarm_count_chain{
	struct list_head link;
	struct alarm_count count;
};
#define MAX_ALARM_COUNT 30
struct alarm_count_chain allocated_alarm_count[MAX_ALARM_COUNT];
LIST_HEAD(alarm_list);

#define MAX_WAKEUP_SOURCES_NAME 16
#define MAX_WAKEUP_SOURCES_COUNT 150

#define IRQ_TABLE_SIZE 1021
#define PMIC_WAKEUP_GIC_IRQ 222
static int sh_irq_count[IRQ_TABLE_SIZE];
static bool sh_pmic_wakeup_flag;

struct screen_state{
	struct timespec time;
	int state;
};
#define MAX_SCREEN_STATE_ARRAY 100
struct screen_state screen_state_array[MAX_SCREEN_STATE_ARRAY];
static int screen_state_write_point = 0;

#define MAX_UID_STATS 100
#define BAD_PROCESS_STRING "UNKNOWN"
static int uid_stats_write_count = 0;

#define ALARM_SIZE 180
#define WAKEUP_SOURCES_SIZE 3600
#define IRQ_SIZE 2042
#define PM_STATS_SIZE 16
#define SCREEN_STATE_SIZE 1000
#define TCP_DUMP_SIZE 4200

#define ALARM_OFFSET 0
#define WAKEUP_SOURCES_OFFSET (ALARM_OFFSET + ALARM_SIZE)
#define IRQ_OFFSET (WAKEUP_SOURCES_OFFSET + WAKEUP_SOURCES_SIZE)
#define PM_STATS_OFFSET (IRQ_OFFSET + IRQ_SIZE)
#define SCREEN_STATE_OFFSET (PM_STATS_OFFSET + PM_STATS_SIZE)
#define TCP_DUMP_OFFSET (SCREEN_STATE_OFFSET + SCREEN_STATE_SIZE)

#define BUFFER_SIZE (ALARM_SIZE + WAKEUP_SOURCES_SIZE + IRQ_SIZE + PM_STATS_SIZE + SCREEN_STATE_SIZE + TCP_DUMP_SIZE)

static char *dump_buffer = NULL;
static int dumped_size = 0;

void sh_write_buffer_word(char *buffer, int value)
{
	unsigned short temp = value > 0x0000FFFF ? 0xFFFF : (unsigned short)value;
	memcpy(buffer, &temp, 2);
}

static void sh_write_buffer_alarm_list(char *buffer){
	struct list_head *pos;
	
	list_for_each(pos, &alarm_list){
		struct alarm_count_chain *c = 
			list_entry(pos, struct alarm_count_chain, link);
		memcpy(buffer, &c->count.function, sizeof(int));
		sh_write_buffer_word(buffer+sizeof(int), c->count.count);
		buffer += sizeof(int) + sizeof(short);
	}
}

void sh_write_buffer_wakeup_sources_internal(char *buffer, struct list_head *wakeup_sources)
{
	struct wakeup_source *ws;
	ktime_t active_time;
	unsigned long flags;
	ktime_t total_time;
	int copy_size;
	int i = 0;

	rcu_read_lock();
	list_for_each_entry_rcu(ws, wakeup_sources, entry){
		spin_lock_irqsave(&ws->lock, flags);
		total_time = ws->total_time;
		if (ws->active) {
			ktime_t now = ktime_get();
			active_time = ktime_sub(now, ws->last_time);
			total_time = ktime_add(total_time, active_time);
		}
		copy_size = strlen(ws->name) < MAX_WAKEUP_SOURCES_NAME ? 
			strlen(ws->name) : MAX_WAKEUP_SOURCES_NAME;
		memcpy(buffer, &total_time, sizeof(ktime_t));
		memcpy(buffer+sizeof(ktime_t), ws->name, copy_size);
		spin_unlock_irqrestore(&ws->lock, flags);

		buffer += sizeof(ktime_t) + MAX_WAKEUP_SOURCES_NAME;
		i++;
		if(i == MAX_WAKEUP_SOURCES_COUNT)
			break;
	}
	rcu_read_unlock();
}

static void sh_write_buffer_irq_counter(char *buffer)
{
	int i = 0;

	for(i=0; i<IRQ_TABLE_SIZE; i++){
		sh_write_buffer_word(buffer, sh_irq_count[i]);
		buffer += sizeof(short);
	}
}

static void sh_write_buffer_pm_stats(char *buffer)
{
	int64_t suspend_result = sh_get_pm_stats_suspend();
	int64_t idle_result = sh_get_pm_stats_idle();

	memcpy(buffer, &suspend_result, sizeof(int64_t));
	memcpy(buffer+sizeof(int64_t), &idle_result, sizeof(int64_t));
}

static void sh_write_buffer_screen_state(char *buffer){
	int i = 0;

	for(i=0; i<MAX_SCREEN_STATE_ARRAY; i++){
		memcpy(buffer, &screen_state_array[i].time, sizeof(struct timespec));
		sh_write_buffer_word(buffer + sizeof(struct timespec), screen_state_array[i].state);
		buffer += sizeof(struct timespec) + sizeof(short);
	}
}

char *sh_write_buffer_uid_stat_internal(char *buffer, 
	uid_t uid, char *process_name, unsigned int tcp_rcv, unsigned int tcp_snd)
{
	sh_write_buffer_word(buffer, uid);
	memcpy(buffer + sizeof(short), process_name, UID_STATS_MAX_PROCESS_NAME);
	memcpy(buffer + sizeof(short) + UID_STATS_MAX_PROCESS_NAME,
		 &tcp_rcv, sizeof(unsigned int));
	memcpy(buffer + sizeof(short) + UID_STATS_MAX_PROCESS_NAME + sizeof(unsigned int),
		 &tcp_snd, sizeof(unsigned int));
	buffer += sizeof(short) + UID_STATS_MAX_PROCESS_NAME +
		sizeof(unsigned int) + sizeof(unsigned int);

	uid_stats_write_count++;
	if(uid_stats_write_count > MAX_UID_STATS)
		return 0;

	return (char *)buffer;
}

static void init_data(void){
	struct list_head *pos;
	unsigned long flags;

	spin_lock_irqsave(&lock, flags);
	list_for_each(pos, &alarm_list){
		struct alarm_count_chain *c;
		c = list_entry(pos, struct alarm_count_chain, link);
		c->count.count = 0;
	}
	memset(sh_irq_count, 0, sizeof(int) * IRQ_TABLE_SIZE);
	memset(screen_state_array, 0, sizeof(screen_state_array));
	spin_unlock_irqrestore(&lock, flags);
}

static int create_buffer(void)
{
	int size = PAGE_ALIGN(BUFFER_SIZE);
	if(dump_buffer == NULL) {
		if( (dump_buffer = kzalloc(size, GFP_KERNEL)) == NULL ){
			pr_err("shsleeplog : fail to allocate buffer.\n");
			return 0;
		}
	}
	dumped_size = 0;

	sh_write_buffer_alarm_list(dump_buffer);
	sh_write_buffer_wakeup_sources(dump_buffer + WAKEUP_SOURCES_OFFSET);
	sh_write_buffer_irq_counter(dump_buffer + IRQ_OFFSET);
	sh_write_buffer_pm_stats(dump_buffer + PM_STATS_OFFSET);
	sh_write_buffer_screen_state(dump_buffer + SCREEN_STATE_OFFSET);
	uid_stats_write_count = 0;
	sh_write_buffer_uid_stat(dump_buffer + TCP_DUMP_OFFSET);
	init_data();

	return size;
}

static void delete_buffer(void)
{
	if(dump_buffer != NULL) {
		kfree(dump_buffer);
		dump_buffer = NULL;
	}
}

void sh_count_mark_alarm(enum android_alarm_type alarm_type, int function)
{
	struct list_head *pos;
	int i = 0;
	unsigned long flags;

	if(alarm_type != ANDROID_ALARM_RTC_WAKEUP &&
		alarm_type != ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP)
		return;
	
	list_for_each(pos, &alarm_list) {
		struct alarm_count_chain *c;
		c = list_entry(pos, struct alarm_count_chain, link);
		if(c->count.function == function){
			spin_lock_irqsave(&lock, flags);
			c->count.count++;
			spin_unlock_irqrestore(&lock, flags);
			return;
		}
		i++;
	}
	
	if(i < MAX_ALARM_COUNT){
		spin_lock_irqsave(&lock, flags);
		allocated_alarm_count[i].count.function = function;
		allocated_alarm_count[i].count.count    = 1;
		list_add_tail(&allocated_alarm_count[i].link, pos);
		spin_unlock_irqrestore(&lock, flags);
	}
}

static void sh_up_pmic_wakeup_flag(void)
{
	sh_pmic_wakeup_flag = true;
}

static bool sh_is_pmic_wakeup_flag(void)
{
	return sh_pmic_wakeup_flag;
}

static void sh_down_pmic_wakeup_flag(void)
{
	sh_pmic_wakeup_flag = false;
}

void sh_count_gic_counter(int irq)
{
	sh_count_irq_counter(irq);
	if(irq == PMIC_WAKEUP_GIC_IRQ)
		sh_up_pmic_wakeup_flag();
}

void sh_count_irq_counter(int irq)
{
	unsigned long flags;

	if(irq >= 0 && irq < IRQ_TABLE_SIZE){
		spin_lock_irqsave(&lock, flags);
		sh_irq_count[irq]++;
		spin_unlock_irqrestore(&lock, flags);
	}
}

void sh_count_irq_if_pmic_wakeup(int irq)
{
	if(sh_is_pmic_wakeup_flag()){
		sh_count_irq_counter(irq);
		sh_down_pmic_wakeup_flag();
	}
}

void sh_set_screen_state(struct timespec ts, suspend_state_t state)
{
	unsigned long flags;

	spin_lock_irqsave(&lock, flags);
	screen_state_array[screen_state_write_point].time = ts;
	screen_state_array[screen_state_write_point].state =
		state > PM_SUSPEND_ON ? 0 : 1;

	screen_state_write_point =
		(screen_state_write_point + 1) % MAX_SCREEN_STATE_ARRAY;
	spin_unlock_irqrestore(&lock, flags);
}

void sh_get_process_name(struct task_struct *task, char *result_name)
{
	struct mm_struct *mm;
	unsigned int len;

	memset(result_name, 0, UID_STATS_MAX_PROCESS_NAME);
	if(!task){
		strncpy(result_name, BAD_PROCESS_STRING, strlen(BAD_PROCESS_STRING));
		return;
	}

	mm = get_task_mm(task);

	if (!mm || !mm->arg_end){
		strncpy(result_name, BAD_PROCESS_STRING, strlen(BAD_PROCESS_STRING));
	}else{
		len = mm->arg_end - mm->arg_start < UID_STATS_MAX_PROCESS_NAME ? 
			mm->arg_end - mm->arg_start : UID_STATS_MAX_PROCESS_NAME;
		access_process_vm(task, mm->arg_start, result_name, len, 0);
	}
	if(mm){
		mmput(mm);
	}
}

static int sh_sleeplog_mmap(struct file *filep, struct vm_area_struct *vma)
{
	int ret = 0;
	int size;
	unsigned long pfn;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long length = vma->vm_end - vma->vm_start;

	size = create_buffer();
	if(dump_buffer == NULL) return -EINVAL;
	if( (offset >= size) || (length > (size - offset)) ) {
		ret = -EINVAL;
		goto exit_after_alloc;
	}

	vma->vm_flags |= VM_RESERVED;
	pfn = virt_to_phys(dump_buffer + offset) >> PAGE_SHIFT;
	if( (ret = remap_pfn_range(vma, vma->vm_start, pfn, length, vma->vm_page_prot)) != 0 ){
		pr_err("shsleeplog : failed to remap : %d\n", ret);
		ret = -EAGAIN;
		goto exit_after_alloc;
	}

exit_after_alloc:
	delete_buffer();

	return ret;
}

static ssize_t sh_sleeplog_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
	int size   = BUFFER_SIZE;

	if (count == sizeof(int)) {
		if(copy_to_user(buf, (void *)&size, sizeof(int)))
			return -EFAULT;
		return count;
	}

	return -EFAULT;
}

static struct file_operations sh_sleeplog_fops = {
    .owner          = THIS_MODULE,
    .mmap           = sh_sleeplog_mmap,
    .read           = sh_sleeplog_read,
};

static struct miscdevice sh_sleeplog_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "sh_sleeplog",
    .fops = &sh_sleeplog_fops,
};

static int __init sh_sleeplog_init( void )
{
    int ret;
    spin_lock_init(&lock);
    ret = misc_register(&sh_sleeplog_dev);
    return ret;
}
static void __exit sh_sleeplog_exit(void)
{
	misc_deregister(&sh_sleeplog_dev);
}

module_init(sh_sleeplog_init);
module_exit(sh_sleeplog_exit);

MODULE_DESCRIPTION("sh_sleeplog");
MODULE_LICENSE("GPL v2");
