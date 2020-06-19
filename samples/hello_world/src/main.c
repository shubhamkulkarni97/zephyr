/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>

void main(void)
{
	printk("Hello World! %s\n", CONFIG_BOARD);
	struct k_mutex *my_mutex = (struct k_mutex *) k_malloc(sizeof(struct k_mutex));
	if (my_mutex == NULL) {
		printk("Mutex creation failed\n");
	}
	k_mutex_lock(my_mutex, K_FOREVER);
	k_mutex_unlock(my_mutex);
	printk("Done\n");
}
