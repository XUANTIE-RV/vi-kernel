/****************************************************************************

 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 VeriSilicon Holdings Co., Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 *****************************************************************************
 *
 * The GPL License (GPL)
 *
 * Copyright (c) 2020 VeriSilicon Holdings Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program;
 *
 *****************************************************************************
 *
 * Note: This software is released under dual MIT and GPL licenses. A
 * recipient may use this file under the terms of either the MIT license or
 * GPL License. If you wish to use only one license not the other, you can
 * indicate your decision by deleting one of the above license notices in your
 * version of this file.
 *
 *****************************************************************************/

#ifdef __KERNEL__
#include <asm/io.h>

#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/mm.h>

#include <linux/timer.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#else
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#endif
#include "isp_irq_queue.h"

  //enqueue
int isp_irq_enqueue(isp_mis_t *new,isp_mis_t* head)
  {
#ifdef __KERNEL__
    isp_mis_t* new_node = (isp_mis_t*)kmalloc(sizeof(isp_mis_t), GFP_KERNEL); //create new node


    if (new == NULL || head == NULL) {
        //printk("%s: input wrong parameter\n", __func__);
        return -1;
    }
    new_node->val = new->val;
    new_node->irq_src = new->irq_src;
    /*printk("%s: new_node %px irq_src %d", __func__, new_node,  new->irq_src);*/
    INIT_LIST_HEAD(&new_node->list);
    list_add_tail(&new_node->list, &head->list);             //append to tail
 #endif
    return 0;

  }

  //dequeue && release memory
int isp_irq_dequeue(isp_mis_t* data, isp_mis_t* head)
  {

#ifdef __KERNEL__
    isp_mis_t* entry;
    if (data == NULL || head == NULL) {
        //printk("%s: input wrong parameter\n", __func__);
        return -1;
    }
    if (list_empty(&head->list)) {
        //printk("%s: There is no node\n", __func__);
        return -1;
    }

    entry = list_first_entry(&head->list, isp_mis_t, list);
    /*printk("%s: entry %px irq_src %d", __func__, entry,  entry->irq_src);*/
    data->val = entry->val;
    data->irq_src = entry->irq_src;
    list_del_init(&entry->list);

    kfree(entry);
#endif
    return 0;
  }

bool isp_irq_is_queue_empty( isp_mis_t* head)
  {
#ifdef __KERNEL__
      return list_empty(&head->list);
#endif
      return 0;
  }

int isp_irq_create_circle_queue(isp_mis_list_t* pCList, int number)
{
#ifdef __KERNEL__
  int i;
  isp_mis_t* pMisNode;
  if (pCList == NULL || number <= 0) {
      printk("%s: create circle queue failed\n", __func__);
      return -1;
  }

  if (pCList->pHead == NULL) {

      pCList->pHead = (isp_mis_t*)kmalloc(sizeof(isp_mis_t), GFP_KERNEL);
      INIT_LIST_HEAD(&pCList->pHead->list);
      pCList->pRead = pCList->pHead;
      pCList->pWrite = pCList->pHead;
  }
  printk("%s:pHead %px\n", __func__, pCList->pHead);
  for (i = 0; i < number - 1; i++) {
      pMisNode = (isp_mis_t*)kmalloc(sizeof(isp_mis_t), GFP_KERNEL);
      INIT_LIST_HEAD(&pMisNode->list);
      list_add_tail(&pMisNode->list, &pCList->pHead->list);
      printk("%s:pMisNode %px\n", __func__, pMisNode);
  }

#endif
  return 0;
}

int isp_irq_destroy_circle_queue(isp_mis_list_t* pCList)
{
#ifdef __KERNEL__
  isp_mis_t* pMisNode;
  if ((pCList == NULL) || (pCList->pHead == NULL) ) {
      printk("%s: destroy circle queue failed. pClist %px\n", __func__, pCList);
      return -1;
  }

  while(!list_empty(&pCList->pHead->list)) {
      pMisNode = list_first_entry(&pCList->pHead->list, isp_mis_t, list);
      printk("%s:pMisNode %px\n", __func__, pMisNode);
      list_del(&pMisNode->list);
      kfree(pMisNode);
      pMisNode = NULL;
  }
  printk("%s:pHead %px\n", __func__, pCList->pHead);
  kfree(pCList->pHead);
  pCList->pHead = NULL;
  pCList->pRead = NULL;
  pCList->pWrite = NULL;
#endif
  return 0;
}

int isp_irq_read_circle_queue(isp_mis_t* data, isp_mis_list_t* pCList)
{
#ifdef __KERNEL__
  //isp_mis_t* pReadEntry;
  if (pCList == NULL) {
      printk("%s: can not read circle queue\n", __func__);
      return -1;
  }

  if (pCList->pRead == pCList->pWrite) {
    /*printk("%s: There is no irq mis data\n", __func__);*/
    return -1;
  }
  data->val = pCList->pRead->val;
  data->irq_src = pCList->pRead->irq_src;
  /*printk("%s: entry %px irq_src %d, msi %08x\n", __func__, pCList->pRead,  data->irq_src, data->val);*/
  /*Get the next entry that link with read entry list*/
  /*Update read pointer to next entry*/
  pCList->pRead = list_first_entry(&pCList->pRead->list, isp_mis_t, list);

  //pCList->pRead = pReadEntry;

#endif
  return 0;
}

int isp_irq_write_circle_queue(isp_mis_t* data, isp_mis_list_t* pCList)
{
#ifdef __KERNEL__
  isp_mis_t* pWriteEntry;
  if (pCList == NULL) {
      printk("%s: can not read circle queue\n", __func__);
      return -1;
  }

  pCList->pWrite->val = data->val;
  pCList->pWrite->irq_src = data->irq_src;
  /*printk("%s: entry %px irq_src %d, msi %08x\n", __func__,  pCList->pWrite,  data->irq_src, data->val);*/
  /*get the next write entry pointer that link with the write entry list*/
  pWriteEntry = list_first_entry(&pCList->pWrite->list, isp_mis_t, list);

  /*Update write pointer to point next entry*/
  pCList->pWrite = pWriteEntry;

#endif
  return 0;
}

