/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.
This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/
#include <errno.h>

// 可能修复 Ubuntu 16.04 下的兼容性问题
#define _GNU_SOURCE  // Possible fix for 16.04
#define __USE_GNU
// #include "common/pthreads_cross.h"
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "workerpool.h"
// #include "debug_print.h"

/*  主线程负责分配任务、设置启动信号、等待子线程完成。主线程通过 endcond 知道所有线程已完成。  子线程负责等待信号、执行任务、更新完成计数。
主线程: workerpool_run()
    |
    v
加锁 mutex
    |
    v
end_count = 0
start_predicate = true
广播 startcond
    |
    v
等待 end_count == nthreads  <---------------------------+
    |                                                     |
    v                                                     |
解锁 mutex                                                 |
    |                                                     |
    v                                                     |
清空任务队列 <--------------------------------------------+
    |
    v
结束


子线程: worker_thread()
    |
    v
加锁 mutex
    |
    v
while (没有任务 或 start_predicate==false)
    end_count++
    广播 endcond
    等待 startcond
    |
    v
取出任务 (tasks[taskspos++])
解锁 mutex
    |
    v
如果 task.f == NULL → 退出线程
    |
    v
执行 task.f(task.p)
    |
    v
循环等待下一次任务
*/

// 线程池结构体
struct workerpool {
    int nthreads;            // 工作线程数量
    zarray_t *tasks;         // 任务队列（存放 struct task）
    int taskspos;            // 当前任务执行位置索引

    pthread_t *threads;      // 工作线程数组
    int *status;             // （未使用，可扩展为状态记录）

    pthread_mutex_t mutex;   // 任务队列互斥锁
    pthread_cond_t startcond;// used to signal the availability of work    通知线程开始执行的条件变量
    bool start_predicate;    // predicate that prevents spurious wakeups on startcond 防止伪唤醒的布尔条件
    pthread_cond_t endcond;  // used to signal completion of all work   通知所有任务执行完成的条件变量

    int end_count;           // how many threads are done? 已完成任务的线程数
};

// 单个任务的结构体
struct task
{
    void (*f)(void *p);     // 任务函数指针
    void *p;                // 任务函数的参数
};

/* ============================================================
 * 工作线程函数，每个线程循环执行以下步骤：
 * 1. 等待有新任务
 * 2. 获取任务并执行
 * 3. 遇到空任务（f==NULL）时退出
 * ============================================================ */
void *worker_thread(void *p)
{
    workerpool_t *wp = (workerpool_t*) p;

    while (1) {
        struct task *task;

        pthread_mutex_lock(&wp->mutex);

         // 如果没有任务 或 start_predicate==false，则等待新任务
        while (wp->taskspos == zarray_size(wp->tasks) || !wp->start_predicate) {
            wp->end_count++;                                // 当前线程已完成，计数+1
            pthread_cond_broadcast(&wp->endcond);           // 通知主线程
            pthread_cond_wait(&wp->startcond, &wp->mutex);  // 等待任务信号
        }

        // 取出任务
        zarray_get_volatile(wp->tasks, wp->taskspos, &task);
        wp->taskspos++;
        pthread_mutex_unlock(&wp->mutex);
        sched_yield();                                      // 让出CPU

         // 若任务为空，表示要求线程退出
        // we've been asked to exit.
        if (task->f == NULL)
            return NULL;

        // 执行任务
        task->f(task->p);
    }

    return NULL;
}

/* ============================================================
 * 创建线程池
 * 参数：线程数 nthreads
 * 返回：新建的线程池指针
 * ============================================================ */
workerpool_t *workerpool_create(int nthreads)
{
    assert(nthreads > 0);

    workerpool_t *wp = calloc(1, sizeof(workerpool_t));
    wp->nthreads = nthreads;
    wp->tasks = zarray_create(sizeof(struct task));
    wp->start_predicate = false;

    // 多线程模式
    if (nthreads > 1) {
        wp->threads = calloc(wp->nthreads, sizeof(pthread_t));

        pthread_mutex_init(&wp->mutex, NULL);
        pthread_cond_init(&wp->startcond, NULL);
        pthread_cond_init(&wp->endcond, NULL);

        for (int i = 0; i < nthreads; i++) {
            int res = pthread_create(&wp->threads[i], NULL, worker_thread, wp);
            if (res != 0) {
                // debug_print("Insufficient system resources to create workerpool threads\n");
                // errno already set to EAGAIN by pthread_create() failure
                return NULL;
            }
        }

        // 等待所有线程进入ready状态
        // Wait for the worker threads to be ready
        pthread_mutex_lock(&wp->mutex);
        while (wp->end_count < wp->nthreads) {
            pthread_cond_wait(&wp->endcond, &wp->mutex);
        }
        pthread_mutex_unlock(&wp->mutex);
    }

    return wp;
}

/* ============================================================
 * 销毁线程池
 * - 向每个线程分配一个空任务（NULL）表示退出
 * - 等待所有线程退出
 * - 释放内存
 * ============================================================ */
void workerpool_destroy(workerpool_t *wp)
{
    if (wp == NULL)
        return;

    // force all worker threads to exit.
    if (wp->nthreads > 1) {
        
        // 给每个线程一个空任务，表示退出
        for (int i = 0; i < wp->nthreads; i++)
            workerpool_add_task(wp, NULL, NULL);
            
        // 唤醒所有线程
        pthread_mutex_lock(&wp->mutex);
        wp->start_predicate = true;
        pthread_cond_broadcast(&wp->startcond);
        pthread_mutex_unlock(&wp->mutex);

        // 等待所有线程退出
        for (int i = 0; i < wp->nthreads; i++)
            pthread_join(wp->threads[i], NULL);

        // 销毁同步原语
        pthread_mutex_destroy(&wp->mutex);
        pthread_cond_destroy(&wp->startcond);
        pthread_cond_destroy(&wp->endcond);
        free(wp->threads);
    }

    zarray_destroy(wp->tasks);
    free(wp);
}

/* ============================================================
 * 获取线程池的线程数
 * ============================================================ */
int workerpool_get_nthreads(workerpool_t *wp)
{
    return wp->nthreads;
}

/* ============================================================
 * 向线程池添加任务
 * 参数：
 *   f : 任务函数
 *   p : 参数
 * ============================================================ */
void workerpool_add_task(workerpool_t *wp, void (*f)(void *p), void *p)
{
    struct task t;
    t.f = f;
    t.p = p;

    if (wp->nthreads > 1) {
        pthread_mutex_lock(&wp->mutex);
        zarray_add(wp->tasks, &t);
        pthread_mutex_unlock(&wp->mutex);
    } else {
        // 单线程模式，无需锁
        zarray_add(wp->tasks, &t);
    }
}

/* ============================================================
 * 在单线程模式下执行任务
 * ============================================================ */
void workerpool_run_single(workerpool_t *wp)
{
    for (int i = 0; i < zarray_size(wp->tasks); i++) {
        struct task *task;
        zarray_get_volatile(wp->tasks, i, &task);
        task->f(task->p);
    }

    zarray_clear(wp->tasks);
}

/* ============================================================
 * 执行所有任务并等待完成（多线程模式）
 * runs all added tasks, waits for them to complete.
 * ============================================================ */
void workerpool_run(workerpool_t *wp)
{
    if (wp->nthreads > 1) {
        pthread_mutex_lock(&wp->mutex);
        wp->end_count = 0;                          // 重置完成计数
        wp->start_predicate = true;                 // 启动信号
        pthread_cond_broadcast(&wp->startcond);

        // 等待所有线程完成
        while (wp->end_count < wp->nthreads) {
//            printf("caught %d\n", wp->end_count);
            pthread_cond_wait(&wp->endcond, &wp->mutex);
        }

        wp->taskspos = 0;
        wp->start_predicate = false;
        pthread_mutex_unlock(&wp->mutex);

        // 清空任务队列
        zarray_clear(wp->tasks);

    } else {
        workerpool_run_single(wp);
    }
}

/* ============================================================
 * 获取系统可用CPU核心数
 * ============================================================ */
int workerpool_get_nprocs()
{
#ifdef _WIN32
    SYSTEM_INFO sysinfo;
    GetSystemInfo(&sysinfo);
    return sysinfo.dwNumberOfProcessors;
#else
    return sysconf (_SC_NPROCESSORS_ONLN);
#endif
}
