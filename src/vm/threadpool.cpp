/* -*- Mode: C++; c-basic-offset: 4; tab-width: 4; indent-tabs-mode: nil -*- */
/* vim: set ts=4 sw=4 et tw=99: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jscntxt.h"
#include "jslock.h"
#include "vm/threadpool.h"
#include "monitor.h"

#ifdef JS_THREADSAFE
#  include "prthread.h"
#endif

namespace js {

/****************************************************************************
 * ThreadPoolWorker
 *
 * Each |ThreadPoolWorker| just hangs around waiting for items to be added
 * to its |worklist_|.  Whenever something is added, it gets executed.
 * Once the worker's state is set to |TERMINATING|, the worker will
 * exit as soon as its queue is empty.
 */

#define WORKER_THREAD_STACK_SIZE (1*1024*1024)

enum WorkerState {
    CREATED, ACTIVE, TERMINATING, TERMINATED
};

class ThreadPoolWorker : public Monitor
{
    const size_t workerId_;
    ThreadPool *const threadPool_;

    /* Currrent point in the worker's lifecycle.
     *
     * Modified only while holding the ThreadPoolWorker's lock */
    WorkerState state_;

    /* Worklist for this thread.
     *
     * Modified only while holding the ThreadPoolWorker's lock */
    js::Vector<TaskExecutor*, 4, SystemAllocPolicy> worklist_;

    /* The thread's main function */
    static void ThreadMain(void *arg);
    void run();

public:
    ThreadPoolWorker(size_t workerId, ThreadPool *tp);
    ~ThreadPoolWorker();

    bool init();

    /* Invoked from main thread; signals worker to start */
    bool start();

    /* Submit work to be executed. If this returns true, you are
       guaranteed that the task will execute before the thread-pool
       terminates (barring an infinite loop in some prior task) */
    bool submit(TaskExecutor *task);

    /* Invoked from main thread; signals worker to terminate
     * and blocks until termination completes */
    void terminate();
};

ThreadPoolWorker::ThreadPoolWorker(size_t workerId, ThreadPool *tp)
    : workerId_(workerId), threadPool_(tp), state_(CREATED), worklist_()
{}

ThreadPoolWorker::~ThreadPoolWorker()
{}

bool
ThreadPoolWorker::init()
{
    return Monitor::init();
}

bool
ThreadPoolWorker::start()
{
#ifndef JS_THREADSAFE
    return false;
#else
    JS_ASSERT(state_ == CREATED);

    // Set state to active now, *before* the thread starts:
    state_ = ACTIVE;

    if (!PR_CreateThread(PR_USER_THREAD,
                         ThreadMain, this,
                         PR_PRIORITY_NORMAL, PR_LOCAL_THREAD,
                         PR_UNJOINABLE_THREAD,
                         WORKER_THREAD_STACK_SIZE))
    {
        // If the thread failed to start, call it TERMINATED.
        state_ = TERMINATED;
        return false;
    }

    return true;
#endif
}

void
ThreadPoolWorker::ThreadMain(void *arg)
{
    ThreadPoolWorker *thread = (ThreadPoolWorker*) arg;
    thread->run();
}

void
ThreadPoolWorker::run()
{
    // This is hokey in the extreme.  To compute the stack limit,
    // subtract the size of the stack from the address of a local
    // variable and give a 2k buffer.  Is there a better way?
    uintptr_t stackLimitOffset = WORKER_THREAD_STACK_SIZE - 2*1024;
    uintptr_t stackLimit = (((uintptr_t)&stackLimitOffset) +
                             stackLimitOffset * JS_STACK_GROWTH_DIRECTION);

    AutoLockMonitor lock(*this);

    for (;;) {
        while (!worklist_.empty()) {
            TaskExecutor *task = worklist_.popCopy();
            {
                // Unlock so that new things can be added to the
                // worklist while we are processing the current item:
                AutoUnlockMonitor unlock(*this);
                task->executeFromWorker(workerId_, stackLimit);
            }
        }

        if (state_ == TERMINATING)
            break;

        JS_ASSERT(state_ == ACTIVE);

        lock.wait();
    }

    JS_ASSERT(worklist_.empty() && state_ == TERMINATING);
    state_ = TERMINATED;
    lock.notify();
}

bool
ThreadPoolWorker::submit(TaskExecutor *task)
{
    AutoLockMonitor lock(*this);
    JS_ASSERT(state_ == ACTIVE);
    if (!worklist_.append(task))
        return false;
    lock.notify();
    return true;
}

void
ThreadPoolWorker::terminate()
{
    AutoLockMonitor lock(*this);

    if (state_ == CREATED) {
        state_ = TERMINATED;
        return;
    } else if (state_ == ACTIVE) {
        state_ = TERMINATING;
        lock.notify();
        while (state_ != TERMINATED) {
            lock.wait();
        }
    } else {
        JS_ASSERT(state_ == TERMINATED);
    }
}

/****************************************************************************
 * ThreadPool
 *
 * The |ThreadPool| starts up workers, submits work to them, and shuts
 * them down when requested.
 */

ThreadPool::ThreadPool(JSRuntime *rt)
    : runtime_(rt),
      nextId_(0)
{
}

ThreadPool::~ThreadPool() {
    terminateWorkers();
    while (workers_.length() > 0) {
        ThreadPoolWorker *worker = workers_.popCopy();
        js_delete(worker);
    }
}

bool
ThreadPool::init()
{
#ifdef JS_THREADSAFE
    // Compute desired number of workers based on env var or # of CPUs.
    size_t numWorkers = 0;
    char *pathreads = getenv("PATHREADS");
    if (pathreads != NULL) {
        numWorkers = strtol(pathreads, NULL, 10);
    } else {
        numWorkers = GetCPUCount() - 1;
    }

    // Allocate workers array and then start the worker threads.
    // Ensure that the field numWorkers_ always tracks the number of
    // *successfully initialized* workers.
    for (size_t workerId = 0; workerId < numWorkers; workerId++) {
        ThreadPoolWorker *worker = js_new<ThreadPoolWorker>(workerId, this);
        if (!worker->init()) {
            js_delete(worker);
            return false;
        }
        if (!workers_.append(worker)) {
            js_delete(worker);
            return false;
        }
        if (!worker->start()) {
            return false;
        }
    }
#endif

    return true;
}

void
ThreadPool::terminateWorkers()
{
    for (size_t i = 0; i < workers_.length(); i++) {
        workers_[i]->terminate();
    }
}

bool
ThreadPool::submitOne(TaskExecutor *executor) {
    runtime_->assertValidThread();

    if (numWorkers() == 0)
        return false;

    // Find next worker in round-robin fashion.
    size_t id = JS_ATOMIC_INCREMENT(&nextId_) % workers_.length();
    return workers_[id]->submit(executor);
}

bool
ThreadPool::submitAll(TaskExecutor *executor) {
    for (size_t id = 0; id < workers_.length(); id++) {
        if (!workers_[id]->submit(executor))
            return false;
    }
    return true;
}

bool
ThreadPool::terminate() {
    terminateWorkers();
    return true;
}

}
