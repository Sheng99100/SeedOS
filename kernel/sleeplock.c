// Sleeping locks

#include "types.h"
#include "riscv.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "proc.h"
#include "sleeplock.h"

void
initsleeplock(struct sleeplock *lk, char *name)
{
  initlock(&lk->lk, "sleep lock");
  lk->name = name;
  lk->locked = 0;
  lk->pid = 0;
}


  // 用了 acquire 以自旋的方式互斥，怎么能说利用放弃 CPU 来等待呢？
  // 这是因为，acquire 并不是锁 acquiresleep 外部的临界区
  // 只是锁 acquiresleep 内部对 sleep->locked 的并发访问
  // acquiresleep 要锁的外部的临界区还是用 sleep 来实现互斥
  // 获得 sleeplock->locked 的访问权（acquire获得锁），不代表当前没有线程持有 sleeplock
  // 仅仅只说明了当前没有线程正在 [访问] sleeplock->locked 而已
  // 1. spinlock 的 acquire 获得 sleeplock->locked 的访问权。考虑两种情况
  //   a. sleeplock 被持有，则 sleep；sleep() 释放 spinlock
  //   b. sleeplock 没被持有. 则 sleepacquire() 获得锁. 
  //      sleepacquire() 函数结束就能释放 spinlock 
  // 2. spinlock 的 acquire 没有获得 sleeplock->locked 的访问权
  //    根据 1. 可得，sleepacquire() 无论如何都会马上释放 spinlock
  //    所以也只需要 spin 一小会. 自旋之后能马上释放锁，又符合 1. 的情况  
void
acquiresleep(struct sleeplock *lk)
{ 
  acquire(&lk->lk);
  while (lk->locked) {
    sleep(lk, &lk->lk);
  }
  lk->locked = 1;
  lk->pid = myproc()->pid;
  release(&lk->lk);
}

// wakeup 在 sleeplock 上睡眠的所有线程
// sleeplock 的 sleep 以 sleeplock 本身作为 sleep、wakeup  的条件
void
releasesleep(struct sleeplock *lk)
{
  acquire(&lk->lk);
  lk->locked = 0;
  lk->pid = 0;
  wakeup(lk);
  release(&lk->lk);
}

int
holdingsleep(struct sleeplock *lk)
{
  int r;
  
  acquire(&lk->lk);
  r = lk->locked && (lk->pid == myproc()->pid);
  release(&lk->lk);
  return r;
}



