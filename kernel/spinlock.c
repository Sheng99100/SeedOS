// Mutual exclusion spin locks.

#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "riscv.h"
#include "proc.h"
#include "defs.h"

void
initlock(struct spinlock *lk, char *name)
{
  lk->name = name;
  lk->locked = 0;
  lk->cpu = 0;
}

// Acquire the lock.
// Loops (spins) until the lock is acquired.
void
acquire(struct spinlock *lk)
{
  // ----- 线程持有锁, 不能 yield CPU. (所以仅仅因为这个原因也不能开中断)
  // 原因是：
  // 如果线程主动 yield CPU，回到用户模式或 usertrap 内可被中断的部分，中断重新开启
  // 当前 CPU 和其他 CPU （所有 CPU）的线程可能都发生中断，于是进入内核
  // 并试图 acquire() 该锁, 进而所有 CPU 全部自旋
  // 再根据 (d.)
  // 所有 CPU 都在自旋状态，就没有 CPU 可以调度正持有锁的那个线程，让他释放锁了
  // ----- 线程持有锁，不能开中断（为了不内嵌acquire(). 附带的效果是不会被动 yield CPU）(a.)
  // 因为再次中断可能内核代码再次 trap，可能会再次试图 acquire() 然后自旋
  // 而多级的 trap ( 在 kerneltrap 内 )
  // 要想释放锁，就必须先返回其上级的 trap
  // 而每个线程在 kernel 内的 trap 从调度来说，实际上是函数调用
  // 要达到上级只能通过返回，哪怕放弃 CPU 在调度回来，也是切换回从最近一层 trap 的上下文
  // 无论内嵌中的 acquire() 是否释放 CPU
  // 都会导致内嵌的 acquire() 要获得锁，必须先返回外部，等其自己的外部释放锁
  // 而要返回外部，必须先使内嵌的 acquire() 释放锁. 导致死锁
  // 上述是针对持有锁的线程自身不能开中断.
  // 其他线程中断处理程序内的 acquire() 不影响持有锁的线程继续运行并释放锁。
  // ----- 线程等待锁时，也不能开中断（不会被动 yield CPU）(b.)
  // 这是原因 a 导致的
  // 因为持有锁就要关中断，而关中断指令不能在进入循环后才执行，必须在循环判断锁标记前关中断
  // 否则，循环中的原子交换指令和关中断指令之间存在一个可以被中断的窗口，可能导致死锁
  // 这就导致，线程等待锁时，也不能开中断
  // ----- 线程等待锁时，spinlock 的 acquire 本身采用自旋，不会主动 yield CPU （c.）
  // ----- 结合 b. c. acquire() 等待锁时会一直自旋，持续占用 CPU (d.)
  push_off(); // disable interrupts to avoid deadlock.
  if(holding(lk))
    // 如上述，因为线程已经持有锁后就不应该释放 CPU.
    // 即一个锁从持有到释放期间一定都只在同一个内 CPU 内执行.
    // 当前 CPU 在已经持有锁的前提下又调用 acquire()，则是不符合预期的。报错方便调试.
    panic("acquire");

  // On RISC-V, sync_lock_test_and_set turns into an atomic swap:
  //   a5 = 1
  //   s1 = &lk->locked
  //   amoswap.w.aq a5, a5, (s1)
  // 用原子交换实现的 test_and_set
  // 原子交换指令做的事情是读取原值，并设置新值. 这两个事情原子化.
  // 交换 a5 和地址 &lk->locked 的值. 返回交换后 a5 的值 (交换前 &lk->locked 的值) 
  // 如果返回 1，说明 &lk->locked 的值在交换前是 1, 交换和不变还是 1. 继续 spin
  // 如果返回 0. 说明交换前 &lk->locked 是 0. 现在已经交换成 1 了.
  // 思想是：
  // 不管锁是否已经被获取，都直接赋值 1，同时读取原值.
  // 如果锁已经被获取，那么赋值 1 也是不变的.
  // 如果锁还没有被获取，那么本来就该赋值 1.
  // 赋值后，再利用读取的原值作是否继续 spin 的判断
  while(__sync_lock_test_and_set(&lk->locked, 1) != 0)
      ;

  // Tell the C compiler and the processor to not move loads or stores
  // past this point, to ensure that the critical section's memory
  // references happen strictly after the lock is acquired.
  // On RISC-V, this emits a fence instruction.
  // 确保 lk->cpu = mycpu(); 不会被编译器或 CPU 重排到 while 之前 (获取锁之前)
  __sync_synchronize();

  // Record info about lock acquisition for holding() and debugging.
  // 用于调试
  lk->cpu = mycpu();
}

// Release the lock.
void
release(struct spinlock *lk)
{
  if(!holding(lk))
    panic("release");

  lk->cpu = 0;

  // Tell the C compiler and the CPU to not move loads or stores
  // past this point, to ensure that all the stores in the critical
  // section are visible to other CPUs before the lock is released,
  // and that loads in the critical section occur strictly before
  // the lock is released.
  // On RISC-V, this emits a fence instruction.
  __sync_synchronize();

  // Release the lock, equivalent to lk->locked = 0.
  // This code doesn't use a C assignment, since the C standard
  // implies that an assignment might be implemented with
  // multiple store instructions.
  // On RISC-V, sync_lock_release turns into an atomic swap:
  //   s1 = &lk->locked
  //   amoswap.w zero, zero, (s1)
  __sync_lock_release(&lk->locked);

  pop_off();
}

// Check whether this cpu is holding the lock.
// Interrupts must be off.
int
holding(struct spinlock *lk)
{
  int r;
  r = (lk->locked && lk->cpu == mycpu());
  return r;
}

// push_off/pop_off are like intr_off()/intr_on() except that they are matched:
// it takes two pop_off()s to undo two push_off()s.  Also, if interrupts
// are initially off, then push_off, pop_off leaves them off.
void
push_off(void)
{
  int old = intr_get();

  intr_off();
  if(mycpu()->noff == 0)
    mycpu()->intena = old;
  mycpu()->noff += 1;
}

void
pop_off(void)
{
  struct cpu *c = mycpu();
  // push_off 后会关中断，所以执行 pop_off 时应该是关中断状态
  if(intr_get())
    panic("pop_off - interruptible");
  if(c->noff < 1)
    panic("pop_off");
  c->noff -= 1;
  if(c->noff == 0 && c->intena)
    intr_on();
}
