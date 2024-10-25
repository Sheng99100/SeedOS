#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "spinlock.h"
#include "proc.h"
#include "defs.h"

struct cpu cpus[NCPU];

struct proc proc[NPROC];

struct proc *initproc;

int nextpid = 1;
struct spinlock pid_lock;

extern void forkret(void);
static void freeproc(struct proc *p);

extern char trampoline[]; // trampoline.S

// helps ensure that wakeups of wait()ing
// parents are not lost. helps obey the
// memory model when using p->parent.
// must be acquired before any p->lock.
struct spinlock wait_lock;

// Allocate a page for each process's kernel stack.
// Map it high in memory, followed by an invalid guard page.
//
// kvminit() -> kvmmake() -> proc_mapstacks()
// 
// xv6 进程的数量固定, 在进程创建前就创建好固定的进程结构体. alloc进程时候再分配 trapframe 等内存
// 但是每个进程的内核栈是在初始化内核页表的时候分配的
// 并映射到内核空间的高地址
// 
// main() -> procinit() 会把内核栈在内核的虚拟地址绑定给各个进程的 p->kstack
// 
// 1. p->kstack: 内核栈的低地址, 即在内核栈在页表中映射的虚拟地址
// 2. p->trapframe->kernel_sp: 当前栈帧的高地址的指针(栈从高到低生长)
//    trap 进内核时 trampoline.S 会载入 p->trapframe->kernel_sp 到 sp 寄存器
// 3. p->trapframe->sp: 用户栈的当前栈帧的高地址的指针
// 4. p->context.sp 
// 5. cpu->context.sp
// 
// 内核栈是在内核态下，用户进程 trap 到执行 kernel 的 C 代码时，该进程用的调用栈 
// 分配每个进程的内核栈(xv6中仅一个页大小)，每个内核栈都映射到内核空间的高地址处
// 内核栈在内核空间的虚拟地址记录在 proc 结构体和其中的 trapframe 中
// 从用户态的任何情况（interrupt、execption、system call）trap 进内核（stvec设置为内核入口向量）
// trampoline.S 会载入 trapframe->kernel_sp 到 sp 寄存器
// 从内核态中断，那 sp 就是原来的内核栈了
// 每个进程的 kstack 都跟随一个有效位为 0 的 "guard page"
// 目的是，在栈溢出时，利用触发 page fault exception 来处理
void
proc_mapstacks(pagetable_t kpgtbl)
{
  struct proc *p;
  
  for(p = proc; p < &proc[NPROC]; p++) {
    // 分配每个进程的内核栈的物理内存区域(由内核自己分配, 而不是由 C 分配)
    char *pa = kalloc();
    if(pa == 0)
      panic("kalloc");
    // 使用内核栈的时候已经切换到内核空间的页表了
    // 将每个内核栈的虚拟地址都映射到内核的地址空间的高位
    uint64 va = KSTACK((int) (p - proc));
    kvmmap(kpgtbl, va, (uint64)pa, PGSIZE, PTE_R | PTE_W);
  }
}

// initialize the proc table.
// 再初始化内核页表时, 就分配了内核栈的内存, 并映射到了内核空间的高地址
// main() -> kvminit() -> kvmmake() -> proc_mapstacks()
// main() -> procinit() 初始化进程结构体时
// 现在把内核栈在内核中的虚拟地址绑定到各个进程结构体的 p->kstack 上
void
procinit(void)
{
  struct proc *p;
  
  initlock(&pid_lock, "nextpid");
  initlock(&wait_lock, "wait_lock");
  for(p = proc; p < &proc[NPROC]; p++) {
      initlock(&p->lock, "proc");
      p->state = UNUSED;
      // 每个内核栈物理空间由 kernel 自己分配
      // 但是每个内核栈的虚拟地址都已经映射在内核空间的高地址处
      // 现在把内核栈在内核中的虚拟地址绑定到各个进程结构体中
      p->kstack = KSTACK((int) (p - proc));
  }
}

// Must be called with interrupts disabled,
// to prevent race with process being moved to a different CPU.
int
cpuid()
{
  int id = r_tp();
  return id;
}

// Return this CPU's cpu struct.
// Interrupts must be disabled.
struct cpu*
mycpu(void)
{
  int id = cpuid();
  struct cpu *c = &cpus[id];
  return c;
}

// Return the current struct proc *, or zero if none.
struct proc*
myproc(void)
{
  push_off();
  struct cpu *c = mycpu();
  struct proc *p = c->proc;
  pop_off();
  return p;
}

int
allocpid()
{
  int pid;
  
  acquire(&pid_lock);
  pid = nextpid;
  nextpid = nextpid + 1;
  release(&pid_lock);

  return pid;
}

// Look in the process table for an UNUSED proc.
// If found, initialize state required to run in the kernel,
// and return with p->lock held.
// If there are no free procs, or a memory allocation fails, return 0.
// 进程结构体作为 kernel 程序的静态变量
// 激活（使进程state改为USED）一个进程
// 并为它分配作页表的物理页、trampframe
// 关键是：通过设置 alloc 的 proc 的 context -> ra = forkret
// 新配置出的进程第一次被 scheduler() 切换到其 context 后就会根据 ra 返回到 forkret()
// 而 forkret() 会 call usertrapret()，usertrapret（） 再 call trampline.S 的 userret
// scheduler() -> swtch() -> forkret() ->  usertrapret() -> userret(trampline.S)
static struct proc*
allocproc(void)
{
  struct proc *p;

  for(p = proc; p < &proc[NPROC]; p++) {
    acquire(&p->lock);
    if(p->state == UNUSED) {
      goto found;
    } else {
      release(&p->lock);
    }
  }
  return 0;

found:
  p->pid = allocpid();
  p->state = USED;

  // Allocate a trapframe page.
  if((p->trapframe = (struct trapframe *)kalloc()) == 0){
    freeproc(p);
    release(&p->lock);
    return 0;
  }

  // An empty user page table.
  p->pagetable = proc_pagetable(p);
  if(p->pagetable == 0){
    freeproc(p);
    release(&p->lock);
    return 0;
  }

  // Set up new context to start executing at forkret,
  // which returns to user space.
  memset(&p->context, 0, sizeof(p->context));
  p->context.ra = (uint64)forkret;

  // p->kstack 在 procinit() 被写为内核栈在内核页表的低地址
  // p->kstack 是整个内核栈在内核空间的低地址
  // (内核栈对齐到一个虚拟页, 所以内核栈的低地址等于某一个页表项的 VPN )
  // 加上一个 PGSIZE 转变为高地址(约定栈从高地址向低地址生长)
  p->context.sp = p->kstack + PGSIZE;

  return p;
}

// free a proc structure and the data hanging from it,
// including user pages.
// p->lock must be held.
static void
freeproc(struct proc *p)
{
  if(p->trapframe)
    kfree((void*)p->trapframe);
  p->trapframe = 0;
  if(p->pagetable)
    proc_freepagetable(p->pagetable, p->sz);
  p->pagetable = 0;
  p->sz = 0;
  p->pid = 0;
  p->parent = 0;
  p->name[0] = 0;
  p->chan = 0;
  p->killed = 0;
  p->xstate = 0;
  p->state = UNUSED;
}

// Create a user page table for a given process, with no user memory,
// but with trampoline and trapframe pages.
pagetable_t
proc_pagetable(struct proc *p)
{
  pagetable_t pagetable;

  // An empty page table.
  pagetable = uvmcreate();
  if(pagetable == 0)
    return 0;

  // map the trampoline code (for system call return)
  // at the highest user virtual address.
  // only the supervisor uses it, on the way
  // to/from user space, so not PTE_U.
  if(mappages(pagetable, TRAMPOLINE, PGSIZE,
              (uint64)trampoline, PTE_R | PTE_X) < 0){
    uvmfree(pagetable, 0);
    return 0;
  }

  // map the trapframe page just below the trampoline page, for
  // trampoline.S.
  if(mappages(pagetable, TRAPFRAME, PGSIZE,
              (uint64)(p->trapframe), PTE_R | PTE_W) < 0){
    uvmunmap(pagetable, TRAMPOLINE, 1, 0);
    uvmfree(pagetable, 0);
    return 0;
  }

  return pagetable;
}

// Free a process's page table, and free the
// physical memory it refers to.
void
proc_freepagetable(pagetable_t pagetable, uint64 sz)
{
  uvmunmap(pagetable, TRAMPOLINE, 1, 0);
  uvmunmap(pagetable, TRAPFRAME, 1, 0);
  uvmfree(pagetable, sz);
}

// a user program that calls exec("/init")
// assembled from ../user/initcode.S
// od -t xC ../user/initcode
uchar initcode[] = {
  0x17, 0x05, 0x00, 0x00, 0x13, 0x05, 0x45, 0x02,
  0x97, 0x05, 0x00, 0x00, 0x93, 0x85, 0x35, 0x02,
  0x93, 0x08, 0x70, 0x00, 0x73, 0x00, 0x00, 0x00,
  0x93, 0x08, 0x20, 0x00, 0x73, 0x00, 0x00, 0x00,
  0xef, 0xf0, 0x9f, 0xff, 0x2f, 0x69, 0x6e, 0x69,
  0x74, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00
};

// Set up first user process.
void
userinit(void)
{
  struct proc *p;

  // 为初始进程获取一个进程结构体，同时部分填充这个结构体
  p = allocproc();
  initproc = p;
  
  // allocate one user page and copy initcode's instructions
  // and data into it.
  // 分配一个物理页, 并映射到该初始进程的地址空间的 0 地址处
  // 把 initcode.S 的代码复制到该进程的物理页
  uvmfirst(p->pagetable, initcode, sizeof(initcode));
  p->sz = PGSIZE;

  // prepare for the very first "return" from kernel to user.
  p->trapframe->epc = 0;      // user program counter
  p->trapframe->sp = PGSIZE;  // user stack pointer
 
  safestrcpy(p->name, "initcode", sizeof(p->name));
  p->cwd = namei("/");

  p->state = RUNNABLE;

  release(&p->lock);
}

// Grow or shrink user memory by n bytes.
// Return 0 on success, -1 on failure.
// 增长进程能使用的虚拟地址
int
growproc(int n)
{
  uint64 sz;
  struct proc *p = myproc();

  sz = p->sz;
  if(n > 0){
    if((sz = uvmalloc(p->pagetable, sz, sz + n, PTE_W)) == 0) {
      return -1;
    }
  } else if(n < 0){
    sz = uvmdealloc(p->pagetable, sz, sz + n);
  }
  p->sz = sz;
  return 0;
}

// Create a new process, copying the parent.
// Sets up child kernel stack to return as if from fork() system call.
int
fork(void)
{
  int i, pid;
  struct proc *np;
  struct proc *p = myproc();

  // Allocate process.
  // 激活（使进程state改为USED）一个进程
  // 并为它分配物理页作页表、trampframe等
  if((np = allocproc()) == 0){
    return -1;
  }

  // Copy user memory from parent to child.
  // 复制页表数据(除物理地址)和物理页的数据, 到新进程的页表和物理页
  if(uvmcopy(p->pagetable, np->pagetable, p->sz) < 0){
    freeproc(np);
    release(&np->lock);
    return -1;
  }
  np->sz = p->sz;

  // copy saved user registers.
  *(np->trapframe) = *(p->trapframe);

  // Cause fork to return 0 in the child.
  // fork是系统调用, fork出的新进程复制原进程数据
  // 所以原进程和新进程都会从 usertrapret 返回
  // 但是 fork 系统调用约定，子进程的 fork() 返回值为 0. 
  // 而返回值先存在 trapframe 中，在 trampline.S 的 userret 中载入到 a0 寄存器
  // 所以有下面的 np->trapframe->a0 = 0;
  // 为什么 a0 放在内存的trapframe, 而不是直接放在 a0 ? 
  // 因为 a0 在kernel的函数调用之间也要作为返回值寄存器使用，所以 trap 入 kernel 后先存在 trapframe
  np->trapframe->a0 = 0;

  // increment reference counts on open file descriptors.
  for(i = 0; i < NOFILE; i++)
    if(p->ofile[i])
      np->ofile[i] = filedup(p->ofile[i]);
  np->cwd = idup(p->cwd);

  safestrcpy(np->name, p->name, sizeof(p->name));

  pid = np->pid;

  release(&np->lock);

  acquire(&wait_lock);
  np->parent = p;
  release(&wait_lock);

  acquire(&np->lock);
  np->state = RUNNABLE;
  release(&np->lock);

  return pid;
}

// Pass p's abandoned children to init.
// Caller must hold wait_lock.
void
reparent(struct proc *p)
{
  struct proc *pp;

  for(pp = proc; pp < &proc[NPROC]; pp++){
    if(pp->parent == p){
      pp->parent = initproc;
      wakeup(initproc);
    }
  }
}

// Exit the current process.  Does not return.
// An exited process remains in the zombie state
// until its parent calls wait().
void
exit(int status)
{
  struct proc *p = myproc();

  if(p == initproc)
    panic("init exiting");

  // Close all open files.
  for(int fd = 0; fd < NOFILE; fd++){
    if(p->ofile[fd]){
      struct file *f = p->ofile[fd];
      fileclose(f);
      p->ofile[fd] = 0;
    }
  }

  begin_op();
  iput(p->cwd);
  end_op();
  p->cwd = 0;

  acquire(&wait_lock);

  // Give any children to init.
  reparent(p);

  // Parent might be sleeping in wait().
  wakeup(p->parent);

  acquire(&p->lock);

  p->xstate = status;
  p->state = ZOMBIE;

  release(&wait_lock);

  // Jump into the scheduler, never to return.
  sched();
  panic("zombie exit");
}

// Wait for a child process to exit and return its pid.
// Return -1 if this process has no children.
int
wait(uint64 addr)
{
  struct proc *pp;
  int havekids, pid;
  struct proc *p = myproc();

  // 保证 wait() 和 exit() 中相关片段(包含 wakeup())之间的原子性
  // 为避免执行顺序是 wait() -> exit()
  // 但 wakeup() 和 ZOMBIE 进程状态等设置在 sleep() 之前执行
  // 导致父进程的 wait() 在 sleep() 后不被唤醒
  acquire(&wait_lock);

  for(;;){
    // Scan through table looking for exited children.
    havekids = 0;
    for(pp = proc; pp < &proc[NPROC]; pp++){
      if(pp->parent == p){
        // make sure the child isn't still in exit() or swtch().
        // 
        acquire(&pp->lock);
        havekids = 1;
        if(pp->state == ZOMBIE){
          // Found one.
          pid = pp->pid;
          if(addr != 0 && copyout(p->pagetable, addr, (char *)&pp->xstate,
                                  sizeof(pp->xstate)) < 0) {
            release(&pp->lock);
            release(&wait_lock);
            return -1;
          }
          freeproc(pp);
          release(&pp->lock);
          release(&wait_lock);
          return pid;
        }
        release(&pp->lock);
      }
    }

    // No point waiting if we don't have any children.
    if(!havekids || killed(p)){
      release(&wait_lock);
      return -1;
    }
    
    // Wait for a child to exit.
    sleep(p, &wait_lock);  //DOC: wait-sleep
  }
}

// Per-CPU process scheduler.
// Each CPU calls scheduler() after setting itself up.
// Scheduler never returns.  It loops, doing:
//  - choose a process to run.
//  - swtch to start running that process.
//  - eventually that process transfers control
//    via swtch back to the scheduler.
// 
//  思考线程切换时应该采用的几个直觉是：
//    线程切换机制的核心, 也就是 "切换"， 不在乎线程在切换前是处于什么线程、是否在中断处理程序
//    对于切换后会产生影响的情况，可以由切换逻辑之外负责
//    而切换机制的核心本身应该独立、不依赖切换前是否是处于中断处理程序之类
//    思考时应该区分到切换本身和切换带来的额外影响
//    思考切换本身时，应该呈现出线程重新被调度时，一切照常的从切换函数之后返回，就好像切换没有发生的假象
//，
//    xv6 的上下文切换内嵌在中断过程中
//    1. 中断（或主动sleep）发生，执行中断处理程序 uservec -> usertrap
//    2. yield() -> sched() -> swtch()
//    3. 从 swtch 返回。之后一切按照从中断返回的过程执行
//    重点是，整个过程如果忽视 swtch() 之后发生的一切（包括发生了被 scheduler 加载上下文数据然后写进寄存器的事情）
//    从函数调用歌返回的过程看，这个过程仅仅只是一个从用户模式进入内核，调用了一个函数
//    最后从这个函数普通的返回，最后回到用户模式的过程
//    就好像这个 swtch 函数什么都没做一样
//    切换关注做好从函数正确的 [照常] 返回的抽象即可，其余的影响就留给切换之外的来关注
// 
//  调度除了保存/恢复寄存器 sp、s0-s11 
//  线程切换前的以下状态也是需要恢复的
//    a. 中断状态
//    b. 持有和等待的锁，也是应该恢复的 “状态”
//  和锁相关的 “状态” 是隐含的，不需要特别保存。另外，自旋锁持有时不应该放弃 CPU，但这是切换机制本身之外的事情。
//  
// risc-v 对中断处理的硬件过程给予了中断后关中断，sret 后根据状态位设置中断的，恢复机制
// 对于中断程序跳转、返回的中断状态保存恢复，是利用了硬件过程
// （但是这些机制不一定只用于中断，能否把这些抽象利用到其地方呢？）
// 
// 上下文切换后，原先的 SSTATUS 寄存器会被覆盖
// 讨论的基础是：
//  a. 两个内核线程，执行 scheduler 的 CPU 线程; 跨线程的 acquire(p->lock) 和 release(p->lock)
//  b. push_off, pop_off；以及它们保存的初始中断状态是内核空间中 C 的全局变量，跨线程共享
//     因为无论 pop_off 后处于哪个线程，锁释放干净就可以允许开中断。 
//  c. 切换前确保 [仅] 持有 p->lock，即在一个push后处于关中断，切换后在切换到的新线程再释放，恢复中断状态
//  d. 切换前除了 p->lock 不持有其他任何锁。也就是保证切换到内核线程（即非scheduler线程）前
//     之前线程持有的所有锁都已经释放干净
// 考虑这个例子
// 切换前内核线程：usertrap 处理 syscall，调用 sleep，sleep 及获得锁前处于开中断。
// 切换后内核线程：正常被定时器中断，yield 前在 kerneltrap 处于关中断。理想的被调度后应是关中断。
// 切换前，push 前的初始中断状态：开
// 中断状态的转变过程是
// [关（sleep外获得锁）、关（sleep内acquire(p->lock)），开（sleep释放条件锁），恢复（scheduler）]
// [关（scheduler内acquire另一个p->lock），恢复（另一内核线程内释放 acquire）]
// 最后，在切换后的线程里，中断状态却被恢复成了中断前的中断状态。在这个例子里是开，而不是理想的关。
// 
// 有没有在现有的、能用的这些机制的基础上的，可以恢复切换前中断状态的方法 ？
// 首先，切换前后一定有对于进程锁的 [push_off 关、pop_off 恢复] 中断的过程
// 在这个基础上，"恢复切换前的中断状态" 就是 “恢复切换前且 push_off 前的的状态”
// 于是 "恢复中断状态"，就可以转变为 “恢复 cpu->intren”
// 于是可以这么解决
// 在 push_off 后保存初始状态到线程局部的栈帧中
// 在 pop_off 前从局部栈帧中恢复到全局的 cpu->intren 变量中
//
// 另外，思考设计上下文切换时，脑海中的逻辑上下文可以尝试以 CPU 为中心
// 内核线程切换到其他 CPU 和当前 CPU 加载其他内核线程是一样的 
// 例如
// 关中断的线程切换到其他 cpu->intren 为开的 CPU，和
// cpu->intren 为开的当前 CPU 加载其他切换前关中断的线程，是一样的
void
scheduler(void)
{
  struct proc *p;
  struct cpu *c = mycpu();

  c->proc = 0;
  for(;;){
    // The most recent process to run may have had interrupts
    // turned off; enable them to avoid a deadlock if all
    // processes are waiting.
    // 为了等待 IO 操作而 sleep 的线程，wakeup() 它的代码只会在中断handler中执行
    // 如果所有其他线程也都在 sleep，那就没有线程可以运行并有机会重新开中断，去响应 IO 中断。
    // 导致包括等待 IO 操作而 sleep 的线程以及所有其他线程都在 SLEEP 状态
    // 所以，应该让 scheduler 内部也可以响应中断。
    // 
    // 为什么 scheduler 内用的是 CPU stack，却允许它执行中断 handler ？
    // 1. 因为中断机制作为异步通讯双方的桥梁，和其他线程之间在逻辑上可以是独立的
    //    中断程序根本无所谓自己在哪一个线程被触发，然后在那个线程栈上处理（除非被刻意设计在特定线程环境执行）
    //    例如某用户线程发送读磁盘 syscall
    //    而 IO 完成的中断完全可以在执行另一个线程时被触发，然后进入处理
    //    中断发生的时间不确定，而且其他线程不需要立即使用中断执行的结果
    //    且每次中断和中断之间也是独立的，每次中断，都不依赖上次中断的上下文
    //    
    //    中断处理程序的功能无所谓触发中断时在什么地方，无所谓在哪个线程.
    //    所以从处理程序代码功能来说，不需要 “回到中断之前的地方”. 
    //    除非两次中断利用同一个栈来保存和恢复。 
    //    
    //    每一次中断处理的过程，它们之间的局部状态变量都是独立的，不存在依赖关系，不需要恢复
    //    不用因为上一次中断用的是 kstack，这一次就也用 kstack
    //    例如，在 scheduler 内发生中断，和之前的中断用的可以不是同一个栈
    //    甚至因为两次中断用了不同的栈，之前的中断甚至可以被另一个 CPU 调度同时运行，只要管理好全局变量的并发即可.
    //    当然，若不同中断在同一个线程发送，已经用了同一个栈，像 usertrap 和 kerneltrap 两个中断
    //    就不能同时运行，避免并发读写同一个栈
    //   
    //    虽然中断过程间没有依赖关系，不需要依赖返回值等
    //    但通常对当前执行线程内发生的中断，都使用同一个栈，直到返回用户线程。
    //    用户线程发生的中断，都用对应的内核栈
    //    schedler 线程（CPU 线程）发生的中断，都用 CPU stack，最后返回 scheduler
    //    这是因为，中断发生时它总是处于某一个线程
    //    简单的做法是让中断过程执行完后，返回到中断发生线程的，发生中断前的原位置
    //    使用和中断发生线程相同的栈，利用这个栈进行上下文恢复
    //    类似函数调用时，在栈帧中进行的调用前保存和调用后恢复
    //    但是中断的硬件过程增加了 sret 等恢复中断前状态，模式的机制 
    // 
    //    而对于多个中断
    //    如果最早是从用户模式开始的，最后栈帧返回到 usertrap 
    //    usertrap 最后调用 usertrapret，之后有两个栈帧 
    //    而 usertrapret 进入 userret, userret 回到用户线程前
    //    会直接放弃这个用户线程对应的内核线程的栈上的这两个栈帧
    //    即直接切换 sp 为用户栈，而不保存当前内核栈的 sp
    //    下次从用户模式 trap，又从全新的栈顶开始（从该线程的 trapframe 加载）
    //    这是因为每次中断各自独立，且要切换栈，不再需要在当前内核线程的栈上来保存和恢复了
    //    如果最早从 scheduler 开始，则最后的栈会返回到 scheduler ，然后继续循环
    //   
    //    是否可以让中断处理都在单独的线程，单独的栈运行？
    //    中断处理程序只负责分发中断消息，中断处理线程去消费
    //    让中断发生的所在线程和处理程序异步进行
    // 
    // 2. 如果 scheduler 内发生时钟中断外的中断，进入 kerneltrap 中发生中断
    //    sched 里调用 swtch 时，岂不是把 c->context 存到了 p->context，然后自己覆盖自己 ?
    //    kerneltrap 是全程关中断的，不用担心时钟中断，唯一有可能导致 swtch 的是 sleep，wait 
    //    外中断已经是驱动的 buttom-half 了，不存在如输入缓冲区空或输出缓冲区满需要 sleep 的情况
    // 
    // 3. 如果在 scheduler 内发生的是时钟中断呢？
    //    sched 里调用 swtch 时，岂不是把 c->context 存到了 p->context，然后自己覆盖自己 ?
    // 
    // 几种进入 scheduler 时中断的开关情况：
    // 进入 scheduler 前是什么状态，进入之后就是什么状态
    // p->lock 的 acquire() release() 调用的 push_off, pop_off 正好一对，恢复持有 p->lock 前的状态
    // 另外，获得 p->lock 前，如果持有其他锁，为了避免死锁是不会放弃 CPU 的
    // 1. usertrap (处理syscall) 时发生的时钟中断，从 kernelvec 进的 scheduler，是关中断的
    // 2. usermode 发生的时钟中断，从 usertrap 进的 scheduler，是关中断的
    // 3. usertrap (处理syscall) 时，发生的 sleep，进入 scheduler，release 后恢复开中断
    //    sleep 前是开中断，usertrap 处理 syscall 允许开中断
    //    sleep 前持有数据结构锁，sleep() 内获得 p->lock, 释放数据锁，进入 scheduler，release 后
    //    释放 p->lock。两对 push_off, pop_off 后恢复 sleep 前的开中断状态。
    // 
    // 
    intr_on();
    int found = 0;
    for(p = proc; p < &proc[NPROC]; p++) {
      acquire(&p->lock);
      if(p->state == RUNNABLE) {
        // Switch to chosen process.  It is the process's job
        // to release its lock and then reacquire it
        // before jumping back to us.
        p->state = RUNNING;
        c->proc = p; // 更新当前进程
        swtch(&c->context, &p->context);

        // fork 新进程首次运行 forkret()->release() 或 yield()->release() 或 sleep() -> release()   
        // running...
        // yield()->acquire() 或 sleep()->acquire()

        // Process is done running for now.
        // It should have changed its p->state before coming back.
        c->proc = 0;
        found = 1;
      }
      release(&p->lock);
    }
    if(found == 0) {
      // nothing to run; stop running on this core until an interrupt.
      intr_on();
      // （Wait for Interrupt）处理器进入低功耗状态，直到发生中断或其他事件来唤醒处理器
      asm volatile("wfi");
    }
  }
}

// Switch to scheduler.  Must hold only p->lock
// and have changed proc->state. Saves and restores
// intena because intena is a property of this
// kernel thread, not this CPU. It should
// be proc->intena and proc->noff, but that would
// break in the few places where a lock is held but
// there's no process.
void
sched(void)
{
  int intena;
  struct proc *p = myproc();

  if(!holding(&p->lock)) // 上下文切换时不能被打乱，否则可能保存到一半就被跳转到kernelvec->kerneltrap，没保存的寄存器就被覆盖
    panic("sched p->lock");
  if(mycpu()->noff != 1) // 二次验证在执行 sched() 之前，已经获得了 p->lock
    panic("sched locks");
  if(p->state == RUNNING)
    panic("sched running");
  if(intr_get()) // 上下文切换时不能被打乱，否则可能保存到一半就被跳转到kernelvec->kerneltrap，没保存的寄存器就被覆盖
    panic("sched interruptible");

  intena = mycpu()->intena;
  swtch(&p->context, &mycpu()->context);
  mycpu()->intena = intena;
}

// Give up the CPU for one scheduling round.
void
yield(void)
{
  struct proc *p = myproc();
  acquire(&p->lock);
  p->state = RUNNABLE; // 从 Running 改为 Runnable
  sched();
  release(&p->lock);
}

// A fork child's very first scheduling by scheduler()
// will swtch to forkret.
void
forkret(void)
{
  static int first = 1;

  // Still holding p->lock from scheduler.
  release(&myproc()->lock);

  if (first) {
    // File system initialization must be run in the context of a
    // regular process (e.g., because it calls sleep), and thus cannot
    // be run from main().
    fsinit(ROOTDEV);

    first = 0;
    // ensure other cores see first=0.
    __sync_synchronize();
  }

  usertrapret();
}

// Atomically release lock and sleep on chan.
// Reacquires lock when awakened.
// 当一个线程需要某个条件满足后再继续时，希望让该线程让出 CPU 待条件被满足后再继续
// 这个“条件”通常在某个数据结构上的. 指针 chan 解释为具体的条件的唯一标识. 一个数据结构可以有多个在其上的不同具体条件 chan.
// 所以在 sleep 后，需要释放掉该数据结构的锁 lk (称为"条件锁")，才可能让条件有机会被满足
// 其它线程获得数据结构锁，修改有关数据结构使条件满足后，调用 wakeup 修改所有正在 SLEEPING 状态的，和该条件 chan 有关的线程状态为 RUNNABLE
// 第一个被唤醒重新执行的线程，从 sleep 的 sched 后重新执行
// 被唤醒后可能还要用数据结构，不希望并发读写，所以要重新获得锁
// 非首个被唤醒的线程，因为数据结构又可能被首个被唤醒的线程修改过(唤醒只是可能满足)，所以要再次检查
// 
// 要做到上述的抽象目的而不出错，充分条件包含同时做到两个事情：
// 1. 在 sleep 后，需要释放掉该数据结构的锁（"后" 具体是什么时机？）
// 2. 这段指令序列，涉及到两个不同原因的时序要求：
// 
// 首先，是 wakeup() 调用时间(位置)的要求
// 即保证 “wakeup 唤醒 sleep 的线程” (描述还是模糊的) 这个时序关系了 
// 如果 sleep 的 caller 线程满足条件，那么 wakeup 对该线程无意义
// 如果 sleep 的 caller 线程不满足条件
// 那么对于 wakeup 的 caller 线程，在使条件满足后，wakeup 的调用时间 
// 如果在 sleep 的 caller 线程检查条件前. 它的逻辑本身就在 sleep 的线程之前发生，所以无所谓是否发生 wakeup 丢失
// 如果在 sleep 的 caller 线程检查条件为不满足之后（同时意味着访问完数据结构了），wakeup 线程使条件满足
// 那么就应该保证 “wakeup 唤醒 sleep 的线程” 这个时序关系了 
// 如果在修改状态为 SLEEPING 前释放锁（无论在 sleep 内外释放）
// 意味着允许 sleep 线程和 wakeup 线程的指令穿插
// wakeup 修改为 RUNNABLE 后可能被 sleep 覆盖为 SLEEPING. 
// 就不满足该时序关系，意味着 sleep 的线程可能无法再被调度
//  所以 ---- wakeup 不应该在 sleep 设置状态为 SLEEPING 之前调用
// 
// 另外，是 sleep 的线程 sleep() -> sched() -> swtch() 的过程
// 和其他 CPU 的 scheduler() 之间需要互斥、按序的并发问题:
// wakeup 设置线程状态为 RUNNABLE 后，该进程结构就可以被 scheduler 调度
// 除了在 sleep 线程所在的 CPU 调用 sched 后，切换到的 scheduler
// 还可以是任意其他 CPU 的 scheduler
// 如果在设置 SLEEPING 后 sched 前，wakeup 修改完 RUNNABLE
// 就可能在 sleep 所在 CPU 还没切换到 scheduler 前，还在使用本线程的内核 sp 时
// 其他 CPU 也同时使用这个线程的上下文
// 两个 CPU 同时使用同一个栈，导致并发修改
// 所以 ---- wakeup 应该在 sleep 彻底切换上下文后使用；在这具体是在切换到 sheduler 线程后调用
// 
// 结合上述的几个要求
// a. 需要释放条件锁
// b. 两个时序关系要求
// 就要考虑具体的手段 
// 要用几个锁、什么地方分别用什么锁来完成？
//
// 一开始的想法是（显然是只满足第一个、不满足第二个时序要求的）
// 把条件锁 lk （需要释放的）顺带作为保证 sleep 和 wakeup 时序的锁的做法
// 条件锁 lk 已经用来保证数据结构发访问互斥
// 即 wakeup 线程和 sleep 访问同一个数据结构，用同一个锁
// 这个锁可以再用来作为 sleep 和 wakeup 之间的互斥，即
// 在 sleep() 前获得锁，锁传入 sleep，执行 p->state = SLEEPING 后释放
// 在 wakeup() 前获得锁，锁传入 wakeup, wakeup 完成后释放
// 可以看出，在条件锁必须在 sleep() 内释放的前提下
// 把条件锁 lk 顺带作为保证 sleep 和 wakeup 时序的锁的做法
// 必须在 p->state = SLEEPING 后释放条件锁 lk，不过也只能满足第二个时序要求
// 
// 最小改动当前具备前提、代码的做法是用 p->lock
// 因为 sleep 和 wakeup 的调用时序是要跨越线程的
// 在 sched() -> swtch() 前获得锁
// 在 sched() -> swtch() 后，进入 scheduler() 就可以释放 
// 所以 p->lock 是符合的
// 在 sleep() 一开始就获得 p->lock，进入 scheduler() 释放 
// 就同时满足两个时序关系
// 而且这样一来，在获取 p->lock 之后就可以马上放心的释放条件锁
// 
void
sleep(void *chan, struct spinlock *lk)
{
  struct proc *p = myproc();
  
  // Must acquire p->lock in order to
  // change p->state and then call sched.
  // Once we hold p->lock, we can be
  // guaranteed that we won't miss any wakeup
  // (wakeup locks p->lock),
  // so it's okay to release lk.

  acquire(&p->lock);  // DOC: sleeplock1
  release(lk);

  // 

  // Go to sleep.
  p->chan = chan;
  p->state = SLEEPING;

  // 
  sched();

  // Tidy up.
  p->chan = 0;

  // Reacquire original lock.
  release(&p->lock);
  acquire(lk);
}

// Wake up all processes sleeping on chan.
// Must be called without any p->lock.
void
wakeup(void *chan)
{
  struct proc *p;

  for(p = proc; p < &proc[NPROC]; p++) {
    if(p != myproc()){
      acquire(&p->lock);
      if(p->state == SLEEPING && p->chan == chan) {
        p->state = RUNNABLE;
      }
      release(&p->lock);
    }
  }
}

// Kill the process with the given pid.
// The victim won't exit until it tries to return
// to user space (see usertrap() in trap.c).
int
kill(int pid)
{
  struct proc *p;

  for(p = proc; p < &proc[NPROC]; p++){
    acquire(&p->lock);
    if(p->pid == pid){
      p->killed = 1;
      if(p->state == SLEEPING){
        // Wake process from sleep().
        p->state = RUNNABLE;
      }
      release(&p->lock);
      return 0;
    }
    release(&p->lock);
  }
  return -1;
}

void
setkilled(struct proc *p)
{
  acquire(&p->lock);
  p->killed = 1;
  release(&p->lock);
}

int
killed(struct proc *p)
{
  int k;
  
  acquire(&p->lock);
  k = p->killed;
  release(&p->lock);
  return k;
}

// Copy to either a user address, or kernel address,
// depending on usr_dst.
// Returns 0 on success, -1 on error.
int
either_copyout(int user_dst, uint64 dst, void *src, uint64 len)
{
  struct proc *p = myproc();
  if(user_dst){
    return copyout(p->pagetable, dst, src, len);
  } else {
    memmove((char *)dst, src, len);
    return 0;
  }
}

// Copy from either a user address, or kernel address,
// depending on usr_src.
// Returns 0 on success, -1 on error.
int
either_copyin(void *dst, int user_src, uint64 src, uint64 len)
{
  struct proc *p = myproc();
  if(user_src){
    return copyin(p->pagetable, dst, src, len);
  } else {
    memmove(dst, (char*)src, len);
    return 0;
  }
}

// Print a process listing to console.  For debugging.
// Runs when user types ^P on console.
// No lock to avoid wedging a stuck machine further.
void
procdump(void)
{
  static char *states[] = {
  [UNUSED]    "unused",
  [USED]      "used",
  [SLEEPING]  "sleep ",
  [RUNNABLE]  "runble",
  [RUNNING]   "run   ",
  [ZOMBIE]    "zombie"
  };
  struct proc *p;
  char *state;

  printf("\n");
  for(p = proc; p < &proc[NPROC]; p++){
    if(p->state == UNUSED)
      continue;
    if(p->state >= 0 && p->state < NELEM(states) && states[p->state])
      state = states[p->state];
    else
      state = "???";
    printf("%d %s %s", p->pid, state, p->name);
    printf("\n");
  }
}
