#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "spinlock.h"
#include "proc.h"
#include "defs.h"

struct spinlock tickslock;
uint ticks;

extern char trampoline[], uservec[], userret[];

// in kernelvec.S, calls kerneltrap().
void kernelvec();

extern int devintr();

void
trapinit(void)
{
  initlock(&tickslock, "time");
}

// set up to take exceptions and traps while in the kernel.
void
trapinithart(void)
{
  w_stvec((uint64)kernelvec);
}

//
// handle an interrupt, exception, or system call from user space.
// called from trampoline.S
//
void
usertrap(void)
{
  int which_dev = 0;

  // 检查当前在内核模式执行usertrap之前()的特权级别
  if((r_sstatus() & SSTATUS_SPP) != 0)
    panic("usertrap: not from user mode");

  // send interrupts and exceptions to kerneltrap(),
  // since we're now in the kernel.
  // 在user mode下的中断, stvec 指向 usertrap
  // 在superviser mode下的中断，stvec 指向 kernelvec
  // 如果 trap 类型是系统调用或时钟中断, 在处理期间会开启中断; 
  // xv6 在处理设备中断时是不开中断的
  // 硬件视 stvec 寄存器值为物理地址直接访问内存
  // 而编译器是将 kernelvec 等函数在(虚拟)地址空间内分配地址的
  // 所以编译后，将 kernelvec 载入到 stvec 的指令内容
  // 是将 kernelvec 的编译出的地址（虚拟地址）载入到 stvec (需要物理地址) 
  // 要正确运行，就需要让 kernelvec 在地址空间里的地址
  // 也就是编译后，kernelvec 所在的虚拟地址
  // 与 kernelvec 实际载入的物理地址(从0x80000000开始)一致
  // 要让编译器知道，视该程序的地址空间从0x80000000开始
  // 这就是为什么说，让内核的虚拟地址和物理地址直接映射是为了更方便
  w_stvec((uint64)kernelvec);

  // cpus[]，procs[], 都作为全局变量存在 C 程序（OS）地址空间中
  struct proc *p = myproc();
  
  // save user program counter.
  // 任何中断发生后，硬件都会关中断，并写sepc, scause, sstatus 寄存器
  // 如果开中断，可能会覆盖 sepc, scause, sstatus 寄存器，所以要保存 sepc 到 trmapframe. 
  // 等返回 userspace 时再写回 sepc
  p->trapframe->epc = r_sepc(); // 也可以在 trampoline 的 uservec 中保存

  if(r_scause() == 8){ // system call
    
    if(killed(p))
      exit(-1);

    // sepc points to the ecall instruction,
    // but we want to return to the next instruction.
    p->trapframe->epc += 4;

    // an interrupt will change sepc, scause, and sstatus,
    // so enable only now that we're done with those registers.
    // xv6 不支持多级中断
    // 所以只有是系统调用时，才在内核模式中再开中断，否则保持中断关闭
    // 等需要的寄存器（包括epc, scause, sstatus）都保存完毕，可以开中断了
    intr_on();

    // system call 的 dispature. 库函数约定调用号放在 a7 寄存器
    // RISC-V 的 ecall 的跳转不考虑调用号，只是改pc到 stvec 寄存器中的值地址
    // 且不修改页表，即访问的物理地址还是trap前的原页表映射的
    syscall();

    // devintr()
    // returns 2 if timer interrupt
    // 1 if other device,
    // 0 if not recognized.
  } else if((which_dev = devintr()) != 0){
    // ok
    // devintr() 会处理中断，同时返回中断类型(非中断号)
    // RISC-V 中断的处理不是查中断向量表，而是把 pc 值改成 stvec 寄存器的值
  } else {
    printf("usertrap(): unexpected scause 0x%lx pid=%d\n", r_scause(), p->pid);
    printf("            sepc=0x%lx stval=0x%lx\n", r_sepc(), r_stval());
    setkilled(p);
  }

  if(killed(p))
    exit(-1);

  // give up the CPU if this is a timer interrupt.
  if(which_dev == 2)
    yield();

  usertrapret();
}

//
// return to user space
//
void
usertrapret(void)
{
  struct proc *p = myproc();

  // we're about to switch the destination of traps from
  // kerneltrap() to usertrap(), so turn off interrupts until
  // we're back in user space, where usertrap() is correct.
  // 中断可能已经在 usertrap 内被开启，现在要关闭，因为
  // 接下来要返回用户模式，所以下面要吧中断向量寄存器换回用户模式下的中断处理程序地址了
  // 但是真正回到用户模式之前，目前的usertrapret()还在内核模式下
  // 所以要先关中断。
  // 不能让内核模式下的中断，用用户模式的trampoline处理
  // 所以要暂时关中断, sret 会根据 SSTATUS 中的 SPIE 位决定返回后中断是否开启.
  // 通过设置 SSTATUS 中的 SPIE 位, 让 sret 返回后再开中断.
  // 否则会产生的问题例如：
  // 1. 用户模式下中断跳转到的 trampoline，是默认假设在用户页表的
  // 而内核中断跳转的 kernelvec 默认假设在内核页表
  // 2. trampoline 中以字面值直接表示 trapframe 的地址
  // 而这个地址只在用户页表才能正确映射到trapframe的物理地址
  // 内核模式和用户模式的trapframe虚拟地址不一样
  // 内核模式trapframe地址是 p->trapframe （虚拟地址和物理地址直接映射）
  // 用户模式trapfeame地址统一都固定在用户空间的高地址
  // 3. 用户 trap 和内核 trap 的整个处理逻辑就不同
  intr_off();

  // send syscalls, interrupts, and exceptions to uservec in trampoline.S
  // 设置回用户模式下发生中断要跳转到的 uservec
  uint64 trampoline_uservec = TRAMPOLINE + (uservec - trampoline);
  w_stvec(trampoline_uservec);

  // set up trapframe values that uservec will need when
  // the process next traps into the kernel.
  // 如果进程刚刚被创建, 例如
  // 1. fork() 出的进程
  // 2. 用来 fork 出首个用户进程的初始进程
  // 那么这些新进程 trapframe 需要一些初始化
  // 对于老进程, 左右值相等
  p->trapframe->kernel_satp = r_satp();         // kernel page table
  p->trapframe->kernel_sp = p->kstack + PGSIZE; // process's kernel stack
  p->trapframe->kernel_trap = (uint64)usertrap;
  p->trapframe->kernel_hartid = r_tp();         // hartid for cpuid()

  // set up the registers that trampoline.S's sret will use
  // to get to user space.
  
  // set S Previous Privilege mode to User.
  // 设置 sret 指令执行后的 CPU 状态
  // 这些状态不是作为用户模式或内核模式的上下文保存的
  // 从内核返回到用户所设置的状态是固定的两件事：
  // 1. 修改特权级别位用户模式
  // 2. 切换回用户线程的上下文后（sret 后）重新开中断
  unsigned long x = r_sstatus();
  x &= ~SSTATUS_SPP; // clear SPP to 0 for user mode
  x |= SSTATUS_SPIE; // enable interrupts in user mode
  w_sstatus(x);

  // set S Exception Program Counter to the saved user pc.
  // 内核模式下中断会覆盖 sepc（跳进内核前的用户模式的地址）
  // 所以在 usertrap() 开中断前就保存了 sepc 到 trapframe
  // 现在，在 usertrapret() 时恢复, 以让 trampoline 中的 sret 跳转回之前用户程序的地址
  //
  // 以及进程刚被分配时，进程首次被调度的情况下，sepc 不是由中断的硬件过程写的
  // 所以要返回到哪里呢?
  // 办法是在初始化进程时， trapframe->sepc 初始化为 0
  // 在 usertrapret() 中写入 sepc 寄存器
  // 当首次被调度时，schduler() 会调用 swtch()，于是上下文切换到 forkret() -> usertrapret()
  // 从而“假装”是从用户态 trap 进内核，再返回用户态的
  w_sepc(p->trapframe->epc);

  // tell trampoline.S the user page table to switch to.
  // 改回进入内核前的用户页表
  uint64 satp = MAKE_SATP(p->pagetable);

  // jump to userret in trampoline.S at the top of memory, which 
  // switches to the user page table, restores user registers,
  // and switches to user mode with sret.
  // 函数调用约定首个函数参数放在 a0
  // trampoline.S 的 userret 会从 a0 读取 satp 
  uint64 trampoline_userret = TRAMPOLINE + (userret - trampoline);
  ((void (*)(uint64))trampoline_userret)(satp);
}

// interrupts and exceptions from kernel code go here via kernelvec,
// on whatever the current kernel stack is.
void
kerneltrap()
{
  int which_dev = 0;
  // yield() 切换到其他线程，在其他线程 scheduler 线程或
  // 会重新开中断, 可能导致 trap, 覆盖以下寄存器
  // 所以先保存被中断过程覆盖的寄存器
  uint64 sepc = r_sepc();
  uint64 sstatus = r_sstatus();
  uint64 scause = r_scause();
  
  if((sstatus & SSTATUS_SPP) == 0)
    panic("kerneltrap: not from supervisor mode");
  // 检查当前是否激活(开)中断, kerneltrap 下不能开中断
  if(intr_get() != 0)
    panic("kerneltrap: interrupts enabled");

  // devintr() 在处理中断的同时返回中断类型(非中断号)
  // RISC-V 中断的处理不是查中断向量表，而是把 pc 值改成 stvec 寄存器的值
  if((which_dev = devintr()) == 0){
    // interrupt or trap from an unknown source
    printf("scause=0x%lx sepc=0x%lx stval=0x%lx\n", scause, r_sepc(), r_stval());
    panic("kerneltrap");
  }

  // give up the CPU if this is a timer interrupt.
  if(which_dev == 2 && myproc() != 0)
    yield();

  // the yield() may have caused some traps to occur,
  // so restore trap registers for use by kernelvec.S's sepc instruction.
  w_sepc(sepc);
  w_sstatus(sstatus);

  // 因为 kernelvec 中以 call 调用此函数 kerneltrap
  // 所以根据调用约定, 返回到 kernelvec 的返回地址会在 call kerneltrap 时写入 ra
  // 现在 kerneltrap 执行 ret, 将 ra 写入 pc, 返回 kernelvec
  // kernelvec 再执行 sret 将 sepc 写入 pc, 返回之前的 kernel 模式下被中断的位置
}

void
clockintr()
{
  if(cpuid() == 0){
    acquire(&tickslock);
    ticks++;
    wakeup(&ticks);
    release(&tickslock);
  }

  // ask for the next timer interrupt. this also clears
  // the interrupt request. 1000000 is about a tenth
  // of a second.
  w_stimecmp(r_time() + 1000000);
}

// check if it's an external interrupt or software interrupt,
// and handle it.
// returns 2 if timer interrupt,
// 1 if other device,
// 0 if not recognized.
int
devintr()
{
  uint64 scause = r_scause();

  if(scause == 0x8000000000000009L){
    // this is a supervisor external interrupt, via PLIC.

    // irq indicates which device interrupted.
    int irq = plic_claim();

    if(irq == UART0_IRQ){
      uartintr();
    } else if(irq == VIRTIO0_IRQ){
      virtio_disk_intr();
    } else if(irq){
      printf("unexpected interrupt irq=%d\n", irq);
    }

    // the PLIC allows each device to raise at most one
    // interrupt at a time; tell the PLIC the device is
    // now allowed to interrupt again.
    if(irq)
      plic_complete(irq);

    return 1;
  } else if(scause == 0x8000000000000005L){
    // timer interrupt.
    clockintr();
    return 2;
  } else {
    return 0;
  }
}

