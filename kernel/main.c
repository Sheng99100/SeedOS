#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "defs.h"

volatile static int started = 0;

// start() jumps here in supervisor mode on all CPUs.
void
main()
{
  // 初始化由一个 CPU 来做即可
  // 其他 CPU 自旋等待初始化完毕，标记变量 started 被设为 1
  // 这里虽然涉及“自旋”却不需要锁，因为判断进入临界区的变量都是不同的
  // 在临界区之后自旋判断初始化完成没有，完成进入即可, 自旋之后的代码也不存在竞争,
  // 无所谓是否多个 CPU 同时判断完成并退出自旋 
  if(cpuid() == 0){
    consoleinit();
    printfinit();
    printf("\n");
    printf("[main]: xv6 kernel is booting\n");
    printf("\n");
    // 在开启 MMU 映射之前，kinit, kvminit 访问的地址都视为物理地址直接传给 RAM
    kinit();         // physical page allocator. 初始化 freelist, 其中包含kernel代码即数据之外（every page between the end of the kernel and PHYSTOP）的全部可用物理页
    kvminit();       // create kernel page table. 创建 directly mapping 的 kernel page table
    kvminithart();   // turn on paging. 把kernel中作为全局变量的 kernel page 的地址写入 satp 页表寄存器
    procinit();      // process table 的初始化、只包含绑定内核栈, 设置状态, 初始化锁
    trapinit();      // trap vectors
    trapinithart();  // install kernel trap vector
    plicinit();      // set up interrupt controller
    plicinithart();  // ask PLIC for device interrupts
    binit();         // buffer cache
    iinit();         // inode table
    fileinit();      // file table
    virtio_disk_init(); // emulated hard disk
    userinit();      // first user process
    __sync_synchronize();
    started = 1;
  } else {
    while(started == 0)
      ;
    __sync_synchronize();
    printf("hart %d starting\n", cpuid());
    kvminithart();    // turn on paging
    trapinithart();   // install kernel trap vector
    plicinithart();   // ask PLIC for device interrupts
  }

  scheduler();        
}
