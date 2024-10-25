// Physical memory allocator, for user processes,
// kernel stacks, page-table pages,
// and pipe buffers. Allocates whole 4096-byte pages.

#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "riscv.h"
#include "defs.h"

void freerange(void *pa_start, void *pa_end);

extern char end[]; // first address after kernel.
                   // defined by kernel.ld.

struct run {
  struct run *next;
};

struct {
  struct spinlock lock;
  struct run *freelist;
} kmem;

void
kinit()
{
  initlock(&kmem.lock, "kmem");
  freerange(end, (void*)PHYSTOP);
}

void
freerange(void *pa_start, void *pa_end)
{
  char *p;
  p = (char*)PGROUNDUP((uint64)pa_start);
  for(; p + PGSIZE <= (char*)pa_end; p += PGSIZE) // p 需要以 4KB 页大小对齐的加入到 freelist
    kfree(p); // kfree 把 p 视为物理页起始地址, 并加入 freelist. 即释放物理页
}

// Free the page of physical memory pointed at by pa,
// which normally should have been returned by a
// call to kalloc().  (The exception is when
// initializing the allocator; see kinit above.)
// 释放物理地址 pa 所在的页
void
kfree(void *pa)
{
  // 变量 "r" 在 kernel 执行此函数时的栈中
  struct run *r;

  if(((uint64)pa % PGSIZE) != 0 || (char*)pa < end || (uint64)pa >= PHYSTOP)
    panic("kfree");

  // Fill with junk to catch dangling refs.
  memset(pa, 1, PGSIZE);

  // "r" 的值是 RAM 中 kernel 的代码和数据之外的部分的某个物理地址.
  // 且 "r" 的值将被作为结构体 run 解释
  r = (struct run*)pa;

  // 记录新的空闲页：更新链表，采用头插，表头是新"释放"的空闲页的起始地址
  // xv6的freelist的每一个节点表示一个空闲页
  // freelist的节点本身存在哪里？
  // 存在空闲页上, 因为这里本来就空闲. 从空闲页的起始地址开始，可以当节点解释
  // 一旦空闲页被分配，该节点也不用在freelist中了，节点内容也可以被覆盖

  // 物理页的“分配”，就是当页被映射到页表使用前，或者页本身作为进程的页表前，需要经过允许
  // 也就是需要被记录、隔离、区分: 这个页是正被使用或没有被使用的
  // 所以在使用页之前，经过一个过程的允许
  // 该过程记录所有页的是否被使用, 返回允许使用的页后，更新记录
  // 就是所谓的“分配”了

  // 下面说说和用户C程序链表的不同
  // 用户程序的页表有效范围有限（但地址空间固定），访问该范围外的地址，会产生page fault。
  // 所以编译后指令使用的地址范围，就是它被分配的地址范围。
  // 而kernel通过页表，可访问整个物理地址空间，大于kernel作为C程序被编译后，以及被加载的范围
  // 或者kernel的data范围可以是整个物理地址空间
  // 所以看待kernel和用户程序的要用不同的直觉：kernel可以读写、使用其编译后被加载的范围之外的地址
  // 所以其链表指针可以指向kernel之外
  acquire(&kmem.lock);
  // "r" 的值被作为结构体 run 解释. 
  // 在"r"的值(一个kernel外的物理地址)附近的一个位置写入kmem.freelist的值
  r->next = kmem.freelist;
  // 变量kmem.freelist(变量的区分会归约到地址即位置的区分，kmem.freelist的位置是kernel内的一个全局变量)
  // 即该全局变量上的值为变量"r"的值
  kmem.freelist = r;
  release(&kmem.lock);
}

// Allocate one 4096-byte page of physical memory.
// Returns a pointer that the kernel can use.
// Returns 0 if the memory cannot be allocated.
// 直接"分配"freelist表头的页，更新表头
void *
kalloc(void)
{
  struct run *r;

  acquire(&kmem.lock);
  r = kmem.freelist;
  if(r)
    kmem.freelist = r->next;
  release(&kmem.lock);

  if(r)
    memset((char*)r, 5, PGSIZE); // fill with junk
  return (void*)r;
}
