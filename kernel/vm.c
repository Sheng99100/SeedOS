#include "param.h"
#include "types.h"
#include "memlayout.h"
#include "elf.h"
#include "riscv.h"
#include "defs.h"
#include "fs.h"

/*
 * the kernel's page table.
 */
pagetable_t kernel_pagetable;

extern char etext[];  // kernel.ld sets this to end of kernel code.

extern char trampoline[]; // trampoline.S

// Make a direct-map page table for the kernel.
pagetable_t
kvmmake(void)
{
  pagetable_t kpgtbl;

  // allocate one 4096-byte page of physical memory as top-level page table
  // allocate the root of pagetable tree.
  kpgtbl = (pagetable_t) kalloc();

  // set the data of all PTEs for the page-table page with 0
  memset(kpgtbl, 0, PGSIZE);

  // uart registers
  kvmmap(kpgtbl, UART0, UART0, PGSIZE, PTE_R | PTE_W);

  // virtio mmio disk interface
  kvmmap(kpgtbl, VIRTIO0, VIRTIO0, PGSIZE, PTE_R | PTE_W);

  // PLIC
  kvmmap(kpgtbl, PLIC, PLIC, 0x4000000, PTE_R | PTE_W);

  // map kernel text executable and read-only.
  kvmmap(kpgtbl, KERNBASE, KERNBASE, (uint64)etext-KERNBASE, PTE_R | PTE_X);

  // map kernel data and the physical RAM we'll make use of.
  kvmmap(kpgtbl, (uint64)etext, (uint64)etext, PHYSTOP-(uint64)etext, PTE_R | PTE_W);

  // the highest virtual address in the kernel.
  kvmmap(kpgtbl, TRAMPOLINE, (uint64)trampoline, PGSIZE, PTE_R | PTE_X);

  // allocate and map a kernel stack for each process.
  proc_mapstacks(kpgtbl);
  
  return kpgtbl;
}

// Initialize the one kernel_pagetable
void
kvminit(void)
{
  kernel_pagetable = kvmmake();
}

// Switch h/w page table register to the kernel's page table,
// and enable paging.
void
kvminithart()
{
  // wait for any previous writes to the page table memory to finish.
  sfence_vma();

  w_satp(MAKE_SATP(kernel_pagetable));

  // flush stale entries from the TLB.
  sfence_vma();
}

// Return the address of the PTE in page table pagetable
// that corresponds to virtual address va. 
// If alloc!=0, create any required page-table pages.
//
// The risc-v Sv39 scheme has three levels of page-table
// pages. A page-table page contains 512 64-bit PTEs.
// A 64-bit virtual address is split into five fields:
//   39..63 -- must be zero.
//   30..38 -- 9 bits of level-2 index.
//   21..29 -- 9 bits of level-1 index.
//   12..20 -- 9 bits of level-0 index.
//    0..11 -- 12 bits of byte offset within the page.
pte_t *
walk(pagetable_t pagetable, uint64 va, int alloc)
{
  if(va >= MAXVA)
    panic("walk");

  // begin at top-level(level 2) page table. 
  for(int level = 2; level > 0; level--) {
    // the type of pagetable_t is uint64, which length is 64 bits. 
    // in abstraction of C, '&array[index]' means address 
    // (&array) + (length of data-type) * (index).
    // so that's why xv6 use uint64 as type of pagetable

    // PX(level, va) is the 9-bit offset (The unit is PTE) 
    // of current level page table
    pte_t *pte = &pagetable[PX(level, va)];
    // if the PTE is valid 
    // (in this case, that means that page-table page of this va was already allocated)
    if(*pte & PTE_V) {
      // get physical base address of next level pagetable
      pagetable = (pagetable_t)PTE2PA(*pte);
    // if the PTE is not valid
    } else {
      // allocate a page-table page for next level
      // e.g, when current pte is in level 1 pagetable and valid flag==0
      // then allocate a page as page-table page in level 0. 
      // and update variable "pagetable" to new next level's page-table page
      if(!alloc || (pagetable = (pde_t*)kalloc()) == 0)
        return 0;
      memset(pagetable, 0, PGSIZE);

      // 到这里时，pte 还是 va 在上级页表中对应的 PTE 指针
      // 但是 pagetable 已经是下一级页表的物理基地址了
      // 将新分配的下级页表页的物理基地址，给va对应的，在上级的pte.
      *pte = PA2PTE(pagetable) | PTE_V;
    }
  }

  // return the physics address of pte of va. 
  // by （&pagetable) + (last 9-bits offset in va)  (pagetable will in final level 0)
  return &pagetable[PX(0, va)];
}

// Look up a virtual address, return the physical address,
// or 0 if not mapped.
// Can only be used to look up user pages.
// 返回虚拟地址在指定页表映射的物理页起始地址(物理页号加 12 位 0)
// 用户页表也会映射内核的物理地址, 例如 trapframe 和 trampoline
// 为了避免用户指令的bug或利用
// 需检查虚拟地址是否有效(是否已被分配), 以及是否是用户空间的地址而不是内核空间地址
// 有效且确实是用户空间地址: 返回物理页起始地址
// 否则返回 0
uint64
walkaddr(pagetable_t pagetable, uint64 va)
{
  pte_t *pte;
  uint64 pa;

  if(va >= MAXVA)
    return 0;

  pte = walk(pagetable, va, 0);
  if(pte == 0)
    return 0;
  if((*pte & PTE_V) == 0)
    return 0;
  if((*pte & PTE_U) == 0)
    return 0;
  pa = PTE2PA(*pte);
  return pa;
}

// add a mapping to the kernel page table.
// only used when booting.
// does not flush TLB or enable paging.
void
kvmmap(pagetable_t kpgtbl, uint64 va, uint64 pa, uint64 sz, int perm)
{
  if(mappages(kpgtbl, va, sz, pa, perm) != 0)
    panic("kvmmap");
}

// Create PTEs for virtual addresses starting at va that refer to
// physical addresses starting at pa.
// va and size MUST be page-aligned.
// Returns 0 on success, -1 if walk() couldn't
// allocate a needed page-table page.
//
// 从首个虚拟页 va 和它对应的首个物理页 pa 开始, 映射连续的虚拟页和物理页
// 即映射连续的（pa / size）个虚拟页和物理页
// size 是页大小的整数倍， 是页的个数
int
mappages(pagetable_t pagetable, uint64 va, uint64 size, uint64 pa, int perm)
{
  uint64 a, last;
  pte_t *pte;

  // 
  if((va % PGSIZE) != 0)
    panic("mappages: va not aligned");

  if((size % PGSIZE) != 0)
    panic("mappages: size not aligned");

  if(size == 0)
    panic("mappages: size");
  
  a = va; // 要映射范围的第一个虚拟页的起始地址
  last = va + size - PGSIZE; // 要映射范围的最后一个虚拟页的起始地址
  for(;;){
    if((pte = walk(pagetable, a, 1)) == 0)
      return -1;
    if(*pte & PTE_V)
      panic("mappages: remap");
    *pte = PA2PTE(pa) | perm | PTE_V;
    if(a == last)
      break;
    a += PGSIZE;
    pa += PGSIZE;
  }
  return 0;
}

// Remove npages of mappings starting from va. va must be
// page-aligned. The mappings must exist.
// Optionally free the physical memory.
void
uvmunmap(pagetable_t pagetable, uint64 va, uint64 npages, int do_free)
{
  uint64 a;
  pte_t *pte;

  if((va % PGSIZE) != 0)
    panic("uvmunmap: not aligned");

  for(a = va; a < va + npages*PGSIZE; a += PGSIZE){
    if((pte = walk(pagetable, a, 0)) == 0)
      panic("uvmunmap: walk");
    if((*pte & PTE_V) == 0)
      panic("uvmunmap: not mapped");
    if(PTE_FLAGS(*pte) == PTE_V)
      panic("uvmunmap: not a leaf");
    if(do_free){
      uint64 pa = PTE2PA(*pte);
      kfree((void*)pa);
    }
    *pte = 0;
  }
}

// create an empty user page table.
// returns 0 if out of memory.
pagetable_t
uvmcreate()
{
  pagetable_t pagetable;
  pagetable = (pagetable_t) kalloc();
  if(pagetable == 0)
    return 0;
  memset(pagetable, 0, PGSIZE);
  return pagetable;
}

// Load the user initcode into address 0 of pagetable,
// for the very first process.
// sz must be less than a page.
// 分配一个物理页, 并映射到该初始进程的地址空间的 0 地址处
// 把 initcode.S 的代码复制到该进程的物理页
void
uvmfirst(pagetable_t pagetable, uchar *src, uint sz)
{
  char *mem;

  if(sz >= PGSIZE)
    panic("uvmfirst: more than a page");
  mem = kalloc();
  memset(mem, 0, PGSIZE);
  mappages(pagetable, 0, PGSIZE, (uint64)mem, PTE_W|PTE_R|PTE_X|PTE_U);
  memmove(mem, src, sz);
}

// Allocate PTEs and physical memory to grow process from oldsz to
// newsz, which need not be page aligned.  Returns new size or 0 on error.
// oldsz 和 newsz 均为虚拟地址
// 虽然变量名有 "size" 的含义
// 但是该函数实际上是在指定页表中,
// 对虚拟地址 oldsz 到 newsz 所在的及之间的虚拟页都分配物理页
// 并且更新和分配所需的页表块
// 所以, 技术上是允许用户进程所使用到的虚拟页是不连续的
// 但该函数的 caller 一般指定 oldsz = proc->sz. 所以一般还是连续的
uint64
uvmalloc(pagetable_t pagetable, uint64 oldsz, uint64 newsz, int xperm)
{
  char *mem;
  uint64 a;

  if(newsz < oldsz)
    return oldsz;

  // 向上对齐到页的结束地址
  oldsz = PGROUNDUP(oldsz);
  for(a = oldsz; a < newsz; a += PGSIZE){
    mem = kalloc();
    if(mem == 0){
      uvmdealloc(pagetable, a, oldsz);
      return 0;
    }
    memset(mem, 0, PGSIZE);
    if(mappages(pagetable, a, PGSIZE, (uint64)mem, PTE_R|PTE_U|xperm) != 0){
      kfree(mem);
      uvmdealloc(pagetable, a, oldsz);
      return 0;
    }
  }
  return newsz;
}

// Deallocate user pages to bring the process size from oldsz to
// newsz.  oldsz and newsz need not be page-aligned, nor does newsz
// need to be less than oldsz.  oldsz can be larger than the actual
// process size.  Returns the new process size.
uint64
uvmdealloc(pagetable_t pagetable, uint64 oldsz, uint64 newsz)
{
  if(newsz >= oldsz)
    return oldsz;

  if(PGROUNDUP(newsz) < PGROUNDUP(oldsz)){
    int npages = (PGROUNDUP(oldsz) - PGROUNDUP(newsz)) / PGSIZE;
    uvmunmap(pagetable, PGROUNDUP(newsz), npages, 1);
  }

  return newsz;
}

// Recursively free page-table pages.
// All leaf mappings must already have been removed.
void
freewalk(pagetable_t pagetable)
{
  // there are 2^9 = 512 PTEs in a page table.
  for(int i = 0; i < 512; i++){
    pte_t pte = pagetable[i];
    if((pte & PTE_V) && (pte & (PTE_R|PTE_W|PTE_X)) == 0){
      // this PTE points to a lower-level page table.
      uint64 child = PTE2PA(pte);
      freewalk((pagetable_t)child);
      pagetable[i] = 0;
    } else if(pte & PTE_V){
      panic("freewalk: leaf");
    }
  }
  kfree((void*)pagetable);
}

// Free user memory pages,
// then free page-table pages.
void
uvmfree(pagetable_t pagetable, uint64 sz)
{
  if(sz > 0)
    uvmunmap(pagetable, 0, PGROUNDUP(sz)/PGSIZE, 1);
  freewalk(pagetable);
}

// Given a parent process's page table, copy
// its memory into a child's page table.
// Copies both the page table and the
// physical memory.
// returns 0 on success, -1 on failure.
// frees any allocated pages on failure.
int
uvmcopy(pagetable_t old, pagetable_t new, uint64 sz)
{
  pte_t *pte;
  uint64 pa, i;
  uint flags;
  char *mem;

  for(i = 0; i < sz; i += PGSIZE){
    if((pte = walk(old, i, 0)) == 0)
      panic("uvmcopy: pte should exist");
    if((*pte & PTE_V) == 0)
      panic("uvmcopy: page not present");
    pa = PTE2PA(*pte);
    flags = PTE_FLAGS(*pte);
    if((mem = kalloc()) == 0)
      goto err;
    memmove(mem, (char*)pa, PGSIZE);
    if(mappages(new, i, PGSIZE, (uint64)mem, flags) != 0){
      kfree(mem);
      goto err;
    }
  }
  return 0;

 err:
  uvmunmap(new, 0, i / PGSIZE, 1);
  return -1;
}

// mark a PTE invalid for user access.
// used by exec for the user stack guard page.
void
uvmclear(pagetable_t pagetable, uint64 va)
{
  pte_t *pte;
  
  pte = walk(pagetable, va, 0);
  if(pte == 0)
    panic("uvmclear");
  *pte &= ~PTE_U;
}

// Copy from kernel to user.
// Copy len bytes from src to virtual address dstva in a given page table.
// Return 0 on success, -1 on error.
int
copyout(pagetable_t pagetable, uint64 dstva, char *src, uint64 len)
{
  uint64 n, va0, pa0;
  pte_t *pte;

  while(len > 0){
    va0 = PGROUNDDOWN(dstva);
    if(va0 >= MAXVA)
      return -1;
    pte = walk(pagetable, va0, 0);
    if(pte == 0 || (*pte & PTE_V) == 0 || (*pte & PTE_U) == 0 ||
       (*pte & PTE_W) == 0)
      return -1;
    pa0 = PTE2PA(*pte);
    n = PGSIZE - (dstva - va0);
    if(n > len)
      n = len;
    memmove((void *)(pa0 + (dstva - va0)), src, n);

    len -= n;
    src += n;
    dstva = va0 + PGSIZE;
  }
  return 0;
}

// Copy from user to kernel.
// Copy len bytes to dst from virtual address srcva in a given page table.
// Return 0 on success, -1 on error.
// 从用户空间读取指定数量的字节到指定物理地址.
// 实际上是从指定用户页表的虚拟地址复制len字节到指定的内核页表地址
// 但是 xv6 内核页表是直接映射到物理地址，所以这种前提下等价于复制到物理地址
int
copyin(pagetable_t pagetable, char *dst, uint64 srcva, uint64 len)
{
  uint64 n, va0, pa0;

  while(len > 0){
    va0 = PGROUNDDOWN(srcva);
    // 首先映射到物理地址页的起始地址
    pa0 = walkaddr(pagetable, va0);
    // 检查给定的虚拟地址是否确实是其用户空间的地址.
    if(pa0 == 0)
      return -1;
    n = PGSIZE - (srcva - va0);
    if(n > len)
      n = len;
    memmove(dst, (void *)(pa0 + (srcva - va0)), n);

    len -= n;
    dst += n;
    srcva = va0 + PGSIZE;
  }
  return 0;
}

// Copy a null-terminated string from user to kernel.
// Copy bytes to dst from virtual address srcva in a given page table,
// until a '\0', or max.
// Return 0 on success, -1 on error.
// 从用户空间读取字符串(字节串)到指定物理地址.
// 和上面的 copyin 的区别是, 是否把字节串解释为字符串而检查终止符 '\0'
// 复制完 '\0' 或 max 个字符结束.
// 复制过 '\0' 返回 0 (算 success), 否则返回 -1 (error)
// 无论结束后有没有复制到 '\0', 前面复制过的字节都会真正复制而不会撤消
int
copyinstr(pagetable_t pagetable, char *dst, uint64 srcva, uint64 max)
{
  uint64 n, va0, pa0;
  int got_null = 0; // flag, 标记是否读到字符串结尾标记 '\0'


  // 当前的解释环境是使用内核页表, 地址本就被一一映射到物理地址
  // 所以, 每轮虚拟地址都要用指定的用户页表转换成物理地址, 物理地址再经过页表原样映射到自己
  // 而给出的目的地址按约定是解释为物理地址, 所以不需要转换, 直接操作指针即可

  // 因为复制范围可能跨多个连续的用户空间的页
  // 内层循环复制单个页内指定地址开始的字节
  // 外层循环切换到下一页
  // 被复制字符串所在的第一个页, 不一定从页的最低地址开始
  // 第二页开始, 一定是从页的最低地址开始
  while(got_null == 0 && max > 0){
    va0 = PGROUNDDOWN(srcva);       // 计算虚拟地址所在虚拟页的起始虚拟地址(低地址)
    pa0 = walkaddr(pagetable, va0); // 得到对应物理页的页起始地址. va0 换成 srcva 是一样的
    if(pa0 == 0)
      return -1;

    // 单轮复制字符数 n 不超过这个页，且不大于 max
    n = PGSIZE - (srcva - va0);
    if(n > max)
      n = max;

    // 加上偏移得到被复制字符串在本轮所在的页的起始物理地址
    char *p = (char *) (pa0 + (srcva - va0));
    while(n > 0){
      if(*p == '\0'){
        *dst = '\0';
        got_null = 1;
        break;
      } else {
        *dst = *p;
      }
      --n;      // 单轮最多复制字符数 - 1
      --max;    // 最多复制字符数 - 1
      p++;      // 被复制字符串起始地址 + 1 (下次复制新字符)
      dst++;    // 复制的目的地址 + 1 (下次复制新字符)
    }

    srcva = va0 + PGSIZE; // 转到下一虚拟页的起始虚拟地址
  }
  if(got_null){
    return 0;
  } else {
    return -1;
  }
}
