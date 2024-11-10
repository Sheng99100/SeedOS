// File system implementation.  Five layers:
//   + Blocks: allocator for raw disk blocks.
//   + Log: crash recovery for multi-step updates.
//   + Files: inode allocator, reading, writing, metadata.
//   + Directories: inode with special contents (list of other inodes!)
//   + Names: paths like /usr/rtm/xv6/fs.c for convenient naming.
//
// This file contains the low-level file system manipulation
// routines.  The (higher-level) system call implementations
// are in sysfile.c.

#include "types.h"
#include "riscv.h"
#include "defs.h"
#include "param.h"
#include "stat.h"
#include "spinlock.h"
#include "proc.h"
#include "sleeplock.h"
#include "fs.h"
#include "buf.h"
#include "file.h"

#define min(a, b) ((a) < (b) ? (a) : (b))
// there should be one superblock per disk device, but we run with
// only one device
struct superblock sb; 

// Read the super block.
// 从已经格式化为文件系统的磁盘中，读取 superblock 磁盘块到内存结构
static void
readsb(int dev, struct superblock *sb)
{
  struct buf *bp;

  bp = bread(dev, 1);
  memmove(sb, bp->data, sizeof(*sb));
  brelse(bp);
}

// Init fs
// 同步 superblock、log 到内存结构；尝试从 log 恢复 
void
fsinit(int dev) {
  readsb(dev, &sb);
  if(sb.magic != FSMAGIC)
    panic("invalid file system");
  initlog(dev, &sb);
}

// Zero a block.
// 使 block 中的 bit 全部置 0
static void
bzero(int dev, int bno)
{
  struct buf *bp;

  bp = bread(dev, bno);
  memset(bp->data, 0, BSIZE);
  log_write(bp);
  brelse(bp);
}

// Blocks.

// Allocate a zeroed disk block.
// returns 0 if out of disk space.
static uint
balloc(uint dev)
{
  int b, bi, m;
  struct buf *bp;

  bp = 0;
  // sb.size 是整个文件系统（包括预留的 data block）的块数量
  // 每个块占 bitmap 的一位
  // sb.size 可以解释为 bitmap 的最大 bit 数量
  // 循环中是把块号解释为 bitmap 中的 bit 号
  // BBLOCK(b, sb) 计算 blockno 对应的 bitmap 中的 bit，处于磁盘的第几个 block
  for(b = 0; b < sb.size; b += BPB){ // 每次循环 b 增加到下一个 bitmap 块的第一位；bitmap 的位数最多为文件系统的块数 sb.size
    bp = bread(dev, BBLOCK(b, sb)); // 读取 bitmap 的各个磁盘块到内存结构（缓存 buf 中的 uchar 数组）
    for(bi = 0; bi < BPB && b + bi < sb.size; bi++){ // 遍历 bitmap 各个块的每一位
      m = 1 << (bi % 8);
      // 磁盘块数据读到内存后，如何使用它？
      // 磁盘驱动复制磁盘块到内存指定的地址
      // 该地址是 buf.data, 程序将其解释为 uchar 数组
      // 在这个解释方式下，可以用 data[] 按 [字节] 取磁盘块内的数据
      // 但需要在当前 bitmap 按 [位] 来遍历
      // 在这两个条件下，取当前 bitmap 块中第 bi 位的办法是
      //  * 将 bitmap 按字节编址，取第 bi 位所处的第 bi/8 个字节；
      //  * 第 bi 位在所处字节中是第 bi % 8 位，利用它来构造掩码
      //    例如，第 bi 位在所处字节中位第 bi % 8 = 7 位，则掩码是 1 << 7 = 01000000 (8位无符号左移)
      //    再将掩码和第 bi 位所处的字节进行按位与过程，就得到第 bi 位是 1 或 0   
      if((bp->data[bi/8] & m) == 0){  // Is block free?
        bp->data[bi/8] |= m;  // Mark block in use.
        log_write(bp);
        brelse(bp);
        // 如果当前 bitmap 块的第 bi 位是 0
        // 即整个 bitmap 中的第 b+bi 位是 0
        // 即文件系统的第 b+bi 块空闲
        // 就将该位置 1，表明文件系统后续再需要新块时，该块就不能再被其他 balloc 分配
        // 最后返回该块号
        // 
        // 什么是 “分配” 以及对应的 “删除” ？
        // 其基础是
        // * 过程不能够随意的使用某个空间，以免 [覆盖] 空间中的内容
        // * 避免一个过程读写与该过程自身的逻辑不符的内容，导致逻辑的不自洽
        //   例如写文件 f1 的过程，把内容写到了文件 f2.
        //   导致，读写 f1，f2 的所有过程的逻辑都不自洽了：
        //   前面写近 "f2" 的内容，马上读出来应该符合之前写进的更新，可是却读到了 f1 的内容
        // 所以各个过程之间必须存在通信和约定。如何制定该 “约定” ？
        // 空间的 “分配” “占用” 的抽象，延伸于 “特定块（空间）只能被特定过程使用”
        // 确保各个过程在过程中只使用特定空间，而不 [覆盖] 其他已被使用的空间
        // 特定过程不是针对某个块写死的过程，而是相对抽象的过程被具体化后的过程
        // 
        // * 每个过程约定，在各自过程中只访问某些特定块，这些特定块也只能由这些过程访问
        //   直到过程自身或其他高权限过程主动解除 “块和过程的绑定”
        //
        // * 在过程需要使用某些块前，[决定] 使用哪些块、这些块不能被其他未经 [决定]
        //   的过程所使用。
        // 
        // * [决定使用某个块] 以及各个过程 [约定] 只使用各自已经 [决定过的块]，称为 "分配"
        //   这两个约定使过程不会 [覆盖] 掉其他过程正在使用的 [空间]
        //
        // * “删除” 是让该空间后续可以重新被其他线程 [决定] 使用
        //
        // 在写文件 f 的过程中，被写的所有 inode 块以及 data 块
        // 都是 [文件 f 访问过程] 分配到的块，或者文件 f 分配到的块
        // 写文件 f 的过程没有写属于其他文件的块
        // 这说明 "分配" 可以被抽象成由多个层级构成
        // * 通过 bitmap 大致分配，说明是否被使用
        // * 通过 inode 分配块给具体文件(文件过程）
        //
        // 在物理空间中，判断一个空间 “是否已经被其他过程决定使用”，可直接用是否被 “占用” 判断
        // 在此基础上，“分配” 是
        // 每个过程在一开始仅寻找空闲的空间，而不把已经占用空间的东西搬走
        // 每个过程知道自己使用的空间在哪里，不使用其他空间
        // 
        // 而对于计算过程，空间是否被占用就需要用表来判断
        bzero(dev, b + bi);
        return b + bi;
      }
    }
    brelse(bp);
  }
  printf("balloc: out of blocks\n");
  return 0;
}

// Free a disk block.
static void
bfree(int dev, uint b)
{
  struct buf *bp;
  int bi, m;

  bp = bread(dev, BBLOCK(b, sb));
  bi = b % BPB;
  m = 1 << (bi % 8);
  if((bp->data[bi/8] & m) == 0)
    panic("freeing free block");
  bp->data[bi/8] &= ~m;
  log_write(bp);
  brelse(bp);
}

// Inodes.
//
// An inode describes a single unnamed file.
// The inode disk structure holds metadata: the file's type,
// its size, the number of links referring to it, and the
// list of blocks holding the file's content.
//
// The inodes are laid out sequentially on disk at block
// sb.inodestart. Each inode has a number, indicating its
// position on the disk.
//
// The kernel keeps a table of in-use inodes in memory
// to provide a place for synchronizing access
// to inodes used by multiple processes. The in-memory
// inodes include book-keeping information that is
// not stored on disk: ip->ref and ip->valid.
//
// An inode and its in-memory representation go through a
// sequence of states before they can be used by the
// rest of the file system code.
//
// * Allocation: an inode is allocated if its type (on disk)
//   is non-zero. ialloc() allocates, and iput() frees if
//   the reference and link counts have fallen to zero.
//
// * Referencing in table: an entry in the inode table
//   is free if ip->ref is zero. Otherwise ip->ref tracks
//   the number of in-memory pointers to the entry (open
//   files and current directories). iget() finds or
//   creates a table entry and increments its ref; iput()
//   decrements ref.
//
// * Valid: the information (type, size, &c) in an inode
//   table entry is only correct when ip->valid is 1.
//   ilock() reads the inode from
//   the disk and sets ip->valid, while iput() clears
//   ip->valid if ip->ref has fallen to zero.
//
// * Locked: file system code may only examine and modify
//   the information in an inode and its content if it
//   has first locked the inode.
//
// Thus a typical sequence is:
//   ip = iget(dev, inum)
//   ilock(ip)
//   ... examine and modify ip->xxx ...
//   iunlock(ip)
//   iput(ip)
//
// ilock() is separate from iget() so that system calls can
// get a long-term reference to an inode (as for an open file)
// and only lock it for short periods (e.g., in read()).
// The separation also helps avoid deadlock and races during
// pathname lookup. iget() increments ip->ref so that the inode
// stays in the table and pointers to it remain valid.
//
// Many internal file system functions expect the caller to
// have locked the inodes involved; this lets callers create
// multi-step atomic operations.
//
// The itable.lock spin-lock protects the allocation of itable
// entries. Since ip->ref indicates whether an entry is free,
// and ip->dev and ip->inum indicate which i-node an entry
// holds, one must hold itable.lock while using any of those fields.
//
// An ip->lock sleep-lock protects all ip-> fields other than ref,
// dev, and inum.  One must hold ip->lock in order to
// read or write that inode's ip->valid, ip->size, ip->type, &c.
//
// 本来 "inode" 是一个宽泛概念，但是在内存中需要缓存磁盘的 "inode",  缓存就要有结构体
// xv6 的命名中，inode 结构体和实际在磁盘存储的 "inode" 数据结构并不一样
// 就像磁盘块的缓存，实际在内存中时 buf->data. 但是引用块缓存还要涉及引用计数等其他信息
// 所以块缓存和它所需要的其他运行时信息被一起放在 buf 结构体中
// 磁盘的 inode 数据结构在内存的缓存也一样
//
// * dinode 结构体和磁盘中的 inode 数据结构相同，就是磁盘 inode 的 copy
// * inode 结构体包含 dinode，除此之外还有该 dinode 在内核运行时的引用计数 ref 等仅在内存使用的数据
// 
// 所以在谈论 "inode" 时，需要区分是宽泛的 “inode” 概念，还是 inode 结构体。约定
// 
// * "inode" 指抽象的 inode 概念
// * “磁盘中的 inode” 指磁盘中实际存储的 inode 格式。在内存中对应 dinode 结构体格式
//    但 dinode 结构体仅在从磁盘读取，然后当成解释方法将 block 转换成 dinode 时使用。见 ialloc()
//    实际 inode 在内存的缓存，存储在 inode 结构体中
//    inode 结构体中还包含其他例如 inode->ref 等只在读到内存中后才需要用到的 filed
// * “内存中的 inode” 指磁盘中的 inode 在内存的 copy
//    磁盘中的 inode 即 dinode 的各个字段，都直接单独缓存在 inode 结构体中
// * “itable 表项”，”inode 结构体“ 
//    指磁盘中的 dinode 在内存缓存的结构体：inode 结构体，包含 dinode 的 copy
//    存在 inode 数组中。
// 
// 使用缓存时，不是直接暴露 dinode, 而是把 dinode 放在 inode 结构体内
// 仅向外暴露 inode 结构体，需要获取缓存时先获取 inode 结构体
//
// [bCache 和 inode 层次的重要的理念是]
// * 分离对 [块缓存] 和 [inode缓存] 的 引用计数ref 和 锁。
//   这么做可以避免 buf 和 inode 的缓存在被频繁使用的时间段内被替换掉
//   否则一旦某个线程释放锁，inode 或 buf 内容就可能被替换成磁盘中其他 inum 或 blockno 的内容
//   马上想要使用对应 inum、blockno 的线程尝试获取其缓存时，就又要重新从磁盘读取到缓存
//
//   iget() 仅获取表项，引用计数 + 1，不获取 inode->sleeplock.
//   iput() 引用计数 - 1，不释放锁. 
//   ilock() 获取 inode->sleeplock
//   iunlock() 释放 inode->sleeplock
//  
//   bget() 在获取 buf->sleeplock 前，引用计数 + 1
//   brelse() 释放 buf->sleeplock 后, 引用计数 - 1
// 
//   buf 和 inode 都是持有引用计数时可以不持有锁，只是表示后续要用
//   -- inode 引用计数的增减和锁的获取释放完全分离
//   不想获取 inode 内容时，iget() 后可以直接 iput() 而，不获取锁
//   例如只是想看看某个目录内有无某个文件，而不想真正访问 inode 内容的时候
//   但是，即使 iget() iput() 外部逻辑不需要锁，但是 iput() 内部逻辑仍需要获得锁，以访问 inode 内容
//   -- buf 引用计数增加说明在 bget() 尝试获取锁，而不是已获得锁
//   inode 在暂时不会再用时释放锁，假设后续还可能尝试获取锁来使用，仍然保持引用计数
//   buf 释放锁就假设不再引用，引用计数 - 1
//   但是 buf 和 inode 都遵守
//   同一时间只有一个线程持有锁，只能有一个线程正使用 "引用", 即指针
//   同一时间引用计数可 > 1
// 
//  * 分离了 [inode 缓存的表项] 和 [inode 缓存的内容]
//    iget() 只是使 inode 缓存表项的 inode->inum 为指定 inum
//    然后通过引用计数，确保该 inode->inum 在被 iget() 设置后就不能被替换
//    以此来实现 iget() 只是为指定 inum [预定] 一个 inode 缓存的表项
//    而不是真的获取了该 inum 对应 inode 的内容 
//    iget() 返回的 inode 无所谓 inode 内容是否确实是该 inum 的实际 inode 内容
//    在持有锁时才根据 inode->calid 确定是否实际是 inum 对应的 inode 内容
//    如果不是，就检查磁盘中该 inum 是否已分配，已分配就加载 inode 内容
//    inode->valid 表示该缓存是否已经加载其 inode 号 inode->inum 对应的内容    
//
// * 分了两类内容
//   1. 缓存的目标内容，即磁盘中的内容
//      buf.data、inode.type, inode.addrs 等
//      inode，buf 结构体本身相关的过程: 读写磁盘块、磁盘内 inode 的内容
//   2. 只在磁盘内容加载到内核内存后，在缓存层面才需要用到、有意义的 filed的内容
//      有一些被缓存的内容相关的，但只在读到内存中后才需要、
//      例如 buf.refcnt、inode.ref
//      这些 filed 不是被缓存的目标 filed，但是放在内存的同一个结构体中
//      所以缓存结构体和被缓存内容的结构不同
//  
// * 根据访问内容的不同，有两类相关过程和锁
//   1. 读写缓存的内容本身的过程。通常要记录到日志，同步磁盘内容到内存（或者相反）
//   2. 在缓存层面，对于读写缓存内容之外，需要
//      分配缓存结构体：buf、inode；
//      追踪引用计数： buf.refcnt、inode.ref
//      等...
//   这两类过程不是互相互斥
//   访问某inode内容本身时，有其他线程需要访问 inode
//   它的引用计数可以在访问它的缓存内容的同时增加，两类过程直接没有同步要求
//   所以分别用两类不同的锁
//   1. 缓存表的锁，全局只有一个. 同时只能有一个线程访问缓存表
//   2. 每个缓存的 buf 和 inode 都有各自的锁
//      仅和同样访问自己的线程保持同步
// 
// * 如果要同时访问两类内容，锁序是先外部的缓存表的锁，在 buf 或 inode 自己的锁
//
// [对于 inode layer]
// * 很多函数必须在持有锁的前提下调用
//   函数假设自己在执行时已经持有锁
//
// * 只要涉及到写磁盘的函数，就应该在事务内调用
//   另外，看起来只是读文件的系统调用，也只是针对文件数据块来说只读
//   在读文件数据块的过程中，也可能会写文件的元数据，[所以读文件的函数也要在事务内]
//   很多函数是假设自己在事务内被调用的（begin_op 和 end_op 之间）
//   例如 iput()
//   假设文件在被几个线程引用的时间内，文件被某个线程删除 nlink == 0
//   但是仍有线程要引用它，它的引用计数不为 0。也就是还存在已经 "打开" 该文件的其他线程
//   这种情况下，即使文件被删除
//   删除文件线程的 iput 也不会释放缓存内及磁盘内的 inode，还有文件的数据块
//   以让其他线程继续访问该文件，直到最后一个 iput()
//   inode->nlink == 0 的情况下，最后一个 iput() 的线程才会释放它
//   释放缓存和磁盘的 inode 及数据块
//   哪怕最后一个访问文件的线程是只读文件
//   它的 iput() 也会写磁盘上的 inode，使它的 inode->type = 0
struct {
  struct spinlock lock;
  struct inode inode[NINODE];
} itable;

void
iinit()
{
  int i = 0;
  
  initlock(&itable.lock, "itable");
  for(i = 0; i < NINODE; i++) {
    initsleeplock(&itable.inode[i].lock, "inode");
  }
}

static struct inode* iget(uint dev, uint inum);

// Allocate an inode on device dev.
// Mark it as allocated by  giving it type type.
// Returns an unlocked but allocated and referenced inode,
// or NULL if there is no free inode.
// 在 dev 上分配一个 dinode，标记它为已占用
// 并马上，使用 iget() 从磁盘读取该 dinode
// iget() 读取刚在磁盘分配的 dinode，在 itable 绑定一个 inode 表项
// 使该 dinode 在内存的缓存 inode 的 inode->ref=1, inode.valid=0
// 最后 ialloc() 返回 iget() 返回的 inode 结构体引用。
// dinode 的内容等到 ilock() 时才读取到该 dinode 的缓存结构体 inode 
//
// 磁盘存储的 inode 结构格式为 dinode
// 磁盘存储的 dinode 是否被分配，用磁盘中的 dinode->type 是否非 0 决定
// 在磁盘中寻找一个空闲的 dinode，设置 dinode->type=type 来分配一个 dinode
// 并返回分配的 dinode 的 inum，inum 由 dinode 在磁盘存储的顺序来，是固定的
// 就像 bitmap 中的 bit 顺序决定每个 bit 对应的 blockno 
//
// 分配完后，在 itable 中绑定一个表项，并返回该表项（inode 结构体）
struct inode*
ialloc(uint dev, short type)
{
  int inum;
  struct buf *bp;
  struct dinode *dip;
 // ialloc can be sure that some other process does not simultaneously see that the
 // inode is available and try to claim it.
  for(inum = 1; inum < sb.ninodes; inum++){
    // 读取 inum 对应的磁盘中的 inode 所在的块
    // dinode 所在的块号 =（inode号）/（每块包含的dinode数） +（inode 区域的起始块号）
    bp = bread(dev, IBLOCK(inum, sb));
    // 前面拿到了块的内容，现在就可以在该块中获取 dinode 了
    //
    // dinode 在内存的地址 = 块在内存的起始地址为基址 + dinode大小 * dinode 在其所属块中的序号 
    // dinode 在其所属块中的序号 = inum%(每块包含的dinode数)
    //
    // bp->data 的值是整个块在内存中的 copy 的基址
    // bp->data 类型是 uchat 数组，所以是用 uchar 地址解释
    // 如果把该地址转为用 dinode 地址解释
    // 那么 "+" 操作就可以将基址按 dinode 大小为单位增加。
    // 基址 +（dinode大小 * dinode 在其所属块中的序号）= dinode 在内存中的地址 
    dip = (struct dinode*)bp->data + inum%IPB;
    if(dip->type == 0){  // a free inode
      memset(dip, 0, sizeof(*dip));
      dip->type = type;
      log_write(bp);   // mark it allocated on the disk
      brelse(bp);
      return iget(dev, inum);
    }
    brelse(bp);
  }
  printf("ialloc: no inodes\n");
  return 0;
}

// Copy a modified in-memory inode to disk.
// Must be called after every change to an ip->xxx field
// that lives on disk.
// Caller must hold ip->lock.
// 把在内存缓存的 inode 更新同步到磁盘
// 只是把更新记录在 logHeader 中记录
// 后续要在用 end_op 提交事务后才真正写到磁盘
//
// inode 是比块还小的单位
// 但块设备驱动约定只能以块大小为单位读写
// 所以不能细粒度到以精确的 inode 地址读写
// 所以文件系统的所有对块的修改，都只能先读取整个块，在整个块中只修改所需的部分，在以整个块写回磁盘
void
iupdate(struct inode *ip)
{
  struct buf *bp;
  struct dinode *dip;

  bp = bread(ip->dev, IBLOCK(ip->inum, sb));
  dip = (struct dinode*)bp->data + ip->inum%IPB; // 拿到 dinode 的内存基址
  // 把 dinode 缓存的内容，同步到刚从磁盘读取到的，在 buf 中的 dinode 副本
  dip->type = ip->type;
  dip->major = ip->major;
  dip->minor = ip->minor;
  dip->nlink = ip->nlink;
  dip->size = ip->size;
  // inode->addrs 是数据块号数组
  // 用 memmove 从 dinode 缓存的 dinode->addrs 连续写到 buf 的 dinode—>addrs
  memmove(dip->addrs, ip->addrs, sizeof(ip->addrs));
  // 在事务头 logHeader 记录 inode 所在块的更新
  log_write(bp);
  // 释放 inode 所在块的 buf
  brelse(bp);
}

// Find the inode with number inum on device dev
// and return the in-memory copy. Does not lock
// the inode and does not read it from disk.
// iget(inum) 返回 inum 对应的 dinode 在内存中的缓存结构体 inode，即在缓存表中的表项
// iget(inum) 不管该 inum 对应的 inode 在是否有和文件或目录绑定
// 即使该 inum 对应的 inode 没有绑定到文件或目录，即没有被分配。也缓存该 inode
// 如果 itable 中有这个 inum 对应的 dinode 缓存，则直接返回包含它的 inode
// 如果 itable 中没有该缓存，则绑定一个 inode 并返回
// 但是不会从磁盘读取实际 dinode 的内容, 只是返回刚绑定的表项结构体引用
// dinode 内容由 ilock() 读取
// 不管这个 inum 对应的 dinode 有没有缓存在内存，表项是可以先绑定的
//
// 类似 bCache
// 如果未命中， bget(blockno) 返回的 buf.valid = 0. buf.data 的数据不是 blockno 的数据
// 在 bread() 才从磁盘读取块数据到 buf.data
// 如果命中， bget(blockno) 直接返回 buf
static struct inode*
iget(uint dev, uint inum)
{
  struct inode *ip, *empty;

  // 避免同时写 inode->ref
  acquire(&itable.lock);

  // Is the inode already in the table?
  empty = 0;
  for(ip = &itable.inode[0]; ip < &itable.inode[NINODE]; ip++){
    if(ip->ref > 0 && ip->dev == dev && ip->inum == inum){
      ip->ref++;
      release(&itable.lock);
      return ip;
    }
    if(empty == 0 && ip->ref == 0)    // Remember empty slot.
      empty = ip;
  }

  // Recycle an inode entry.
  // 没在 itable 中找到 inum 对应的 inode，itable (inode 数组) 中没有空闲表项
  if(empty == 0)
    panic("iget: no inodes");

  // 没在 itable 中找到 inum 对应的 inode，itable (inode 数组) 中存在空闲表项
  // 就把该空闲表项和 inum 绑定，并设置仅在内存中使用的元数据
  // 注意只是绑定表项和 inum
  // iget(inum) 不把磁盘中的 inum 对应的实际 inode 数据结构（对应内存的 dinode）加载到表项的 inode 中
  // iget(inum) 只是返回 inum 对应的表项，即 inode 结构体
  // 返回的 inode 结构体如果是刚被记录的，inode->dinode 就还没加载好 inum 的 dinode
  // 设置 inode->valid = 0
  ip = empty;
  ip->dev = dev;
  ip->inum = inum;
  ip->ref = 1;
  ip->valid = 0;
  release(&itable.lock);

  return ip;
}

// Increment reference count for ip.
// Returns ip to enable ip = idup(ip1) idiom.
// inode 引用 + 1
struct inode*
idup(struct inode *ip)
{
  acquire(&itable.lock);
  ip->ref++;
  release(&itable.lock);
  return ip;
}

// Lock the given inode.
// Reads the inode from disk if necessary.
// 获取 inode->sleeplock
// 如果该 inode 缓存还未从磁盘同步内容，就从磁盘读取内容
void
ilock(struct inode *ip)
{
  struct buf *bp;
  struct dinode *dip;

  // iget(inum) 从 itable 获取 inode 时，inode->ref++
  // ilock(inode) 在 iput() inode->ref-- 之前调用
  // 所以调用约定逻辑如此，ilock(inode) 不应该出现 ref < 1；
  // 和 inode 这个表项为 0 的情况（C 为 itable 分配了好了 inode 所需空间，且 iget 不返回 0） 
  // 所以使用 "断言"  
  if(ip == 0 || ip->ref < 1)
    panic("ilock");

  acquiresleep(&ip->lock);

  // 如果该 inode 缓存，inode 结构体，还没有从磁盘读取实际 inode 内容
  // 类似 bget() 返回的 buf 还没有从磁盘读取磁盘块的内容
  // 则从磁盘读取 dinode 同步到缓存的 indoe 结构体中
  // 但是和块缓存不同的是
  // bCache只负责缓存读写磁盘块，无所谓该块有没有被分配
  // inode 层的逻辑，应该只读取已经分配的 dinode
  // 如果读取到的 dinode 没被分配，dinode->type == 0, 就主动崩溃方便调试 
  if(ip->valid == 0){
    bp = bread(ip->dev, IBLOCK(ip->inum, sb));
    dip = (struct dinode*)bp->data + ip->inum%IPB;
    ip->type = dip->type;
    ip->major = dip->major;
    ip->minor = dip->minor;
    ip->nlink = dip->nlink;
    ip->size = dip->size;
    memmove(ip->addrs, dip->addrs, sizeof(ip->addrs));
    brelse(bp);
    ip->valid = 1;
    if(ip->type == 0)
      panic("ilock: no type");
  }
}

// Unlock the given inode.
// 释放 inode->sleeplock
void
iunlock(struct inode *ip)
{
  // 理想的逻辑是，indoe 持有锁时 inode != 0, inode->ref >= 1 该 inode 应该是正被引用的
  if(ip == 0 || !holdingsleep(&ip->lock) || ip->ref < 1)
    panic("iunlock");

  releasesleep(&ip->lock);
}

// Drop a reference to an in-memory inode.
// If that was the last reference, the inode table entry can
// be recycled.
// If that was the last reference and the inode has no links
// to it, free the inode (and its content) on disk.
// All calls to iput() must be inside a transaction in
// case it has to free the inode.
// 
// iput() 将 inode 引用计数 - 1. 并包含一些回收工作
//
// [inode 释放存在的问题]
// 如果 inode 对应的文件在引用计数不为 1 的情况下，有线程要删除该文件，使其 inode->nlink == 0
// 即除了删除线程，还有其他线程后续想要访问该文件
// 这时候面临的是 inode 层和上层用户层逻辑设计的问题
//  * 可以选择在仍有后续要访问该文件的线程时，不允许删除
//  * 也可以先使 nlink = 0, 但是仍然不改动该文件的数据块、inode 缓存
//    每个线程用 iput() 减少引用计数时，检查释放满足 nlink==0 且 自己是最后一个 iput() 的线程
//    如果满足，才使该文件在内存缓存的 inode.valid = 0、磁盘内的 inode->type = 0、释放数据块
// 对于后者，如果在 nlink = 0，且最后一个线程调用 iput() 前发生系统崩溃
// 就会发生以下情况
//  * 已经没有目录在绑定该 inode
//    不能再通过目录（上层访问该文件 inode 和数据块唯一方式）访问该文件
//    也就是该文件对于最上层（用户）来说已经被删除
//  * 磁盘上的 inode、数据块仍然是 “占用” 状态
// 也就是该文件还在占用着空间，却没有再访问、消除它的机会
// 解决办法有
//  * iput() 将 nlink == 0, ref >= 1 的 inode 加入到 superblock
//    系统启动时检查这些 inode 进行清理
//  * 扫描全部 inode
// ----------------------------------------------------------------------------
// 
// 一个线程持有一个 inode 引用，就可以多次用该 inode，所以 inode->ref 是针对引用的线程数的
// inode->ref - 1 == 0 说明除了执行 iput() 的线程，没有其他线程正引用该 inode
//
// 如果 inode->ref - 1 != 0, 当前还有线程在用它， 无论该 inode 是否有被目录绑定
// 那么 itable 和 磁盘内的 dinode 都不应该被释放，避免内存的 inode 或磁盘内的 dinode 被覆盖
//
// 如果 inode->ref - 1 == 0, 当前没有线程在用它，但是磁盘 dinode->nlink != 0, 还有目录绑定它
// 那么磁盘的 dinode 不该被释放，后续还会使用到它
// itable 缓存的 inode 也没必要马上释放，既然目录还绑定它，后续可能还会用到它
// 所以 ref == 0 时，itable 的 inode 表项没有必要马上清除，inode.valid 仍为 1 表示还可使用
// 但是由于没有线程引用它了，所以允许 ref == 0 的 inode 表项内容被替换为其他 inode
// 所以一旦缓存大小不够需要替换，就可以替换掉该 ref == 0 的 inode
// 替换时 iget() 先使替换的 inode.ref = 1（使其不再能被其他线程替换）
// 再使 inode.valid = 0 （表示需要从磁盘同步）
// 在 ilock() 时，检查到 inode.valid == 0，从磁盘读取新 dinode 的内容 
// 
// 如果该序号对应的 inode 没有被任何目录引用, 
// 且所有线程都没有正引用该 inode
// 说明直到被新目录、文件绑定，后续不会再有线程再引用这个序号的 dinode 了
// 那么可以在内存 itable，和磁盘中都释放它；允许该位置（序号）的 dinode 被分配
//
// iput() 必须在事务中，因为可能调用 iupdate() 写磁盘，iupdate() 必须在事务中
void
iput(struct inode *ip)
{
  acquire(&itable.lock);

  if(ip->ref == 1 && ip->valid && ip->nlink == 0){
    // inode has no links and no other references: truncate and free.

    // ip->ref == 1 means no other process can have ip locked,
    // so this acquiresleep() won't block (or deadlock).
    acquiresleep(&ip->lock);

    release(&itable.lock);

    itrunc(ip); // 标记 inode 的所有数据块为未分配
    ip->type = 0; // 标记该 inode 未分配
    iupdate(ip); // 把 inode 的更新写回磁盘的 inode 区域
    // 因为该文件/目录 inode 被从磁盘中解除了分配
    // 意味着该 inum 的 inode 可以被重新分配给新文件
    // 那么缓存中该 inum 的老内容也应该被废弃
    // 通过标记 inode->valid = 0, 表示该 inode 插槽还未被加载磁盘中 inode 的数据
    // 这样在下一次 iget(inum), ilock(inode) 就要从磁盘重新加载该 inum 的新数据
    //
    // 如果 inum 对应的 inode 缓存的内容和实际不一致的情况下
    // 如果不设置 inode->valid = 0
    // 而缓存中该 inum 的 inode->valid == 1,
    // iget(inum) 从缓存又得到该 inum 的 inode 后
    // ilock(inode) 检查 inode->valid == 1 不从磁盘加载数据
    // 就导致缓存中 inum 对应的 inode 和实际的磁盘中 inum 对应的 inode 内容不一致
    ip->valid = 0;

    releasesleep(&ip->lock);

    acquire(&itable.lock);
  }

  ip->ref--;
  release(&itable.lock);
}

// Common idiom: unlock, then put.
// 后续不再需要该 inode 时
// 先释放 inode->sleeplock
// 再将 inode->ref--
// 
// 如果只是暂时不需要该 inode，只要 iunlock() 释放 inode->sleeplock
// 如果后续还要重新尝试获得 inode->sleeplock 使用该 inode，就不要将引用计数 - 1
void
iunlockput(struct inode *ip)
{
  iunlock(ip);
  iput(ip);
}

// Inode content
//
// The content (data) associated with each inode is stored
// in blocks on the disk. The first NDIRECT block numbers
// are listed in ip->addrs[].  The next NINDIRECT blocks are
// listed in block ip->addrs[NDIRECT].

// Return the disk block address of the nth block in inode ip.
// If there is no such block, bmap allocates one.
// returns 0 if out of disk space.
// 返回 inode 的第 bn 个数据块在整个磁盘(而不是数据块区域)的块地址
// 用于通过 bread(dev, 对于整个磁盘的磁盘块号) 读取数据块内容
// 如果第 bn 个数据块没有被分配磁盘块, 就分配一个磁盘块给第 bn 个数据块
// 把新数据块地址写进 inode->addrs 或间接块的对应 uint 位置中, 再返回新分配的块地址
static uint
bmap(struct inode *ip, uint bn)
{
  uint addr, *a;
  struct buf *bp;

  if(bn < NDIRECT){
    if((addr = ip->addrs[bn]) == 0){
      addr = balloc(ip->dev);
      if(addr == 0)
        return 0;
      ip->addrs[bn] = addr;
    }
    return addr;
  }
  bn -= NDIRECT;

  if(bn < NINDIRECT){
    // Load indirect block, allocating if necessary.
    if((addr = ip->addrs[NDIRECT]) == 0){
      addr = balloc(ip->dev);
      if(addr == 0)
        return 0;
      ip->addrs[NDIRECT] = addr;
    }
    bp = bread(ip->dev, addr); // 读取间接块
    a = (uint*)bp->data; // 间接块地址作为值，用 *uint 解释. 
    // 在间接块中索引指定的第 bn 个数据块号的数据块地址
    if((addr = a[bn]) == 0){ // 将 a 的值解释为 *uint 来操作，表达式操作的结果值为 uint
      // 如果第 bn 个数据块没有分配，就为他分配
      addr = balloc(ip->dev);
      if(addr){ // 分配成功，就写入间接块
        a[bn] = addr;
        log_write(bp); // 将间接块的更新写入日志
      }
    }
    brelse(bp);
    return addr;
  }

  panic("bmap: out of range");
}

// Truncate inode (discard contents).
// Caller must hold ip->lock.
// 清空、释放文件所有数据块(文件内容)，设置 inode->size = 0
// 只是清空文件内容，文件(inode)本身仍保留
// 假设持有 inode->sleeplock.
// 要将更新的 inode 写回磁盘，用于写回磁盘的 iupdate() 也假设持有锁
void
itrunc(struct inode *ip)
{
  int i, j;
  struct buf *bp;
  uint *a;

  // 解除 addrs 和数据块的绑定
  // 释放 addrs 中的所有数据块
  for(i = 0; i < NDIRECT; i++){
    if(ip->addrs[i]){
      bfree(ip->dev, ip->addrs[i]);
      ip->addrs[i] = 0;
    }
  }

  // 如果有间接块，那么
  // 释放间接块中指向的所有数据块
  // 释放间接块本身
  // 解除 addrs[NDIRECT] 和间接块的绑定 
  if(ip->addrs[NDIRECT]){
    bp = bread(ip->dev, ip->addrs[NDIRECT]);
    a = (uint*)bp->data;
    for(j = 0; j < NINDIRECT; j++){
      if(a[j])
        bfree(ip->dev, a[j]);
    }
    brelse(bp);
    bfree(ip->dev, ip->addrs[NDIRECT]);
    ip->addrs[NDIRECT] = 0;
  }

  ip->size = 0;
  iupdate(ip);
}

// Copy stat information from inode.
// Caller must hold ip->lock.
void
stati(struct inode *ip, struct stat *st)
{
  st->dev = ip->dev;
  st->ino = ip->inum;
  st->type = ip->type;
  st->nlink = ip->nlink;
  st->size = ip->size;
}

// Read data from inode.
// Caller must hold ip->lock.
// If user_dst==1, then dst is a user virtual address;
// otherwise, dst is a kernel address.
// 读取文件（目录）数据块的指定位置（偏移量）的内容到指定地址
// * dst: 读取到的起始地址
// * user_dst：读取到的起始地址 dst 是否是用户地址. 若是，就在 c->proc->pagetable 拿到物理地址
// * off: 被读取文件的字节偏移量（被读取文件相对文件所有内容的起始地址）
// * n: 从偏移量 off 开始读取的字节数
// 返回最后实际读取的字节数
int
readi(struct inode *ip, int user_dst, uint64 dst, uint off, uint n)
{
  uint tot, m;
  struct buf *bp;

  if(off > ip->size || off + n < off)
    return 0;
  if(off + n > ip->size)
    n = ip->size - off;

  for(tot=0; tot<n; tot+=m, off+=m, dst+=m){
    uint addr = bmap(ip, off/BSIZE);
    if(addr == 0)
      break;
    bp = bread(ip->dev, addr);
    m = min(n - tot, BSIZE - off%BSIZE);
    if(either_copyout(user_dst, dst, bp->data + (off % BSIZE), m) == -1) {
      brelse(bp);
      tot = -1;
      break;
    }
    brelse(bp);
  }
  return tot;
}

// Write data to inode.
// Caller must hold ip->lock.
// If user_src==1, then src is a user virtual address;
// otherwise, src is a kernel address.
// Returns the number of bytes successfully written.
// If the return value is less than the requested n,
// there was an error of some kind.
// 将内存指定地址的内容写到文件的指定位置
// writei 在写的过程中，如果偏移量（文件光标）超过文件数据块大小，会为文件增加数据块
// 需要包含在事务内调用
// 需要在函数外持有锁 inode->sleeplock
// 块的锁会在该函数内获取和释放
//
// 数据复制方向 src -> dst 
// 对于 readi，src 是磁盘，dst 是内存（用户或内核空间）
// 对于 writei，src 是内存（用户或内核空间），dst 是磁盘
// readi，writei 都要指定在文件内开始读写的位置 off
int
writei(struct inode *ip, int user_src, uint64 src, uint off, uint n)
{
  uint tot, m;
  struct buf *bp;

  if(off > ip->size || off + n < off)
    return -1;
  if(off + n > MAXFILE*BSIZE)
    return -1;

  for(tot=0; tot<n; tot+=m, off+=m, src+=m){
    // bmap() 会为未分配磁盘块的第 n 个数据块分配一个磁盘块
    // 并把块地址写进 inode->addrs 或 inode->addrs[INDIRECT] (间接块) 的对应位置
    // 并返回这个新的数据块的块地址
    // 这就为 writei 在需要扩展文件数据块时提供了方便
    uint addr = bmap(ip, off/BSIZE); // 更新 bitsmap 缓存块；更新 inode 缓存表项
    if(addr == 0)
      break;
    bp = bread(ip->dev, addr); // 更新 inode 数据块的缓存块
    m = min(n - tot, BSIZE - off%BSIZE);
    if(either_copyin(bp->data + (off % BSIZE), user_src, src, m) == -1) {
      brelse(bp);
      break;
    }
    // 块在使用期间被锁和引用保证不会被替换，读取到的都是最新的，且各个线程是同步读写
    // 但是释放块之后，块缓存就可能被替换，导致之前的更新丢失
    // 办法是在释放前，先保证完全更新到低层的磁盘
    // 但是这样开销过大了
    // 一种方法是，通过引用计数+1, 让块缓存不被替换
    // 直到 log layer 把多个事务更新的块，一起同步到磁盘
    // * 块使用期间，块缓存不会被替换，保证访问到的是最新内容
    // * 块被更新，log_write，relese 直到真正写入磁盘期间，块缓存不会被替换，保证访问到的是最新内容
    // * 写入磁盘之后，再次访问时
    //   要么还在缓存，属于前两种情况。
    //   要么从磁盘拉取新内容，保证访问的是最新内容。
    log_write(bp); // 在释放前，用 log_write 把块缓存 pin（固定） 在了缓存
    brelse(bp); // 可以放心释放更新过的块，而不用担心更新由于块缓存替换而丢失
  }

  if(off > ip->size)
    ip->size = off;

  // write the i-node back to disk even if the size didn't change
  // because the loop above might have called bmap() and added a new
  // block to ip->addrs[].
  // 更新 inode 缓存块
  // 因为 inode 缓存表项已经被更新过，释放后可能会因为 inode 缓存替换而丢失更新的内容
  // 在释放 inode 前应该同步更新到块缓存
  // 保证 inode 表项被替换后，ilock 仍然可以通过块缓存获取最新 inode 内容
  // 但是，块缓存也是缓存，也有担心被替换而丢失更新的问题
  // 但这是块缓存的问题，block Cache 对快缓存更新被丢失的解决方法见上（brelse() 前的相关问题）
  iupdate(ip);

  return tot;
}

// Directories

int
namecmp(const char *s, const char *t)
{
  return strncmp(s, t, DIRSIZ);
}

// Look for a directory entry in a directory.
// If found, set *poff to byte offset of entry.
// 检查目录中是否存在该目录项 （name，inum）
// 如果有，就为该 inum 预定一个 inode 表项，并返回该表项. 即 iget(inum)
// caller 必须持有目录 dp 的锁 dp->sleeplock
struct inode*
dirlookup(struct inode *dp, char *name, uint *poff)
{
  uint off, inum;
  struct dirent de;

  if(dp->type != T_DIR)
    panic("dirlookup not DIR");

  for(off = 0; off < dp->size; off += sizeof(de)){
    if(readi(dp, 0, (uint64)&de, off, sizeof(de)) != sizeof(de))
      panic("dirlookup read");
    if(de.inum == 0)
      continue;
    if(namecmp(name, de.name) == 0){
      // entry matches path element
      if(poff)
        *poff = off;
      inum = de.inum;
      // 要访问的 inode 可能同时在多个目录被引用. 会产生导致死锁的可能
      // 假设有两个目录项 (/usr/f.c, inum1), (/root/f.c, inum1). inum1 对应 inode1
      // 假设 iget() 返回的 inode 持有锁
      // 1. 线程 A 先获取了 /usr/f.c，持有 inode1->sleeplock
      // 2. 线程 B 访问 /root/f.c，试图获取 inode1->sleeplock
      // 3. 线程 B 在锁上睡眠
      // 4. 线程 A 在持有 inode1 的情况下，试图访问目录 /root
      // 5. 线程 A 持有 f.c 的锁，试图获取目录 /root 的 inode 锁
      //    线程 B 持有目录 /root 的锁，试图获取 f.c 的锁
      //    造成了死锁 
      // 所以这也是不让 iget() 获取 inode 锁的原因
      // iget() 不会获取锁的情况下
      // iget() 就能确保返回，并在 dirlookup 的 caller 在拿到 inode 以后
      // 获取 inode 锁之前释放目录的锁
      // 保证同一时间只持有一个锁
      // The caller can unlock dp and then lock ip
      // ensuring that it only holds one lock at a time
      // 另外，iget() 开始已经不需要目录锁了
      // 那么在 iget() 返回的 inode 持有锁的情况下
      // 也可以在 iget() 前释放目录锁来避免死锁？
      return iget(dp->dev, inum);
    }
  }

  return 0;
}

// Write a new directory entry (name, inum) into the directory dp.
// Returns 0 on success, -1 on failure (e.g. out of disk blocks).
int
dirlink(struct inode *dp, char *name, uint inum)
{
  int off;
  struct dirent de;
  struct inode *ip;

  // Check that name is not present.
  if((ip = dirlookup(dp, name, 0)) != 0){
    iput(ip);
    return -1;
  }

  // Look for an empty dirent.
  for(off = 0; off < dp->size; off += sizeof(de)){
    if(readi(dp, 0, (uint64)&de, off, sizeof(de)) != sizeof(de))
      panic("dirlink read");
    if(de.inum == 0)
      break;
  }

  strncpy(de.name, name, DIRSIZ);
  de.inum = inum;

  // 把目录结构体写进目录 inode 数据区域（包含所有数据块）的指定偏移量位置
  // writei() 利用 bmap() 自动扩展文件大小
  // 如果目录的当前占用的所有数据块都已经被目录项占满
  // 上面的的 loop 就会使 off 停在大于等于 dp->size 的位置
  // 这个位置就是新目录项的理想偏移位置
  // writei 会扩展数据块, 可以在新的数据块内, 从偏移位置 off 开始写新的目录项
  // 
  // 如果目录的数据区域已经写满, 即 inode->addrs 和间接块内指向的所有数据块都被写满
  // 那么 writei 返回的实际写入字节数会 != sizeof(de)
  // 说明目录 inode, 即目录文件（内容）大小已经达到最大文件大小, 无法再写入了
  if(writei(dp, 0, (uint64)&de, off, sizeof(de)) != sizeof(de))
    return -1;

  return 0;
}

// Paths

// Copy the next path element from path into name.
// Return a pointer to the element following the copied one.
// The returned path has no leading slashes,
// so the caller can check *path=='\0' to see if the name is the last one.
// If no name to remove, return 0.
//
// Examples:
//   skipelem("a/bb/c", name) = "bb/c", setting name = "a"
//   skipelem("///a//bb", name) = "bb", setting name = "a"
//   skipelem("a", name) = "", setting name = "a"
//   skipelem("", name) = skipelem("////", name) = 0
//
// 复制路径的首个（下一个）元素（目录或文件）名到 name
// 返回移除首个元素后的，更短的新路径
// 此函数用来遍历 path
// 一些边界情况：
//  * 路径 path 中仅有一个元素 a, 则首个（下一个元素）为 a，新路径为 path == "" == 0
//  * 路径 path == "" == 0, 就没有首个（下一个元素）了，返回 0
// （*path) == 0 和 path == 0 是不一样的
// 前者是 path 变量的值作为地址指向的位置的值，是 0。即字符串的值 == 0 == ""
// 后者是 path 变量的值 == 0。称为 "path 是一个 空指针"
static char*
skipelem(char *path, char *name)
{
  char *s;
  int len;

  while(*path == '/')
    path++;
  if(*path == 0)
    return 0;
  s = path;
  while(*path != '/' && *path != 0)
    path++;
  len = path - s;
  if(len >= DIRSIZ)
    memmove(name, s, DIRSIZ);
  else {
    memmove(name, s, len);
    name[len] = 0;
  }
  while(*path == '/')
    path++;
  return path;
}

// Look up and return the inode for a path name.
// If parent != 0, return the inode for the parent and copy the final
// path element into name, which must have room for DIRSIZ bytes.
// Must be called inside a transaction since it calls iput().
// 如果 nameiparent == 0
// 返回路径末目录/文件的 inode、name 设置为末目录名
// 返回文件 name 对应的 inode
// path 中的每个元素都需要是目录名，不能是非目录
// 如果 path 中的任意一个元素不是目录而是文件，则无意义，返回 0
//
// 如果 nameiparent 非 0
// 就返回 path 的末目录的父目录 inode, name 设置为末目录名
static struct inode*
namex(char *path, int nameiparent, char *name)
{
  struct inode *ip, *next;

  // 如果 path 自身就是是根目录起始的绝对路径
  // 就从根目录的 inode 开始，沿着 path 寻找 name 目录项
  if(*path == '/')
    ip = iget(ROOTDEV, ROOTINO);
  else
  // 否则，就从进程的 "当前目录" 的 inode 开始
  // 沿着 path 寻找 name 的目录项
    ip = idup(myproc()->cwd);

  // 循环直到 path 的末尾之后，或直到遇到某个不是目录的路径元素
  // 根据 path 寻找目录、文件的迭代过程
  // 是一个连续的用 dirlookup 在当前目录下寻找下一路径元素（目录/文件）inode 的过程
  // 1. 初始当前目录 ip 为整个 path 的父目录：根目录（path以'/'开始）或者当前目录（path不以'/'开始）
  // 2. 循环最开始，skipelem 从 path 拿取下一个 path 元素的名称 name
  //    ilock() 当前目录 inode 表项的锁，以及 inode 内容
  // 3. 循环内，dirlookup 在当前目录 inode 寻找并获得下一元素 name 的 inode 表项，增加引用计数
  // 4. 更新当前目录 ip 为下一元素 name 的 inode
  //    回到 2
  // is carefully designed so that if an invocation of namex by one kernel
  // thread is blocked on a disk I/O, another kernel thread looking up a different pathname can proceed concurrently. 
  // namex locks each directory in the path separately so that lookups in different
  // directories can proceed in parallel.
  while((path = skipelem(path, name)) != 0) {
    // 上一次循环通过 dirlookup->iget 获取的 inode 可能刚在缓存被分配
    // 而没有加载 inode 内容
    // 使用 ilock 锁定并加载 inode 内容
    ilock(ip);
    // 该元素不是目录，返回 0
    if(ip->type != T_DIR){
      iunlockput(ip);
      return 0;
    }
    // 如果要求返回 path 的父目录，即 path 末目录的父目录
    // 且当前要寻找的目录元素是最后一个元素，即 path 指向空字符串（字符终止符）
    // 就不用在当前目录 inode 找最后一个元素 name 的 inode 了；
    // 立即返回当前目录的 inode，此时 name 已经是下一元素（此时为末目录）的名称
    if(nameiparent && *path == '\0'){
      // Stop one level early.
      iunlock(ip);
      return ip;
    }
    // 在遍历的当前目录下，寻找下一个目录的 inode
    // 例如 path == "/usr/a/b/c"
    // 第一次循环内
    //  * ip == 根目录 inode 地址（本次循环被搜索的目录 inode）
    //  * name == "usr" 的地址 （本次循环要搜索的，下一个目录名称）
    //  * path == "a/b/c" 的地址 （剩余要被搜索的路径）
    //  * next == "usr" 的 inode 地址 (搜索到的下一个目录 inode)
    // 最后一次循环
    //  * ip == 目录 b 的 inode 地址（本次循环被搜索的目录 inode）
    //  * name == "c" 的地址 （本次循环要搜索的，下一个目录名称）
    //  * path == "" 的地址 （剩余要被搜索的路径）
    //  * next == "c" 的 inode 地址 (搜索到的下一个目录 inode)
    if((next = dirlookup(ip, name, 0)) == 0){
      // 如果在本次循环的目录下找不到下一个目录的 inode
      // 就返回 0；表示错误
      iunlockput(ip);
      return 0;
    }
    // 目录 inode 使用完毕，释放锁
    // 1. 避免下一个目录是 '.' 时， 导致持有锁的前提下获取已持有的当前目录锁，导致死锁
    // 这也体现了把 iget() 和 ilock() 独立开来的好处
    // 如果让 dirlookup() 返回的是已获得锁的 inode
    // 那么在这个例子里，在持有当前目录 inode 锁的基础下
    // 让 dirlookup() 寻找并获得 "." 对应的 inode 锁就会导致 deadlock
    // 2. 如果没有 inode 锁和引用计数
    //    或者在 dirlookup 前执行 iunlockput(), 释放当前目录的 inode 锁并减少引用计数
    //    那么在 dirlookup 前，当前目录就可能被其他线程删除
    //    被 unlink，并在 iput() 彻底清除当前目录数据块、inode->type，的分配
    //    并且该 inum 的 inode 已经是其他文件的 inode
    //    从 inode 缓存拿到的 inode，块缓存拿到的数据块，都已经是其他文件的内容了
    //    所以应该在 dirlookup() -> iget() 拿到引用计数后
    //    确保该inum的inode不会被替换成其他文件的内容后
    //    再释放锁，减少引用计数
    iunlockput(ip);
    ip = next; // 更新当前目录
  }
  // 退出循环后，ip 是末目录/文件的 inode
  
  // 如果 path 一开始就是 '/' 
  // 路径'/' 没有下一路径元素，也没有下一个路径
  // 所以 skipelem() 不会返回下一 path，和设置 name 为下一元素，而是直接返回值 0
  // 所以上面的循环一次都不会执行
  // 在一开始 path （的值）就指向 "/" 的情况下，如果 caller 要求返回 path 的父目录
  // 而 '/' 没有父目录，就返回 0 表示失败
  if(nameiparent){
    iput(ip);
    return 0;
  }

  // 在一开始 path （的值）就指向 "/" 的情况下
  // 如果不要求返回 path 的父目录，那么会按默认返回末目录的 inode，此时就是根目录本身的 inode
  // 但是 name 不会被设置（设置 name 的 skipelem 在 while 直接返回 0）
  return ip;
}

struct inode*
namei(char *path)
{
  char name[DIRSIZ];
  return namex(path, 0, name);
}

struct inode*
nameiparent(char *path, char *name)
{
  return namex(path, 1, name);
}
 