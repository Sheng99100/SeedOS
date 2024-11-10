// Buffer cache.
//
// The buffer cache is a linked list of buf structures holding
// cached copies of disk block contents.  Caching disk blocks
// in memory reduces the number of disk reads and also provides
// a synchronization point for disk blocks used by multiple processes.
//
// Interface:
// * To get a buffer for a particular disk block, call bread.
// * After changing buffer data, call bwrite to write it to disk.
// * When done with the buffer, call brelse.
// * Do not use the buffer after calling brelse.
// * Only one process at a time can use a buffer,
//     so do not keep them longer than necessary.
// 
// 为了缓解磁盘内存访问速度的差异. 在内存中用固定大小的缓存链表, 缓存固定数量的磁盘块副本
// 在内存分配能缓存 NBUF 个 "磁盘块" 的 "缓冲区" 缓存. 一个 buf 存储一个磁盘块
// 硬件的读写单位是 512B 大小 的"扇区"
// "磁盘块" 是 OS 人为的概念, 块大小是扇区大小的整数倍. xv6 为 1024B
//
// 文件系统中 Cache 层的上层以磁盘块为单位, 经过 Cache 层, 逐块访问 "磁盘块" 
// (加引号是因为可能在缓存, 而不是真的从磁盘访问)
// 1. bread(blockno) 接口先获取在内存中缓存的磁盘块 buf
//    无论是读还是写磁盘块, Cache 上层都用 bread(blockno) 先获取在内存中缓存的磁盘块 buf
//    再从 buf 中读取, 或者写入 buf 后再写回磁盘
//    bread(blockno) 保证了返回的一定是已经在内存中缓存好的 buf, 无论一开始缓存是否命中    
// 以下是关于 bread(blockno) 如何确保返回的一定是已经在内存中缓存好的 buf
//    a. 如果缓存命中, bread()->bget() 直接返回 buf. buf.data 存放目标磁盘块数据
//    b. 如果缓存未命中, bread() 从磁盘读到（更新）缓存。具体是
//       bread()->bget() 选择并返回一个要替换的 buf.（使用 "最近最少访问" 的替换策略）
//       修改被替换的 buf 的元数据
//       buf->blockno 改为要缓存的新 blockno
//       buf->valid == 0 表示还未缓存 buf->blockno 对应的磁盘块
//    c. bread() 从 bget() 拿到 buf 后，检查 buf 的元数据
//       如果是缓存未命中
//       从磁盘读取磁盘块写到新的 buf->data 中
// 2. bwrite(buf) 接口将 buf 内容写到磁盘. Cache 上层拿到缓存的磁盘块后，读写缓存的磁盘块. 
//    如果是写，执行 bwrite(buf) 立即写回磁盘
//    读写后用 brelse(buf) 将缓存的引用计数 -1. 
// 3. brelse(buf) 将 buf 的引用计数 -1. 如果减后为 0, 就插入到头部


#include "types.h"
#include "param.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "riscv.h"
#include "defs.h"
#include "fs.h"
#include "buf.h"

// most-recently-used list
struct {
  // bcache 锁仅一个
  // 确保哪些 buf 正在缓存磁盘块内容
  // 以及有几个线程正在以及等待这些 buf 的相关信息的同步访问
  // buf 锁每个 buf 各自一个, 保护对各自的缓存内容的同步访问
  struct spinlock lock;

  // 大小固定大小的 buffer cache.
  // 采用双向链表. 大小固定不变, 所以链表的所有节点可以预先用数组分配
  // 初始化 binit() 利用头插构建好整个固定大小的链表后, 就不会再插入新节点
  // 
  // 为了快速获取缓存 buf 
  // 链表按照 brlese() 后, 引用计数为 0 的顺序排序
  // 尝试获取磁盘块 buf 时, 按照 "最近引用完" 到 "最远引用完" 遍历
  // 磁盘块未命中, 尝试选择替换的 buf 时, 遍历顺序反之.
  // 
  // buf.valid 表示该 buf 是否存储了磁盘块副本. 初始所有 buf.valid == 0
  // 如果一个 buf 暂未存储磁盘块的副本. 必定是这两种情况
  // 1. 处于 "冷启动" 期间. 读数据才会填充缓存, 所以缓存初始的填充需要等待 os 读磁盘块来逐步填充
  // 2. 缓存未命中, 要替换掉某个 buf 时, 会先设置 buf.valid==0 
  //    这就使缓存的老磁盘块数据 [被解释为无效]
  //    再返回被替换的 buf, 读新的磁盘块, 写进该 buf. 才重新设置 buf.valid==1
  //    从使 buf 中的磁盘块无效, 到返回该 buf, 重新写新磁盘块到这个 buf 的期间 buf.valid==0
  // 
  // buf.disk 表示该缓冲区内容有没有 [被磁盘持有] 
  // 在将写入磁盘块前, Cache上层先写磁盘块缓存, 再把磁盘块缓存用 bwrite(buf) 真正写入磁盘块
  // bwrite(buf) 调用驱动的 top-part, 在写入完磁盘块前, 设置 buf.disk 为 "未写入"
  // 然后线程在磁盘驱动 top-part 的函数进入 sleep, 在该 buf 等待
  // 实际写入完成后, 在任意其他线程 trap 进中断处理程序 kerneltrap 或 usertrap
  // kerneltrap 或 usertrap 检查中断类型和中断号, 调用磁盘驱动 bottom-part
  // 即磁盘驱动的中断处理程序, 在 xv6 是 virtio_disk_intr
  // 磁盘驱动的 top/bottom-part 在内核全局变量范围内, 记录当前磁盘处理的请求对应的磁盘请求号 id
  // virtio_disk_intr() 即驱动的 bottom-part 会找到当前的磁盘请求号 id
  // 磁盘请求号可以关联到磁盘中断对应磁盘请求的相关信息, 包括请求相关的 buf
  // 然后更新 buf.disk, wakeup(buf) 唤醒在 buf 等待的线程, 该线程再从 sleep 返回到磁盘驱动
  // 线程在醒来, 返回到磁盘驱动后检查 buf.disk, 检查有没有完成写入到磁盘
  struct buf buf[NBUF];

  // Linked list of all buffers, through prev/next.
  // Sorted by how recently the buffer was used.
  // head.next is most recent, head.prev is least.
  // 双向链表; 头节点固定, 不缓存磁盘块;
  struct buf head;
} bcache;


// 把数组 buf[NBUF] 中的 buf 缓冲区通过设置 next, prev 指针来建立成一个双向链表
void
binit(void)
{
  struct buf *b;
  initlock(&bcache.lock, "bcache");

  // Create linked list of buffers
  bcache.head.prev = &bcache.head;
  bcache.head.next = &bcache.head;
  for(b = bcache.buf; b < bcache.buf+NBUF; b++){
    b->next = bcache.head.next;
    b->prev = &bcache.head;
    initsleeplock(&b->lock, "buffer");
    bcache.head.next->prev = b;
    bcache.head.next = b;
  }
}

// Look through buffer cache for block on device dev.
// If not found, allocate a buffer.
// In either case, return locked buffer.
static struct buf*
bget(uint dev, uint blockno)
{
  struct buf *b;

  // 确保对 cache 中缓存的磁盘块的元数据如 valid, disk, dev, blockno 的访问是原子的
  // 仅仅这么说还不太根本, 访问本身无所谓是否原子. 
  // 
  // 程序对 "当前状态" 做出解释, 转变为新的状态, 之后再继续解释新的状态
  // 我们把状态被某过程解释, 直到该过程把它变为新状态的这个过程, 称为 "解释过程" 
  // 而对 [当前状态] 做出的 [解释], 其手段也只能是 [先获取状态] 
  // 并假设 [当前的状态] 就是 [之前获取的状态].  ---- RESU1
  // 而人在该假设下建立其程序, 人为编写的程序的目的
  // 是在 [之前获取的状态] 在解释过程中维持 [不变性] 的前提下的建立的
  // 只有该假设成立, 程序才符合初始目的
  //
  // 在这个例子中
  // 线程 A 获取缓冲区 blockno 的状态
  // 其后的解释, 就是把对应的缓存数据解释为该 blockno 下的副本. 
  // 而 blockno 可能因为该 buf 被替换, buf 变成其他 blockno 的缓存
  // 使线程双方都不符合, 人在编写程序时的目的了
  //
  // 所以不同线程对 bcache 的状态进行访问的区域需要保持同步
  // 必须一个结束后, 另一个才能继续
  // 让 "临界区" 保持 "原子性" 即可达成同步要求
  acquire(&bcache.lock);

  // Is the block already cached?
  for(b = bcache.head.next; b != &bcache.head; b = b->next){
    if(b->dev == dev && b->blockno == blockno){
      b->refcnt++;
      release(&bcache.lock);
      acquiresleep(&b->lock);
      return b;
    }
  }

  // Not cached.
  // Recycle the least recently used (LRU) unused buffer.
  // 如果缓存未命中，用 "最近最少访问" 的 buf 替换
  // 事实含义是: 从 buf->refcnt == 0 的 buf 中, 选择最远被 brelse(buf) 的 buf  
  // 对于上面这个替换选择过程，"最近最少访问" 这个名词中隐含的抽象如何对应、解释到基础事实?
  // 选择 buf 直接取决于两个信息: 调用 brelse() 的顺序; buf->refcnt
  // a. "最少" 如果是被替换的 buf 的 buf->refcnt，因为被替换的 buf 必须没有正在或等待被使用. 
  //     那么 "最近" 变成了 brelse() 最远调用的顺序
  // b. "最少" 解释为调用 brelse() 的顺序. 
  //     那么 "最近" 变成了 buf->refcnt 大小
  //     且靠后的 buf 不代表是最少(buf->refcnt 的大小)被访问的. 反而有可能最常被访问. 因为
  //     有可能一瞬间有很多线程要访问它, 而第一个拿到它的线程又没有释放它, 导致它还是被留在后面
  // c. "最近最少" 一起解释为调用 brelse() 的顺序. 可以, 却不是很贴合"最近", "最少"的核心抽象意
  //     因为越后的 buf 显然不是 [最近] 被 brelse() 的
  // d. "最近" 缓存列表中所有的都是最近访问的
  //    "最少" 忽略引用计数非 0 的 buf. "最少" 解释为调用 brelse() 的顺序
  // 所以不如用 "最远最少访问":
  // 从 [最远] 被 brelse() 的 buf 开始检查 [最少] 被引用(buf->refcnt==0)的buf
  // 
  // 可以看到没有在未命中时把缓存的磁盘块写回磁盘
  // Cache 的上层 log 层写块缓存，然后把块缓存写到 log 的数据区域
  // 并记录 logged data blocks 的每个 block 要写到的目标块号
  // 然后一次性将 logged data blocks 写进目标块
  for(b = bcache.head.prev; b != &bcache.head; b = b->prev){
    if(b->refcnt == 0) {
      b->dev = dev;
      b->blockno = blockno;
      b->valid = 0;
      b->refcnt = 1;
      // b->refcnt = 1 就不会有线程再替换该 buf. 直到再改为 0. 可以允许其他线程修改元数据了
      release(&bcache.lock);
      // The sleep-lock protects reads and writes of the block’s buffered content,
      acquiresleep(&b->lock);
      return b;
    }
  }
  panic("bget: no buffers");
}

// Return a locked buf with the contents of the indicated block.
// 返回在内存中缓存的磁盘块 buf 前，会获得该 buf 的锁. 读写 buf 的临界区上锁
// 而唯一获得 buf 的途径是通过 bread()
// 所以返回后可以确保对 buf 的读写的同步性
struct buf*
bread(uint dev, uint blockno)
{
  struct buf *b;

  b = bget(dev, blockno);
  if(!b->valid) {
    virtio_disk_rw(b, 0);
    b->valid = 1;
  }
  return b;
}

// Write b's contents to disk.  Must be locked.
void
bwrite(struct buf *b)
{
  if(!holdingsleep(&b->lock))
    panic("bwrite");
  virtio_disk_rw(b, 1);
}

// Release a locked buffer.
// Move to the head of the most-recently-used list.
void
brelse(struct buf *b)
{
  if(!holdingsleep(&b->lock))
    panic("brelse");

  releasesleep(&b->lock);

  acquire(&bcache.lock);
  b->refcnt--;
  if (b->refcnt == 0) {
    // no one is waiting for it.
    b->next->prev = b->prev;
    b->prev->next = b->next;
    b->next = bcache.head.next;
    b->prev = &bcache.head;
    bcache.head.next->prev = b;
    bcache.head.next = b;
  }
  
  release(&bcache.lock);
}

void
bpin(struct buf *b) {
  acquire(&bcache.lock);
  b->refcnt++;
  release(&bcache.lock);
}

void
bunpin(struct buf *b) {
  acquire(&bcache.lock);
  b->refcnt--;
  release(&bcache.lock);
}


