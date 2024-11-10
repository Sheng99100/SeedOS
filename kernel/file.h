struct file {
  enum { FD_NONE, FD_PIPE, FD_INODE, FD_DEVICE } type;
  int ref; // reference count
  char readable;
  char writable;
  struct pipe *pipe; // FD_PIPE
  struct inode *ip;  // FD_INODE and FD_DEVICE
  uint off;          // FD_INODE
  short major;       // FD_DEVICE
};

#define major(dev)  ((dev) >> 16 & 0xFFFF)
#define minor(dev)  ((dev) & 0xFFFF)
#define	mkdev(m,n)  ((uint)((m)<<16| (n)))

// in-memory copy of an inode
struct inode {
  // inode 缓存的锁和块缓存的锁策略类似
  // 块缓存的块外部的事情，如缓存获取、绑定，用 bcache.lock, 块自己的的事情用 buf.lock
  // inode 外部的事情，如 inode 表项获取、绑定，用，itable.lock, inode 自己的事情用 inode.lock
  
  //-----------这三个 fields 都和inode缓存的分配等 inode 外部事务相关，用 itable->lock 保护，-----------
  uint dev;           // Device number
  uint inum;          // Inode number
  int ref;            // Reference count
  //------------------------------------------------------

 
  struct sleeplock lock; // protects everything below here
 // 下面的 fields 除了 valid, 都是磁盘内 inode 格式
 // 仅和 inode 本身的事情相关，用 inode 专用的锁保护
  int valid;          // inode has been read from disk?
  short type;         // copy of disk inode
  short major;
  short minor;
  short nlink;
  uint size;
  uint addrs[NDIRECT+1];
};

// map major device number to device functions.
struct devsw {
  int (*read)(int, uint64, int);
  int (*write)(int, uint64, int);
};

extern struct devsw devsw[];

#define CONSOLE 1
