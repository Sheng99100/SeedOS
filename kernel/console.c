//
// Console input and output, to the uart.
// Reads are line at a time.
// Implements special input characters:
//   newline -- end of line
//   control-h -- backspace
//   control-u -- kill line
//   control-d -- end of file
//   control-p -- print process list
//

#include <stdarg.h>

#include "types.h"
#include "param.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "fs.h"
#include "file.h"
#include "memlayout.h"
#include "riscv.h"
#include "defs.h"
#include "proc.h"

#define BACKSPACE 0x100

// 约定 ascii 的前 31 位解释为控制字符
// 1 表示 Control-a, 2 表示 Control-b 以此类推 
// 且 Control-x 的 x 不区分大小写
// Control-a 的码值 = 'A'- 64 = 'A'-'@' = 1
#define C(x)  ((x)-'@')  // Control-x

//
// send one character to the uart.
// called by printf(), and to echo input characters,
// but not from write().
// 
void
consputc(int c)
{
  if(c == BACKSPACE){
    // if the user typed backspace, overwrite with a space.
    uartputc_sync('\b'); uartputc_sync(' '); uartputc_sync('\b');
  } else {
    uartputc_sync(c);
  }
}

// 控制台缓冲区
// 约定控制台缓冲区内容满一行后（从 uart 硬件接口读到 '\n') 
// consoleread() 才可开始读取到用户空间或内核空间, 否则阻塞
// 例如，如果是 shell 调用 read()
// shell 将控制台的文件描述符传入 read，read() 会 dispature 到 consoleread()
// 在这种情况下，如果缓冲区已经满一行，consoleread() 会读取行到用户空间
struct {
  struct spinlock lock;
  
  // input
  // [w, e) 区间属于 "键盘缓冲区" 即在控制台回显的字符串
  // [r, w) 才是可读的区间
  // 输入完一行(输入'\n')后，w 的值改为 e 的位置
  // 当输入 '\n' 时，consoleread() 才被唤醒，读取一行字符到用户空间或内核空间(consoleread()一开始不是系统调用dispather来的情况)
  // 为了方便判断是否可读等与位置相关的状态
  // r w e 不会随时被模，而是自然增长，只在使用的时候取模。
#define INPUT_BUF_SIZE 128
  char buf[INPUT_BUF_SIZE];
  uint r;  // Read index;  当前用户读取的位置
  uint w;  // Write index; 当前用户可读的最新字节的最后一个位置. 输入完一行(输入'\n')后，w 的值改为 e 的位置。
  uint e;  // Edit index;  当前最新字节的后一个位置，正编辑的位置，光标所在的位置
} cons;

//
// user write()s to the console go here.
// trap -> write(console file descirpter) -> consolewrite()
// 逐个获取用户在系统调用 write() 中，指定地址开始的指定个字节
// 逐个调用 uartput() 将一个字节放入输出缓冲区
int
consolewrite(int user_src, uint64 src, int n)
{
  int i;

  for(i = 0; i < n; i++){
    char c;
    if(either_copyin(&c, user_src, src+i, 1) == -1)
      break;
    uartputc(c);
  }

  return i;
}

//
// user read()s from the console go here.
// copy (up to) a whole input line to dst.
// user_dist indicates whether dst is a user
// or kernel address.
//
// 控制台被抽象成 "file"
// 控制台文件描述符的 read 函数指针指向这里
// 调用 SYS_read 时，根据用户空间传来的 read() 调用的文件描述符参数
// 得到具体的文件描述符结构体，其中包含用于区分不同 "文件" 类型（如 DEVICE）的数值
// 以及区分不同的具体设备的数值 i
// file.c 的 devsw[] 存储由每个具体设备的具体读写方法组成的结构体, 由 i 索引
// 而每个具体设备的具体读写方法，可以从各自驱动程序使用的输入或输出对应的数据结构读写数据
// 输入数据结构的写入可由驱动的中断处理程序 (驱动的 bottom-part) 处理
// 如 console.c 的输入数据结构 cons
// 所以 xv6 的驱动和内核之间是一体的
//
int
consoleread(int user_dst, uint64 dst, int n)
{
  uint target;
  int c;
  char cbuf;

  target = n;
  acquire(&cons.lock);
  while(n > 0){

    // 如果缓冲区没有可读行 
    // 则等待并释放缓冲区锁，直到其他进程执行时uart中断,UART处理程序写入缓冲区时再唤醒它.
    // 唤醒后，重新获得锁和检查条件
    // wait until interrupt handler has put some
    // input into cons.buffer.
    while(cons.r == cons.w){
      if(killed(myproc())) {
        release(&cons.lock);
        return -1;
      }
      // 
      sleep(&cons.r, &cons.lock);
    }

    c = cons.buf[cons.r++ % INPUT_BUF_SIZE];

    // 约定一个标志(EOF)，如果读的首个字节就是这个标志
    // 就直接退出读取循环并返回，读 0 个字符
    // 如果这个标志不是首个读到的字节
    // 就保留这个标志，留给下一次的trap，作为首个字节读取(第一类情况)
    // 然后退出读取循环并返回。此时前面复制过的字节就是复制过了，不会撤回
    // 具体操作是：把队列的头指针值-1（撤消这一次对这个标志的的读操作）
    // 这么做的意图是什么？
    if(c == C('D')){ // end-of-file
      if(n < target) {
        // 约定把 Control-D 解释为 end-of-file 标志
        // 读到之后把队列的头指针回退（撤消这一次对这个标志的读操作）
        // Save ^D for next time, to make sure
        // caller gets a 0-byte result.
        cons.r--;
      }
      break;
    }

    // 如果 user_dst 是 True，即 dst 被解释为用户地址
    // 那么该轮的字节会被载入到当前用户空间的 dst 地址
    // 所以从一开始的 dst 开始， xv6 就不关心这个地址以及之后的地址，是否是有规划的分配给复制过去的字节的
    // 也就是，如果 C 传了一个字符数组指针作为复制的起始地址 dst
    // 那么复制的 n 个字节的长度 n 是否超出了这个 C 字符数组的长度
    // 是否会发生覆盖、 page fault 都是没有关心的
    //
    // 唤醒后，可以通过获取当前进程的结构体，来获取用户页表
    // 之后就可以根据read()调用的虚拟地址参数，映射到物理地址
    // 并载入缓冲区数据到物理地址
    // 
    // 有一个问题：
    // 特定进程调用 read(..., console file describer, ...)
    // 该描述符的 read 调用 consoleread(), 如果缓冲区为空，就等待.
    // 于是，用户在键盘上针对这个特定进程键入行，并输入完成（按下'\n'）
    // 但是，也许存在多个在内核态等待读取缓冲区的进程被唤醒，并继续 consoleread()，开始在这里读取时
    // 那么针对特定进程键入的行，是否会被其他被唤醒的进程读走？
    // 或者，缓冲区不为空，直接错误的读走不属于自己进程的输入行 ？
    // copy the input byte to the user-space buffer.
    cbuf = c;
    if(either_copyout(user_dst, dst, &cbuf, 1) == -1)
      break;

    dst++;
    --n;

    if(c == '\n'){
      // a whole line has arrived, return to
      // the user-level read().
      break;
    }
  }
  release(&cons.lock);

  return target - n;  // 返回已读的字节数
}

//
// the console input interrupt handler.
// uartintr() calls this for input character.
// do erase/kill processing, append to cons.buf,
// wake up consoleread() if a whole line has arrived.
// uartintr() -> consoleintr()
// 把字节写到输入缓冲区
void
consoleintr(int c)
{
  acquire(&cons.lock);

  switch(c){
  case C('P'):  // Print process list.
    procdump();
    break;
  case C('U'):  // Kill line.
    while(cons.e != cons.w &&
          cons.buf[(cons.e-1) % INPUT_BUF_SIZE] != '\n'){
      cons.e--;
      consputc(BACKSPACE);
    }
    break;
  case C('H'): // Backspace
  case '\x7f': // Delete key
    if(cons.e != cons.w) {
      cons.e--;
      consputc(BACKSPACE);
    }
    break;
  default:
    if(c != 0 && cons.e-cons.r < INPUT_BUF_SIZE){
      c = (c == '\r') ? '\n' : c;

      // echo back to the user
      // 非特殊字符，放入读缓冲区.
      consputc(c);

      // store for consumption by consoleread().
      cons.buf[cons.e++ % INPUT_BUF_SIZE] = c;

      if(c == '\n' || c == C('D') || cons.e-cons.r == INPUT_BUF_SIZE){
        // 因为产生中断的环境（进程）和调用 consoleread() 的进程可以不是同一个
        // 所以 consoleintr 不能假设此时运行的是之前等待缓冲区行输入的特定进程
        // wake up consoleread() if a whole line (or end-of-file)
        // has arrived.
        cons.w = cons.e;
        wakeup(&cons.r);
      }
    }
    break;
  }
  
  release(&cons.lock);
}

void
consoleinit(void)
{
  initlock(&cons.lock, "cons");

  uartinit();

  // connect read and write system calls
  // to consoleread and consolewrite.
  devsw[CONSOLE].read = consoleread;
  devsw[CONSOLE].write = consolewrite;
}
