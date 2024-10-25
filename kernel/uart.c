//
// low-level driver routines for 16550a UART.
//

#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "spinlock.h"
#include "proc.h"
#include "defs.h"

// the UART control registers are memory-mapped
// at address UART0. this macro returns the
// address of one of the registers.
#define Reg(reg) ((volatile unsigned char *)(UART0 + (reg)))

// the UART control registers.
// some have different meanings for
// read vs write.
// see http://byterunner.com/16550.html
#define RHR 0 // receive holding register (for input bytes)
#define THR 0 // transmit holding register (for output bytes)
#define IER 1 // interrupt enable register
#define IER_RX_ENABLE (1 << 0)
#define IER_TX_ENABLE (1 << 1)
#define FCR 2 // FIFO control register
#define FCR_FIFO_ENABLE (1 << 0)
#define FCR_FIFO_CLEAR (3 << 1) // clear the content of the two FIFOs
#define ISR 2                   // interrupt status register
#define LCR 3                   // line control register
#define LCR_EIGHT_BITS (3 << 0)
#define LCR_BAUD_LATCH (1 << 7) // special mode to set baud rate
#define LSR 5                   // line status register
#define LSR_RX_READY (1 << 0)   // input is waiting to be read from RHR
#define LSR_TX_IDLE (1 << 5)    // THR can accept another character to send

// 以常量 UART0 0x10000000L 为偏移，计算uart寄存器 reg 的内存映射地址 Reg + reg
#define ReadReg(reg) (*(Reg(reg)))
#define WriteReg(reg, v) (*(Reg(reg)) = (v))

// the transmit output buffer.
struct spinlock uart_tx_lock;
#define UART_TX_BUF_SIZE 32
char uart_tx_buf[UART_TX_BUF_SIZE];
uint64 uart_tx_w; // write next to uart_tx_buf[uart_tx_w % UART_TX_BUF_SIZE]
uint64 uart_tx_r; // read next from uart_tx_buf[uart_tx_r % UART_TX_BUF_SIZE]

extern volatile int panicked; // from printf.c

void uartstart();

void uartinit(void)
{
  // disable interrupts.
  WriteReg(IER, 0x00);

  // special mode to set baud rate.
  WriteReg(LCR, LCR_BAUD_LATCH);

  // LSB for baud rate of 38.4K.
  WriteReg(0, 0x03);

  // MSB for baud rate of 38.4K.
  WriteReg(1, 0x00);

  // leave set-baud mode,
  // and set word length to 8 bits, no parity.
  WriteReg(LCR, LCR_EIGHT_BITS);

  // reset and enable FIFOs.
  WriteReg(FCR, FCR_FIFO_ENABLE | FCR_FIFO_CLEAR);

  // enable transmit and receive interrupts.
  WriteReg(IER, IER_TX_ENABLE | IER_RX_ENABLE);

  initlock(&uart_tx_lock, "uart");
}

// add a character to the output buffer and tell the
// UART to start sending if it isn't already.
// blocks if the output buffer is full.
// because it may block, it can't be called from interrupts;
// it's only suitable for use by write().
// 将一个字节放入输出缓冲区，如果缓冲区满，则进程等待
// 最后调用 uartstart() 消费缓冲区的一个字节
void uartputc(int c)
{
  acquire(&uart_tx_lock);

  if (panicked)
  {
    for (;;)
      ;
  }

  while (uart_tx_w == uart_tx_r + UART_TX_BUF_SIZE)
  {
    // buffer is full.
    // wait for uartstart() to open up space in the buffer.
    sleep(&uart_tx_r, &uart_tx_lock);
  }

  uart_tx_buf[uart_tx_w % UART_TX_BUF_SIZE] = c;
  uart_tx_w += 1;
  uartstart();
  release(&uart_tx_lock);
}

// alternate version of uartputc() that doesn't
// use interrupts, for use by kernel printf() and
// to echo characters. it spins waiting for the uart's
// output register to be empty.
// "同步" "异步" 是指通讯双方是否采用一致的步调
// 例如 CPU 和 THR 之间
// 如果是同步
// CPU 就要把字节给 THR，THR 收到后就要立刻消费,通信双方步调一致，不能自做自的
// 如果是异步
// 就说明 CPU 和 THR 之间能够在各自做各自的事的前提下，还能把通讯完成
// 例如通过一个缓冲区，CPU 把字节放到缓冲区，然后干自己的事，不用理会发送字节时 THR 是否空闲
// 而 THR 也可以通过中断，从缓冲区接受字节。 
// 使用缓冲区通讯的 CPU 和 THR 之间，通讯的步调就不一致，可以各干各的
// 
// 同步(针对整个CPU和THR之间)版本的 uartputc(), 不会把输出字节先放在缓冲区
// 而是循环等待输出寄存器 THR 为空后, 立即输出字节到输出寄存器 THR. 对于整个 CPU 和 THR 间是同步的
//
// “异步” 可能在以现象中
// 对特定进程，先放下该进程的某个任务，让其他任务来运行
// 也可以针对整个CPU（或者OS），先放下一个进程，让其他进程运行
// 阻塞再在原位置唤醒进程的情况，对于整个 CPU 属于异步，对于这个进程来说还是同步
// 同步异步的概念在具体语境下却模糊的原因，其实是对于这个 抽象 所嵌入的上下文的模糊
// 到底是针对哪两个通信对象
void uartputc_sync(int c)
{
  push_off();

  if (panicked)
  {
    for (;;)
      ;
  }

  // wait for Transmit Holding Empty to be set in LSR.
  // 等待输出寄存器 THR 空闲
   while ((ReadReg(LSR) & LSR_TX_IDLE) == 0)
    ;
  // 立即写出一个字节，而不经过缓冲区
  WriteReg(THR, c);

  pop_off();
}

// if the UART is idle, and a character is waiting
// in the transmit buffer, send it.
// caller must hold uart_tx_lock.
// called from both the top- and bottom-half.
// 逐个把输出缓冲区的字节，写到 uart 输出端口。
// 直到缓冲区空 或 硬件未准备好接受新输出
// uartstart() 在驱动的 top- 和 bottom-half 调用
// 1. consolewrite() -> uartputc() -> uartstart()
// 2. uartintr() -> uartstart()
void uartstart() {
  while (1)
  {
     // transmit buffer is empty.
      // 因为 uartstart() 在两种情况下使用 (驱动的top-和bottom-half)
      // 1. consolewrite() -> uartputc() -> uartstart()
      // 2. uartintr() -> uartstart()
      //       (1). 因为收到输入的中断
      //       (2). 告知可接受新输出的中断
      // 如果不是第一种put到输出缓冲区后调用的，输出缓冲区域从loop一开始就可能空
    if (uart_tx_w == uart_tx_r)
    {
      ReadReg(ISR);
      return;
    }

    if ((ReadReg(LSR) & LSR_TX_IDLE) == 0)
    {
      // 如果非 2. (2) 的情况，uart 输出寄存器就可能从loop一开始就不可接受新字节
      // 循环写出的过程中也可能遇到硬件未准备好的情况
      // 那么就不浪费时间等待（反正硬件准备好输出后又会发生一次中断）
      // 这种情况不做处理，让下一次中断 usrtintr() -> uartstart() 去消费.
      // the UART transmit holding register is full,
      // so we cannot give it another byte.
      // it will interrupt when it's ready for a new byte.
      return;
    }

    // 否则写出输出缓冲区的一个字节，到 uart 输出端口
    int c = uart_tx_buf[uart_tx_r % UART_TX_BUF_SIZE];
    uart_tx_r += 1;

    // maybe uartputc() is waiting for space in the buffer.
    wakeup(&uart_tx_r);

    WriteReg(THR, c);
  }
}

// read one input character from the UART.
// return -1 if none is waiting.
int uartgetc(void)
{
  if (ReadReg(LSR) & 0x01)
  {
    // input data is ready.
    return ReadReg(RHR);
  }
  else
  {
    return -1;
  }
}

// handle a uart interrupt, raised because input has
// arrived, or the uart is ready for more output, or
// both. called from devintr().
// 输入字符和有新字符可输出
// 两种具体原因的中断都用 uartintr 处理
// trap(trampoline.S 或 kernelvec.S) -> usertrap 或 kerneltrap ->
// devintr() -> uartintr()
void uartintr(void)
{
  // read and process incoming characters.
  // 从 uart 硬件逐个读取字节，直到没有剩余字节可读（约定读到 -1）
  // uart 用 RHR 寄存器作为读取的接口
  // uart 中断可能发生在键盘输入
  // 或者uart"完成"输出, 准备好接受新的输出字节的时候.
  // 两类中断都尝试调用 consoleintr()，把输入字节载入到控制台输入缓冲区
  // 另外，中断原因是后者时，输入寄存器可能一开始就没有数据
  while (1)
  {
    int c = uartgetc();
    if (c == -1) // 读完所有可读字节
      break;
    consoleintr(c);
  }

  // send buffered characters.
  // 两种情况下的中断，都尝试将输出缓冲区的下一个字节写到 uart 的输出寄存器
  acquire(&uart_tx_lock);
  uartstart();
  release(&uart_tx_lock);
}
