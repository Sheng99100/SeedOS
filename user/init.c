// init: The initial user-level program
// 首个用户进程
// 被加载调度的过程是
// 1. kernel 的 main()
// 2. main()->userinit() 初始化一个进程结构体，包括把 initcode.S 的内容复制到给进程分配的空间中. 
// 3. main()->scheduler() 调度这个初始的进程 p ，swtch() 到 p 的上下文
//    刚创建的进程 p 是“假装”从 trap 返回的
//    所以 p->contect.ra 被设置成了 forkret 的地址，forkret 又会调用 usertrapret()
//    于是 swtch() 后，p 运行的是 initcode.S 的代码
// 4. initcode.S 调用系统调用 exec，把自己替换称 init.c
//    initcode.S 是临时的用户进程
//    init.c 就是正式的首个用户进程了，它是所有其他用户进程的父进程, 循环的执行
// 5. init.c 又 fork() 出一个子进程, 这个子进程用来替换为 shell.
// 6. init.c fork() 出的子进程调用 exec() 把自己替换成 shell
// 7. init.c 作为 shell 的父进程，循环检查运行 shell 的子进程有没有被退出（被释放资源结束执行）
//    如果退出了，就重新创建一个 shell 进程
//    重新从 5. 开始：fork 出一个子进程，子进程用 exec 替换成 shell  
//
// 另外重要的是，kernel 的 main() 最终会转到 scheduler() 循环执行
// main() 和 scheduler() 的用的栈，是每个 CPU 初始在 entry.S 中设置的 sp
// 而不是某个用户进程的用户栈、内核栈，运行它们时还不存在任何用户进程
// 每个 CPU 执行 entry.S 时设置的 sp，即每个 CPU 初始的栈、main() 和 scheduler() 的栈
// 是 kernel 作为 C 程序被编译器分配的全局变量，用数组表示的连续空间的各个部分
// (kernel 会告诉编译器，自己会手动分配并设置 sp，不需要编译器分配设置)
// 所以 scheduler() 执行的上下文是 cpu->context
// 每次时钟中断的 trap，都会 swtch() 到 cpu->context, 设置 ra 到 scheduler()
// swtch() 的 ret 指令会跳转到 ra

#include "kernel/types.h"
#include "kernel/stat.h"
#include "kernel/spinlock.h"
#include "kernel/sleeplock.h"
#include "kernel/fs.h"
#include "kernel/file.h"
#include "user/user.h"
#include "kernel/fcntl.h"

char *argv[] = { "sh", 0 };

int
main(void)
{
  int pid, wpid;

  // 把 console 的文件描述符关联到 init 进程
  // fork 出的 sh 程序被解释为父进程 init 的子进程，共享父进程的文件描述符
  if(open("console", O_RDWR) < 0){
    mknod("console", CONSOLE, 0);
    open("console", O_RDWR);
  }
  dup(0);  // stdout
  dup(0);  // stderr

  for(;;){
    printf("init: starting sh\n");
    pid = fork();
    if(pid < 0){
      printf("init: fork failed\n");
      exit(1);
    }
    
    // kernel 设置 fork() 的返回值
    // 子进程返回 0
    // 原进程（父进程）返回子进程的 pid 
    if(pid == 0){ 
      // fork 出的子进程运行程序 sh
      exec("sh", argv);

      // 理想情况下 exec 运行后，进程内容被 sh 替换，sepc 被改为 sh 程序的入口地址
      // 然后经过 trampoline.S userret 的 sret 返回到 sepc
      // trap 前 trap 后完全就是两个程序了
      // 如果 exec 替换失败，那么会回到 trap 前的原进程，也就是这里
      printf("init: exec sh failed\n");
      exit(1);
    }

    // 父进程（init）循环执行这里的指令
    //（子进程不执行这里，因为它的内存区域已经完全被exec的新内容替换，包括下面的代码）
    // 父进程用于循环检查运行 shell 的子进程有没有被退出（被释放资源结束执行）
    // 如果退出了，就重新创建一个 shell 进程：
    // 重新 fork() 出一个子进程，子进程中调用用 exec, 把自己替换成 shell  
    for(;;){
      // this call to wait() returns if the shell exits,
      // or if a parentless process exits.
      wpid = wait((int *) 0);
      if(wpid == pid){
        // the shell exited; restart it.
        break;
      } else if(wpid < 0){
        printf("init: wait returned an error\n");
        exit(1);
      } else {
        // it was a parentless process; do nothing.
      }
    }
  }
}
