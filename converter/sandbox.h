// Converter sandbox (direction review 6-3, plan A): once the DWG is in
// memory and the output file is open, the converter drops every way out of
// the process with a seccomp filter. From then on it can only compute, read
// and write the files it already has open (stdin/stdout/stderr, the output),
// manage memory and exit. Opening files, sockets, starting programs, new
// threads and executable memory all fail with EPERM. A bug in the DWG
// decoder can then crash the converter, but cannot send a drawing anywhere
// or touch the app's other files.
//
// Linux (Android) only; elsewhere sandbox_enter() reports SANDBOX_NONE.
// Blocked calls fail softly (EPERM), so a libc that tries something harmless
// (naming memory, checking for a terminal) keeps working.
#ifndef UNIMOG_SANDBOX_H
#define UNIMOG_SANDBOX_H

enum { SANDBOX_NONE, SANDBOX_ON, SANDBOX_FAILED };
static const char *const SANDBOX_NAME[] = {"none", "on", "failed"};

#if defined(__linux__) && (defined(__aarch64__) || defined(__x86_64__))
#include <errno.h>
#include <linux/audit.h>
#include <linux/filter.h>
#include <linux/seccomp.h>
#include <stddef.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/syscall.h>
#include <unistd.h>

#if defined(__aarch64__)
#define SANDBOX_ARCH AUDIT_ARCH_AARCH64
#else
#define SANDBOX_ARCH AUDIT_ARCH_X86_64
#endif

#define SB_LD(f) BPF_STMT(BPF_LD | BPF_W | BPF_ABS, offsetof(struct seccomp_data, f))
#define SB_ALLOW(nr) BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, (nr), 0, 1), \
                     BPF_STMT(BPF_RET | BPF_K, SECCOMP_RET_ALLOW)
// mmap / mprotect: allowed unless the protection (argument 3 / 2) asks for
// PROT_EXEC. Low 32 bits of the argument are enough for the flag.
#define SB_NO_EXEC(nr, arg)                                                    \
  BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, (nr), 0, 4),                             \
      BPF_STMT(BPF_LD | BPF_W | BPF_ABS, offsetof(struct seccomp_data, args[arg])), \
      BPF_JUMP(BPF_JMP | BPF_JSET | BPF_K, PROT_EXEC, 1, 0),                   \
      BPF_STMT(BPF_RET | BPF_K, SECCOMP_RET_ALLOW),                            \
      BPF_STMT(BPF_RET | BPF_K, SECCOMP_RET_ERRNO | (EPERM & SECCOMP_RET_DATA))

static int sandbox_enter(void) {
  struct sock_filter f[] = {
      // Only native calls: a foreign architecture means other numbers.
      SB_LD(arch),
      BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, SANDBOX_ARCH, 1, 0),
      BPF_STMT(BPF_RET | BPF_K, SECCOMP_RET_KILL_PROCESS),
      SB_LD(nr),
      // Files it already has: stdin/stdout/stderr, the output.
      SB_ALLOW(__NR_read), SB_ALLOW(__NR_readv), SB_ALLOW(__NR_pread64),
      SB_ALLOW(__NR_write), SB_ALLOW(__NR_writev), SB_ALLOW(__NR_pwrite64),
      SB_ALLOW(__NR_lseek), SB_ALLOW(__NR_fstat), SB_ALLOW(__NR_close),
      SB_ALLOW(__NR_ftruncate), SB_ALLOW(__NR_fsync), SB_ALLOW(__NR_fdatasync),
      // Memory (no executable pages).
      SB_ALLOW(__NR_munmap), SB_ALLOW(__NR_mremap), SB_ALLOW(__NR_madvise),
      SB_ALLOW(__NR_brk), SB_ALLOW(__NR_prctl),  // allocator names its regions
      // Time, identity, signals for abort(), one-thread locking.
      SB_ALLOW(__NR_clock_gettime), SB_ALLOW(__NR_gettimeofday),
      SB_ALLOW(__NR_nanosleep), SB_ALLOW(__NR_clock_nanosleep),
      SB_ALLOW(__NR_getpid), SB_ALLOW(__NR_gettid), SB_ALLOW(__NR_getrandom),
      SB_ALLOW(__NR_futex), SB_ALLOW(__NR_sched_yield),
      SB_ALLOW(__NR_rt_sigaction), SB_ALLOW(__NR_rt_sigprocmask),
      SB_ALLOW(__NR_rt_sigreturn), SB_ALLOW(__NR_sigaltstack), SB_ALLOW(__NR_tgkill),
      SB_ALLOW(__NR_exit), SB_ALLOW(__NR_exit_group),
      SB_NO_EXEC(__NR_mmap, 2),
      SB_NO_EXEC(__NR_mprotect, 2),
      // Everything else: openat, socket, connect, execve, clone, ...
      BPF_STMT(BPF_RET | BPF_K, SECCOMP_RET_ERRNO | (EPERM & SECCOMP_RET_DATA)),
  };
  struct sock_fprog prog = {(unsigned short)(sizeof f / sizeof *f), f};
  if (prctl(PR_SET_NO_NEW_PRIVS, 1, 0, 0, 0)) return SANDBOX_FAILED;
  if (syscall(__NR_seccomp, SECCOMP_SET_MODE_FILTER, 0, &prog) &&
      prctl(PR_SET_SECCOMP, SECCOMP_MODE_FILTER, &prog))
    return SANDBOX_FAILED;
  return SANDBOX_ON;
}

// For checking a device: after sandbox_enter(), what still works? Prints
// one line per probe; every one must say "blocked".
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
static int sandbox_selftest(void) {
  int bad = 0, r;
#define PROBE(name, call)                                                      \
  do {                                                                         \
    errno = 0;                                                                 \
    r = (call);                                                                \
    int blocked = r < 0 && errno == EPERM;                                     \
    bad += !blocked;                                                           \
    printf("PROBE %s %s\n", name, blocked ? "blocked" : "OPEN");               \
  } while (0)
  PROBE("open", open("/proc/self/status", O_RDONLY));
  PROBE("socket_inet", socket(AF_INET, SOCK_STREAM, 0));
  PROBE("socket_unix", socket(AF_UNIX, SOCK_STREAM, 0));
  char *const av[] = {"/system/bin/sh", NULL};
  PROBE("exec", execve(av[0], av, NULL));
  PROBE("mmap_exec", mmap(NULL, 4096, PROT_READ | PROT_EXEC, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0) == MAP_FAILED ? -1 : 0);
  PROBE("unlink", unlink("/data/local/tmp/sandbox-probe"));
  PROBE("fork", (int)syscall(__NR_clone, 17 /* SIGCHLD */, 0, 0, 0, 0));
#undef PROBE
  // Still alive for real work: memory and output.
  char *m = malloc(64 << 20);
  if (m) memset(m, 1, 64 << 20);
  printf("PROBE malloc %s\n", m ? "ok" : "FAILED");
  bad += !m;
  free(m);
  return bad;
}
#else
static int sandbox_enter(void) { return SANDBOX_NONE; }
static int sandbox_selftest(void) { return 0; }
#endif

#endif
