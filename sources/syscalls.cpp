//
// Created by marmelade on 25/01/23.
//
#include <cerrno>

#define unused_param(param) (void) param

__attribute__((weak)) int _kill(int pid, int sig) {

    unused_param(pid);
    unused_param(sig);

    errno = EINVAL;

    return -1;
}

[[noreturn]] __attribute__((weak)) void _exit(int status) {

    _kill(status, -1);

    while(true) {

    }

}

__attribute__((weak)) int _getpid() {

    return 1;
}

void *_sbrk(std::ptrdiff_t size) {

    unused_param(size);

    return nullptr;
}
