#include <stddef.h>
#include <stdio.h>

// should probably move these to seperate files and also have ASM implementations
//
// lifted from GCC which are labled public domain,
// should probably investigae further just to be sure
extern "C" {
    #include "umm_malloc.h"

    #define __BKPT() asm("bkpt")

    void * malloc(size_t size) { return umm_malloc(size); }
    void * calloc(size_t num, size_t size) { return umm_calloc(num, size); }
    void * realloc(void *ptr, size_t size) { return umm_realloc(ptr, size); }
    void free(void *ptr) { umm_free(ptr); }

    void *memcpy (void *dest, const void *src, size_t len) {
        char *d = (char *) dest;
        const char *s = (const char *) src;
        while (len--) {
            *d++ = *s++;
        }
        return dest;
    }

    void * memset (void *dest, int val, size_t len)
    {
    unsigned char *ptr = (unsigned char *) dest;
    while (len-- > 0)
        *ptr++ = val;
    return dest;
    }

    void * __attribute__ ((__noreturn__)) memmove ( void * destination, const void * source, size_t num ) {
        __BKPT();
        for(;;);
    }

    int __attribute__ ((__noreturn__)) strcmp ( const char * str1, const char * str2 ) {
        __BKPT();
        for(;;);
    }

    int __attribute__ ((__noreturn__)) fputs(const char *str, FILE *stream) {
        __BKPT();
        for(;;);
    }

    void __attribute__ ((__noreturn__)) __assert_func (const char *expr, const char *file, unsigned int line, const char *function) {
        asm("bkpt");
        for(;;);
    }

    int memcmp (const void * str1, const void * str2, size_t count) {
        const unsigned char *s1 = (const unsigned char*)str1;
        const unsigned char *s2 = (const unsigned char*)str2;

        while (count-- > 0)
            {
            if (*s1++ != *s2++)
            return s1[-1] < s2[-1] ? -1 : 1;
            }
        return 0;
    }

    char * strchr (const char *s, int c) {
    do {
        if (*s == c)
        {
        return (char*)s;
        }
    } while (*s++);
    return (0);
    }

    uintptr_t __stack_chk_guard = 0xa5f3cc8d;
    __attribute__((weak,noreturn)) void __stack_chk_fail(void)
    {
        __BKPT();
        for(;;);
    }
}

void* operator new(size_t sz) {
  void* m = malloc(sz);
  return m;
}

void* operator new[] (size_t sz) {
  __BKPT();
  for(;;);
}

void operator delete(void* m, size_t size) {
  free(m);
}

void operator delete[] (void* ptr) {
    __BKPT();
    for(;;);
}