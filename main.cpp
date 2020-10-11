#include "imalloc.h"
#include <cstdio>

int main()
{
  void* ptr = imalloc(0);
  fprintf(stderr, "ptr: %p\n", ptr);
  return 0;
}

