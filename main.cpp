#include "iMalloc.h"
#include <cstdio>

int main()
{
  void* ptr = iMalloc(0);
  fprintf(stderr, "ptr: %p\n", ptr);
  return 0;
}

