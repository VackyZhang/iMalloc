#ifndef DLMALLOC_SRC_IMALLOC_H_
#define DLMALLOC_SRC_IMALLOC_H_

#include <cstddef>

void* iMalloc(size_t bytes);

void iFree(void* mem);

#endif
