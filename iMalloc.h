#ifndef DLMALLOC_SRC_IMALLOC_H_
#define DLMALLOC_SRC_IMALLOC_H_

#include <cstddef>

void* imalloc(size_t bytes);

void ifree(void* mem);

#endif//DLMALLOC_SRC_IMALLOC_H_
