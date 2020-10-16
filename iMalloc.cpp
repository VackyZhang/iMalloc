#include "iMalloc.h"
#include <cassert>
#include <cstdio>
#include <cerrno>
#include <unistd.h>
#include <sys/mman.h>

struct malloc_chunk
{
  size_t prev_size;
  size_t size;
  struct malloc_chunk* fd;
  struct malloc_chunk* bk;
};

typedef struct malloc_chunk* mchunkptr;
typedef struct malloc_chunk* mfastbinptr;
typedef struct malloc_chunk* mbinptr;

#define SIZE_SZ   (sizeof(size_t))
#define CHUNK_SIZE_T unsigned long

#ifndef MORECORE
#define MORECORE sbrk
#endif

#define MMAP(addr, size, prot, flags) (mmap((addr), (size), (prot), (flags)|MAP_ANONYMOUS, -1, 0))

#define DEFAULT_TOP_PAD   (0)

#ifndef HAVE_MMAP
#define HAVE_MMAP 1
#endif

#ifndef MORECORE_FAILURE
#define MORECORE_FAILURE (-1)
#endif

#if HAVE_MMAP
#define DEFAULT_MMAP_MAX  (65536)
#else
#define DEFAULT_MMAP_MAX (0)
#endif

#ifndef DEFAULT_MMAP_THRESHOLD
#define DEFAULT_MMAP_THRESHOLD (256 * 1024)
#endif

#ifndef DEFAULT_TRIM_THRESHOLD
#define DEFAULT_TRIM_THRESHOLD (256 * 1024)
#endif

#ifndef DEFAULT_MXFAST
#define DEFAULT_MXFAST     64
#endif

#ifndef MORECORE_CONTIGUOUS
#define MORECORE_CONTIGUOUS   1
#endif

#ifndef MMAP_AS_MORECORE_SIZE
#define MMAP_AS_MORECORE_SIZE (1024 * 1024)
#endif

#ifndef MALLOC_FAILURE_ACTION
#define MALLOC_FAILURE_ACTION   errno = ENOMEM;
#endif

#define MORECORE_CONTIGUOUS_BIT  (1U)

#define ANYCHUNKS_BIT   (1U)
#define have_anychunks(M)   (((M)->max_fast & ANYCHUNKS_BIT))
#define set_anychunks(M)    ((M)->max_fast |= ANYCHUNKS_BIT)
#define clear_anychunks(M)  ((M)->max_fast &= ~ANYCHUNKS_BIT)

#define FASTCHUNKS_BIT  (2U)
#define have_fastchunks(M)  (((M)->max_fast & FASTCHUNKS_BIT))
#define set_fastchunks(M)   ((M)->max_fast |= (FASTCHUNKS_BIT | ANYCHUNKS_BIT))
#define clear_fastchunks(M) ((M)->max_fast &= ~(FASTCHUNKS_BIT))

#define set_contiguous(M)     ((M)->morecore_properties |= MORECORE_CONTIGUOUS_BIT)
#define set_noncontiguous(M)  ((M)->morecore_properties &= ~MORECORE_CONTIGUOUS_BIT)

#define SMALLBIN_WIDTH      8
#define MIN_LARGE_SIZE      256

#define FIRST_SORTED_BIN_SIZE MIN_LARGE_SIZE

#define in_smallbin_range(sz)   ((CHUNK_SIZE_T)(sz) < (CHUNK_SIZE_T)MIN_LARGE_SIZE)

#define smallbin_index(sz)  (((unsigned)(sz)) >> 3)

#define last(b)   ((b)->bk)

#define MIN_CHUNK_SIZE (sizeof(struct malloc_chunk))

#define NBINS   96
#define NSMALLBINS  32

// Conservatively use 32 bits per map word, even if on 64bit system.
#define BINMAPSHIFT 5
#define BITSPERMAP (1U << BINMAPSHIFT)
#define BINMAPSIZE (NBINS / BITSPERMAP)

#define idx2block(i)  ((i) >> BINMAPSHIFT)
#define idx2bit(i)    ((1U << ((i) & ((1U << BINMAPSHIFT) - 1))))

#define mark_bin(m, i)  ((m)->binmap[idx2block(i)] |= idx2bit(i))
#define get_binmap(m, i)  ((m)->binmap[idx2block(i)] & idx2bit(i))

#define MALLOC_ALIGNMENT  (2 * SIZE_SZ)
#define MALLOC_ALIGN_MASK (MALLOC_ALIGNMENT - 1)
#define MINSIZE (CHUNK_SIZE_T)(((MIN_CHUNK_SIZE+MALLOC_ALIGN_MASK) & ~MALLOC_ALIGN_MASK))

#define MAX_FAST_SIZE   80
// offset 2 to use otherwise unindexable first 2 bins.
#define fastbin_index(sz) ((((unsigned int)(sz)) >> 3) - 2)

#define chunk2mem(p) ((void*)((char*)(p) + 2 * SIZE_SZ))
#define mem2chunk(mem) ((mchunkptr)((char*)(mem) - 2 * SIZE_SZ))

// size field is or'ed with PREV_INUSE when previous adjacent chunk in use.
#define PREV_INUSE 0x1
// extract inuse bit of previous chunk
#define prev_inuse(p) ((p)->size & PREV_INUSE)

// size field is or'ed with IS_MMAPPED if the chunk was obtained with mmap().
#define IS_MMAPPED 0x2
// check for mmap()'ed chunk
#define chunk_is_mmapped(p) ((p)->size & IS_MMAPPED)

#define SIZE_BITS (PREV_INUSE|IS_MMAPPED)
// Get size, ignoring use bits.
#define chunksize(p) ((p)->size & ~(SIZE_BITS))

// pad request bytes into a usable size -- internal version.
// req + SIZE_SZ + 2 * SIZE_SZ，并按8byte进行取整。
// 最小取MINSIZE: (sizeof(malloc_chunk) + MALLOC_ALIGN_MASK) & ~MALLOC_ALIGN_MASK
// [MINSIZE, (req + size_sz)按2*size_sz进行padding)
#define request2size(req)                               \
  (((req) + SIZE_SZ + MALLOC_ALIGN_MASK < MINSIZE) ?    \
  MINSIZE :                                             \
  ((req) + SIZE_SZ + MALLOC_ALIGN_MASK) & ~MALLOC_ALIGN_MASK)
#define NFASTBINS (fastbin_index(request2size(MAX_FAST_SIZE))+1)

#define malloc_getpagesize  sysconf(_SC_PAGE_SIZE)

// Set value of max_fast.
// Use impossible small value if 0.
#define set_max_fast(M, s) \
  (M)->max_fast = (((s) == 0) ? SMALLBIN_WIDTH : request2size(s)) | ((M)->max_fast & (FASTCHUNKS_BIT|ANYCHUNKS_BIT))

// addressing -- note that bin_at(0) does not exist.
#define bin_at(m, i)  ((mbinptr)((char*)&((m)->bins[(i)<<1]) - (SIZE_SZ<<1)))

#define next_bin(b)   ((mbinptr)((char*)(b) + (sizeof(mchunkptr) << 1)))

// The otherwise unindexable 1-bin is used to hold unsorted chunks.
#define unsorted_chunks(M)  (bin_at(M, 1))

// Conveniently, the unsorted bin can be used as dummy top on first call.
#define initial_top(M)  (unsorted_chunks(M))

#define chunk_at_offset(p, s) ((mchunkptr)(((char*)(p)) + (s)))

// check/set/clear inuse bits in known places.
#define inuse_bit_at_offset(p, s)   (((mchunkptr)(((char*)(p)) + (s)))->size & PREV_INUSE)

#define set_inuse_bit_at_offset(p, s)   (((mchunkptr)(((char*)(p)) + (s)))->size |= PREV_INUSE)

// Set size/use field.
#define set_head(p, s)  ((p)->size = (s))

#define set_foot(p, s)  (((mchunkptr)((char*)(p) + (s)))->prev_size = (s))

#define contiguous(M) (((M)->morecore_properties & MORECORE_CONTIGUOUS_BIT))
#define set_contiguous(M) ((M)->morecore_properties |= MORECORE_CONTIGUOUS_BIT)

// Take a chunk off a bin list.
#define unlink(P, BK, FD) \
{                         \
  FD = P->fd;             \
  BK = P->bk;             \
  FD->bk = BK;            \
  BK->fd = FD;            \
}

struct malloc_state
{
  // The maximum chunk size to be eligible for fastbin.
  size_t max_fast;    // low 2 bits used as flags.

  // Fastbins
  mfastbinptr fastbins[NFASTBINS];

  // Base of the topmost chunk -- not otherwise kept in a bin.
  mchunkptr top;

  // The remainder from the most recent split of a small request
  mchunkptr last_remainder;

  // Normal bins packed as described above.
  mchunkptr bins[NBINS * 2];

  // Bitmap of bins. Trailing zero map handles cases of largest binned size.
  unsigned int binmap[BINMAPSIZE + 1];

  // Tunable parameters.
  CHUNK_SIZE_T trim_threshold;
  size_t top_pad;
  size_t mmap_threshold;

  // Memory map support.
  int n_mmaps;
  int n_mmaps_max;
  int max_n_mmaps;

  // Cache malloc_getpagesize
  unsigned int pagesize;

  // Track properties of MORECORE.
  unsigned int morecore_properties;

  // Statistics.
  size_t mmapped_mem;
  size_t sbrked_mem;
  size_t max_sbrked_mem;
  size_t max_mmapped_mem;
  size_t max_total_mem;
};

typedef struct malloc_state* mstate;

static struct malloc_state av_; // never directly referenced

#define get_malloc_state() (&(av_))

// Initialize a malloc_state struct.
// 初始化全局的malloc_state对象
static void malloc_init_state(mstate av)
{
  int i;
  mbinptr bin;

  // Establish circular links for normal bins.
  fprintf(stderr, "[%s] NBINS: %d\n", __FUNCTION__, NBINS);
  fprintf(stderr, "%s SIZE_SZ: %lu, SIZE_SZ << 1: %lu\n",
          __FUNCTION__, SIZE_SZ, SIZE_SZ << 1);

  /*
  for (i = 0; i < NBINS*2; ++i)
  {
    fprintf(stderr, "bins[%d] = %p\n", i, &(av->bins[i]));
  }
  for (i = 1; i < NBINS; ++i)
  {
    fprintf(stderr, "bin_at(i = %d) = %p\n", i, bin_at(av, i));
  }
  */

  // malloc_state => mchunkptr bins[NBINS * 2];
  fprintf(stderr, "%s sizeof(mchunkptr): %zu\n", __FUNCTION__, sizeof(mchunkptr));
  for (i = 1; i < NBINS; ++i)
  {
    // bin_at(1) => bins[0]
    // bin_at(2) => bins[2]
    // bin_at(3) => bins[4]
    // ...
    // bin_at(NBINS - 1: 95) => bins[188]
    // bins空间是从0到191
    bin = bin_at(av, i);
    bin->fd = bin->bk = bin;
    // 每次循环到x处，即将x+2，x+3的地址设置为x的地址
  }

  av->top_pad = DEFAULT_TOP_PAD;
  av->n_mmaps_max = DEFAULT_MMAP_MAX;
  av->mmap_threshold = DEFAULT_MMAP_THRESHOLD;
  av->trim_threshold = DEFAULT_TRIM_THRESHOLD;

#if MORECORE_CONTIGUOUS
  // 设置av->morecore_properties的MORECORE_CONTIGUOUS_BIT属性
  set_contiguous(av);
#else
  // 清理av->morecore_properties的MORECORE_CONTIGUOUS_BIT属性
  set_noncontiguous(av);
#endif

  for (int i = 1; i <= DEFAULT_MXFAST; ++i)
  {
    set_max_fast(av, i);
    fprintf(stderr, "(i: %d) => (max_fast: %d)\n", i, av->max_fast);
  }

  set_max_fast(av, DEFAULT_MXFAST);

  // top设置到bins[0]的地址.
  av->top = initial_top(av);
  av->pagesize = malloc_getpagesize;
}

// consolidate: 加强/巩固/合并
static void malloc_consolidate(mstate av)
{
  mfastbinptr* fb;              // current fastbin being consolidated
  mfastbinptr* maxfb;           // last fastbin (for loop control)
  mchunkptr p;                  // current chunk being consolidated
  mchunkptr nextp;              // next chunk to consolidate
  mchunkptr unsorted_bin;       // bin header
  mchunkptr first_unsorted;     // chunk to link to

  mchunkptr nextchunk;          // These have same use as in free()
  size_t size;
  size_t nextsize;
  size_t prevsize;
  int nextinuse;
  mchunkptr bck;
  mchunkptr fwd;

  // If max_fast is 0, we know that av hasn't
  // yet been initialized, in which case do so below
  // av->max_fast != 0，说明已经被初始化过。
  if (av->max_fast != 0)
  {
    clear_fastchunks(av);

    unsorted_bin = unsorted_chunks(av);

    maxfb = &(av->fastbins[fastbin_index(av->max_fast)]);
    fb = &(av->fastbins[0]);
    do
    {
      if ((p = *fb) != 0)
      {
        *fb = 0;
        do
        {
          nextp = p->fd;

          size = p->size & ~PREV_INUSE;
          nextchunk = chunk_at_offset(p, size);
          nextsize = chunksize(nextchunk);

          if (!prev_inuse(p))
          {
            prevsize = p->prev_size;
            size += prevsize;
            p = chunk_at_offset(p, -((long)prevsize));
            unlink(p, bck, fwd);
          }

          if (nextchunk != av->top)
          {
            nextinuse = inuse_bit_at_offset(nextchunk, nextsize);
            set_head(nextchunk, nextsize);

            if (!nextinuse)
            {
              size += nextsize;
              unlink(nextchunk, bck, fwd);
            }

            first_unsorted = unsorted_bin->fd;
            unsorted_bin->fd = p;
            first_unsorted->bk = p;

            set_head(p, size | PREV_INUSE);
            p->bk = unsorted_bin;
            p->fd = first_unsorted;
            set_foot(p, size);
          }
          else
          {
            size += nextsize;
            set_head(p, size | PREV_INUSE);
            av->top = p;
          }
        } while ((p = nextp) != 0);
      }
    } while (fb++ != maxfb);
  }
  // av->max_fast == 0，说明没有被初始化过。
  else
  {
    malloc_init_state(av);
  }
}

static int largebin_index(unsigned int sz)
{
  unsigned int x = sz >> SMALLBIN_WIDTH;
  unsigned int m;

  if (x >= 0x10000)
  {
    return NBINS - 1;
  }

  {
    unsigned int n = ((x - 0x100) >> 16) & 8;
    x <<= n;
    m = ((x - 0x1000) >> 16) & 4;
    n += m;
    x <<= m;
    m = ((x - 0x4000) >> 16) & 2;
    n += m;
    x = (x << m) >> 14;
    m = 13 - n + (x & ~(x >> 1));
  }

  // use next 2 bins to create finer-granularity bins.
  return NSMALLBINS + (m << 2) + ((sz >> (m + 6)) & 3);
}

static void* sysmalloc(size_t nb, mstate av)
{
  mchunkptr old_top;  // incoming value of av->top
  size_t old_size;    // its size
  char* old_end;      // its end address

  long size;          // arg to first MORECORE or mmap call
  char* brk;          // return value from MORECORE

  long correction;    // arg to 2nd MORECORE call
  char* snd_brk;      // 2nd return val

  size_t front_misalign;  // unusable bytes at front of new space
  size_t end_misalign;    // partial page left at end of new space
  char* aligned_brk;      // aligned offset into brk

  mchunkptr p;                    // the allocated/returned chunk
  mchunkptr remainder;            // remainder from allocation
  CHUNK_SIZE_T remainder_size;    // its size

  CHUNK_SIZE_T sum;               // for updating stats

  size_t pagemask = av->pagesize - 1;

  if (have_fastchunks(av))
  {
    malloc_consolidate(av);
    return iMalloc(nb - MALLOC_ALIGN_MASK);
  }

#if HAVE_MMAP
  if ((CHUNK_SIZE_T)(nb) >= (CHUNK_SIZE_T)(av->mmap_threshold) && (av->n_mmaps < av->n_mmaps_max))
  {
    char* mm;
    size = (nb + SIZE_SZ + MALLOC_ALIGN_MASK + pagemask) & ~pagemask;

    if ((CHUNK_SIZE_T)(size) > (CHUNK_SIZE_T)(nb))
    {
      mm = (char*)(MMAP(0, size, PROT_READ|PROT_WRITE, MAP_PRIVATE));
      if (mm != (char*)(MORECORE_FAILURE))
      {
        front_misalign = (size_t)chunk2mem(mm) & MALLOC_ALIGN_MASK;
        if (front_misalign > 0)
        {
          correction = MALLOC_ALIGNMENT - front_misalign;
          p = (mchunkptr)(mm + correction);
          p->prev_size = correction;
          set_head(p, (size - correction) | IS_MMAPPED);
        }
        else
        {
          p = (mchunkptr)mm;
          p->prev_size = 0;
          set_head(p, size | IS_MMAPPED);
        }

        if (++av->n_mmaps > av->max_n_mmaps)
        {
          av->max_n_mmaps = av->n_mmaps;
        }

        sum = av->mmapped_mem += size;
        if (sum > (CHUNK_SIZE_T)(av->max_mmapped_mem))
        {
          av->max_mmapped_mem = sum;
        }
        sum += av->sbrked_mem;
        if (sum > (CHUNK_SIZE_T)(av->max_total_mem))
        {
          av->max_total_mem = sum;
        }

        return chunk2mem(p);
      }
    }
  }
#endif
  old_top = av->top;
  old_size = chunksize(old_top);
  old_end = (char*)(chunk_at_offset(old_top, old_size));

  brk = snd_brk = (char*)(MORECORE_FAILURE);

  // Request enough space for nb + pad + overhead.
  size = nb + av->top_pad + MINSIZE;

  if (contiguous(av))
  {
    size -= old_size;
  }

  size = (size + pagemask) & ~pagemask;

  if (size > 0)
  {
      brk = (char*)(MORECORE(size));
  }

#if HAVE_MMAP
  if (brk == (char*)(MORECORE_FAILURE))
  {
    if (contiguous(av))
    {
      size = (size + old_size + pagemask) & ~pagemask;
    }

    if ((CHUNK_SIZE_T)(size) < (CHUNK_SIZE_T)(MMAP_AS_MORECORE_SIZE))
    {
      size = MMAP_AS_MORECORE_SIZE;
    }

    if ((CHUNK_SIZE_T)(size) > (CHUNK_SIZE_T)(nb))
    {
      brk = (char*)(MMAP(0, size, PROT_READ | PROT_WRITE, MAP_PRIVATE));
      if (brk != (char*)(MORECORE_FAILURE))
      {
        snd_brk = brk + size;
        set_noncontiguous(av);
      }
    }
  }
#endif

  if (brk != (char*)(MORECORE_FAILURE))
  {
    av->sbrked_mem += size;

    if (brk == old_end && snd_brk == (char*)(MORECORE_FAILURE))
    {
      set_head(old_top, (size + old_size) | PREV_INUSE);
    }
    else
    {
      front_misalign = 0;
      end_misalign = 0;
      correction = 0;
      aligned_brk = brk;

      if (contiguous(av) && old_size != 0 && brk < old_end)
      {
        set_noncontiguous(av);
      }

      if (contiguous(av))
      {
        if (old_size != 0)
        {
          av->sbrked_mem += brk - old_end;
        }

        front_misalign = (size_t)chunk2mem(brk) & MALLOC_ALIGN_MASK;
        if (front_misalign > 0)
        {
          correction = MALLOC_ALIGNMENT - front_misalign;
          aligned_brk += correction;
        }

        correction += old_size;

        end_misalign = (size_t)(brk + size + correction);
        correction += ((end_misalign + pagemask) & ~pagemask) - end_misalign;

        snd_brk = (char*)(MORECORE(correction));

        if (snd_brk == (char*)(MORECORE_FAILURE))
        {
          correction = 0;
          snd_brk = (char*)(MORECORE(0));
        }
        else if (snd_brk < brk)
        {
          snd_brk = brk + size;
          correction = 0;
          set_noncontiguous(av);
        }
      }
      else
      {
        if (snd_brk == (char*)(MORECORE_FAILURE))
        {
          snd_brk = (char*)(MORECORE(0));
          av->sbrked_mem += snd_brk - brk - size;
        }
      }

      if (snd_brk != (char*)(MORECORE_FAILURE))
      {
        av->top = (mchunkptr)aligned_brk;
        set_head(av->top, (snd_brk - aligned_brk + correction) | PREV_INUSE);
        av->sbrked_mem += correction;

        if (old_size != 0)
        {
          old_size = (old_size - 3 * SIZE_SZ) & ~MALLOC_ALIGN_MASK;
          set_head(old_top, old_size | PREV_INUSE);

          chunk_at_offset(old_top, old_size)->size = SIZE_SZ | PREV_INUSE;
          chunk_at_offset(old_top, old_size + SIZE_SZ)->size = SIZE_SZ | PREV_INUSE;

          if (old_size >= MINSIZE)
          {
            size_t tt = av->trim_threshold;
            av->trim_threshold = (size_t)(-1);
            iFree(chunk2mem(old_top));
            av->trim_threshold = tt;
          }
        }
      }
    }

    sum = av->sbrked_mem;
    if (sum > (CHUNK_SIZE_T)(av->max_sbrked_mem))
    {
      av->max_sbrked_mem = sum;
    }

    sum += av->mmapped_mem;
    if (sum > (CHUNK_SIZE_T)(av->max_total_mem))
    {
      av->max_total_mem = sum;
    }

    p = av->top;
    size = chunksize(p);

    if ((CHUNK_SIZE_T)(size) >= (CHUNK_SIZE_T)(nb + MINSIZE))
    {
      remainder_size = size - nb;
      remainder = chunk_at_offset(p, nb);
      av->top = remainder;
      set_head(p, nb | PREV_INUSE);
      set_head(remainder, remainder_size | PREV_INUSE);
      return chunk2mem(p);
    }
  }

  MALLOC_FAILURE_ACTION
  return 0;
}

void* iMalloc(size_t bytes)
{
  mstate av = get_malloc_state();

  size_t nb;          // normalized request size
  unsigned int idx;   // associated bin index
  mbinptr bin;        // associated bin
  mfastbinptr* fb;    // associated fastbin

  mchunkptr victim;   // inspected/selected chunk
  size_t size;        // its size
  int victim_index;   // its bin index

  mchunkptr remainder;            // remainder from a split
  CHUNK_SIZE_T remainder_size;    // its size

  unsigned int block;     // bit map traverser
  unsigned int bit;       // bit map traverser
  unsigned int map;       // current word of binmap

  mchunkptr fwd;          // misc temp for linking
  mchunkptr bck;          // misc temp for linking

  fprintf(stderr, "==========================================================\n");
  fprintf(stderr, "MIN_CHUNK_SIZE(sizeof(struct malloc_chunk)): %lu\n", MIN_CHUNK_SIZE);
  fprintf(stderr, "MALLOC_ALIGNMENT(2 * sizeof(size_t)): %lu\n", MALLOC_ALIGNMENT);
  fprintf(stderr, "MALLOC_ALIGN_MASK(MALLOC_ALIGNMENT - 1): %lu\n", MALLOC_ALIGN_MASK);
  fprintf(stderr, "MINSIZE((unsigned long)((MIN_CHUNK_SIZE + MALLOC_ALIGN_MASK) & ~MALLOC_ALIGN_MASK)): %lu\n", MINSIZE);
  fprintf(stderr, "=====================\n");
  fprintf(stderr, "MAX_FAST_SIZE: %d\n", MAX_FAST_SIZE);
  fprintf(stderr, "request2size(MAX_FAST_SIZE): %lu\n", request2size(MAX_FAST_SIZE));
  fprintf(stderr, "fastbin_index(request2size(MAX_FAST_SIZE)): %d\n", fastbin_index(request2size(MAX_FAST_SIZE)));
  fprintf(stderr, "NFASTBINS(fastbin_index(request2size(MAX_FAST_SIZE)) + 1): %d\n", fastbin_index(request2size(MAX_FAST_SIZE)) + 1);
  fprintf(stderr, "=====================\n");
  fprintf(stderr, "NBINS: %d\n", NBINS);
  fprintf(stderr, "BITSPERMAP(1 << BINMAPSHIFT(5)): %d\n", BITSPERMAP);
  fprintf(stderr, "BINMAPSIZE(NBINS/BITSPERMAP): %d\n", BINMAPSIZE);
  fprintf(stderr, "==========================================================\n");

  if ((CHUNK_SIZE_T)(bytes) >= (CHUNK_SIZE_T)(size_t)(-2 * MINSIZE))
  {
    errno = ENOMEM;
    return 0;
  }

  // 将申请的内存大小bytes，切换成内部实际申请的内存大小nb
  nb = request2size(bytes);
  fprintf(stderr, "from input: %zu, get nb: %zu\n", bytes, nb);

  // Bypass search if no frees yet
  fprintf(stderr, "av->mx_fast: %zu, mx_fast & ANYCHUNKS_BIT(1U): %lu, have_anychunks: %s\n",
          av->max_fast, (av->max_fast & ANYCHUNKS_BIT), (av->max_fast & ANYCHUNKS_BIT) ? "TRUE" : "FALSE");

  /*
  for (int i = 0; i < 10; ++i)
  {
    fprintf(stderr, "i: %d => (i(%d) & ANYCHUNKS_BIT(%u)) = %d\n", i, i, ANYCHUNKS_BIT, (i & ANYCHUNKS_BIT));
  }
  */

  // have_anychunks == 0(false)，说明av->max_fast为偶数，即0，2，4，6，8，...
  // have_anychunks == 1(true)，说明av->max_fast为奇数，即1，3，5，7，9，...
  if (!have_anychunks(av))
  {
    // 这里的av->max_fast为偶数
    if (av->max_fast == 0)    // initialization check
    {
      // 这里的av->max_fast == 0
      malloc_consolidate(av);
    }
    goto use_top;
  }

  if ((CHUNK_SIZE_T)(nb) <= (CHUNK_SIZE_T)(av->max_fast))
  {
    fb = &(av->fastbins[(fastbin_index(nb))]);
    if ((victim = *fb) != 0)
    {
      *fb = victim->fd;
      return chunk2mem(victim);
    }
  }

  if (in_smallbin_range(nb))
  {
    idx = smallbin_index(nb);
    bin = bin_at(av, idx);

    if ((victim = last(bin)) != bin)
    {
      bck = victim->bk;
      set_inuse_bit_at_offset(victim, nb);
      bin->bk = bck;
      bck->fd = bin;

      return chunk2mem(victim);
    }
  }
  else
  {
    idx = largebin_index(nb);
    if (have_fastchunks(av))
    {
      malloc_consolidate(av);
    }
  }

  while ((victim = unsorted_chunks(av)->bk) != unsorted_chunks(av))
  {
    bck = victim->bk;
    size = chunksize(victim);

    if (in_smallbin_range(nb) && bck == unsorted_chunks(av)
      && victim == av->last_remainder && (CHUNK_SIZE_T)(size) > (CHUNK_SIZE_T)(nb + MINSIZE))
    {
      remainder_size = size - nb;
      remainder = chunk_at_offset(victim, nb);
      unsorted_chunks(av)->bk = unsorted_chunks(av)->fd = remainder;
      av->last_remainder = remainder;
      remainder->bk = remainder->fd = unsorted_chunks(av);

      set_head(victim, nb | PREV_INUSE);
      set_head(remainder, remainder_size | PREV_INUSE);
      set_foot(remainder, remainder_size);

      return chunk2mem(victim);
    }

    // remove from unsorted list.
    unsorted_chunks(av)->bk = bck;
    bck->fd = unsorted_chunks(av);

    if (size == nb)
    {
      set_inuse_bit_at_offset(victim, size);
      return chunk2mem(victim);
    }

    if (in_smallbin_range(size))
    {
      victim_index = smallbin_index(size);
      bck = bin_at(av, victim_index);
      fwd = bck->fd;
    }
    else
    {
      victim_index = largebin_index(size);
      bck = bin_at(av, victim_index);
      fwd = bck->fd;

      if (fwd != bck)
      {
        if ((CHUNK_SIZE_T)(size) < (CHUNK_SIZE_T)(bck->bk->size))
        {
          fwd = bck;
          bck = bck->bk;
        }
        else if ((CHUNK_SIZE_T)(size) >= (CHUNK_SIZE_T)(FIRST_SORTED_BIN_SIZE))
        {
          size |= PREV_INUSE;
          while ((CHUNK_SIZE_T)(size) < (CHUNK_SIZE_T)(fwd->size))
          {
            fwd = fwd->fd;
          }
          bck = fwd->bk;
        }
      }
    }

    mark_bin(av, victim_index);
    victim->bk = bck;
    victim->fd = fwd;
    fwd->bk = victim;
    bck->fd = victim;
  }

  if (!in_smallbin_range(nb))
  {
    bin = bin_at(av, idx);
    for (victim = last(bin); victim != bin; victim = victim->bk)
    {
      size = chunksize(victim);

      if ((CHUNK_SIZE_T)(size) >= (CHUNK_SIZE_T)(nb))
      {
        remainder_size = size - nb;
        unlink(victim, bck, fwd);

        if (remainder_size < MINSIZE)
        {
          set_inuse_bit_at_offset(victim, size);
          return chunk2mem(victim);
        }
        else
        {
          remainder = chunk_at_offset(victim, nb);
          unsorted_chunks(av)->bk = unsorted_chunks(av)->fd = remainder;
          remainder->bk = remainder->fd = unsorted_chunks(av);
          set_head(victim, nb | PREV_INUSE);
          set_head(remainder, remainder_size);
          set_foot(remainder, remainder_size);
          return chunk2mem(victim);
        }
      }
    }
  }

  ++idx;
  bin = bin_at(av, idx);
  block = idx2block(idx);
  map = av->binmap[block];
  bit = idx2bit(idx);

  for (;;)
  {
    if (bit > map || bit == 0)
    {
      do
      {
        if (++block >= BINMAPSIZE)
        {
          goto use_top;
        }
      } while ((map = av->binmap[block]) == 0);

      bin = bin_at(av, (block << BINMAPSHIFT));
      bit = 1;
    }

    while ((bit & map) == 0)
    {
      bin = next_bin(bin);
      bit <<= 1;
    }

    victim = last(bin);

    if (victim == bin)
    {
      av->binmap[block] = map &= ~bit;
      bin = next_bin(bin);
      bit <<= 1;
    }
    else
    {
      size = chunksize(victim);

      assert((CHUNK_SIZE_T)(size) >= (CHUNK_SIZE_T)(nb));

      remainder_size = size - nb;

      bck = victim->bk;
      bin->bk = bck;
      bck->fd = bin;

      if (remainder_size < MINSIZE)
      {
        set_inuse_bit_at_offset(victim, size);
        return chunk2mem(victim);
      }
      else
      {
        remainder = chunk_at_offset(victim, nb);
        unsorted_chunks(av)->bk = unsorted_chunks(av)->fd = remainder;
        remainder->bk = remainder->fd = unsorted_chunks(av);

        if (in_smallbin_range(nb))
        {
          av->last_remainder = remainder;
        }

        set_head(victim, nb | PREV_INUSE);
        set_head(remainder, remainder_size | PREV_INUSE);
        set_foot(remainder, remainder_size);
        return chunk2mem(victim);
      }
    }
  }

  use_top:
  victim = av->top;
  size = chunksize(victim);

  if ((CHUNK_SIZE_T)(size) >= (CHUNK_SIZE_T)(nb + MINSIZE))
  {
    remainder_size = size - nb;
    remainder = chunk_at_offset(victim, nb);
    av->top = remainder;
    set_head(victim, nb | PREV_INUSE);
    set_head(remainder, remainder_size | PREV_INUSE);

    return chunk2mem(victim);
  }

  // If no space in top, relay to handle system-dependent cases.
  return sysmalloc(nb, av);
}

void iFree(void* mem)
{

}