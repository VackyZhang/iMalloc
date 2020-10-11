#include "imalloc.h"
#include <cstdio>
#include <cerrno>
#include <unistd.h>

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

#define DEFAULT_TOP_PAD   (0)

#ifndef HAVE_MMAP
#define HAVE_MMAP 1
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

#define MIN_CHUNK_SIZE (sizeof(struct malloc_chunk))

#define NBINS   96

// Conservatively use 32 bits per map word, even if on 64bit system.
#define BINMAPSHIFT 5
#define BITSPERMAP (1U << BINMAPSHIFT)
#define BINMAPSIZE (NBINS / BITSPERMAP)

#define MALLOC_ALIGNMENT  (2 * SIZE_SZ)
#define MALLOC_ALIGN_MASK (MALLOC_ALIGNMENT - 1)
#define MINSIZE (CHUNK_SIZE_T)(((MIN_CHUNK_SIZE+MALLOC_ALIGN_MASK) & ~MALLOC_ALIGN_MASK))

#define MAX_FAST_SIZE   80
// offset 2 to use otherwise unindexable first 2 bins.
#define fastbin_index(sz) ((((unsigned int)(sz)) >> 3) - 2)

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

// The otherwise unindexable 1-bin is used to hold unsorted chunks.
#define unsorted_chunks(M)  (bin_at(M, 1))

// Conveniently, the unsorted bin can be used as dummy top on first call.
#define initial_top(M)  (unsorted_chunks(M))

#define chunk_at_offset(p, s) ((mchunkptr)(((char*)(p)) + (s)))

// check/set/clear inuse bits in known places.
#define inuse_bit_at_offset(p, s)   (((mchunkptr)(((char*)(p)) + (s)))->size & PREV_INUSE)

// Set size/use field.
#define set_head(p, s)  ((p)->size = (s))

#define set_foot(p, s)  (((mchunkptr)((char*)(p) + (s)))->prev_size = (s))

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
static void malloc_init_state(mstate av)
{
  int i;
  mbinptr bin;

  // Establish circular links for normal bins.
  fprintf(stderr, "[%s] NBINS: %d\n", __FUNCTION__, NBINS);
  for (i = 1; i < NBINS; ++i)
  {
    bin = bin_at(av, i);
    bin->fd = bin->bk = bin;
  }

  av->top_pad = DEFAULT_TOP_PAD;
  av->n_mmaps_max = DEFAULT_MMAP_MAX;
  av->mmap_threshold = DEFAULT_MMAP_THRESHOLD;
  av->trim_threshold = DEFAULT_TRIM_THRESHOLD;

#if MORECORE_CONTIGUOUS
  set_contiguous(av);
#else
  set_noncontiguous(av);
#endif

  set_max_fast(av, DEFAULT_MXFAST);

  av->top = initial_top(av);
  av->pagesize = malloc_getpagesize;
}

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
  else
  {
    malloc_init_state(av);
  }
}

void* imalloc(size_t bytes)
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

//fprintf(stderr, "==> (unsigned long)(size_t)(-2 * MINSIZE): %llu\n", (CHUNK_SIZE_T)(size_t)(-2 * MINSIZE));
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
  if (!have_anychunks(av))
  {
    if (av->max_fast == 0)    // initialization check
    {
      malloc_consolidate(av);
    }
    goto use_top;
  }

  use_top:
  return NULL;
}