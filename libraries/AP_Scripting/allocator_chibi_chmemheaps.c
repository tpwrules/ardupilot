/*
    ChibiOS - Copyright (C) 2006,2007,2008,2009,2010,2011,2012,2013,2014,
              2015,2016,2017,2018,2019,2020,2021 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    oslib/src/chmemheaps.c
 * @brief   Memory heaps code.
 *
 * @addtogroup oslib_memheaps
 * @details Heap Allocator related APIs.
 *          <h2>Operation mode</h2>
 *          The heap allocator implements a first-fit strategy and its APIs
 *          are functionally equivalent to the usual @p malloc() and @p free()
 *          library functions. The main difference is that the OS heap APIs
 *          are guaranteed to be thread safe and there is the ability to
 *          return memory blocks aligned to arbitrary powers of two.<br>
 * @pre     In order to use the heap APIs the @p CH_CFG_USE_HEAP option must
 *          be enabled in @p chconf.h.
 * @note    Compatible with RT and NIL.
 * @{
 */

#include "allocator_chibi_chmemheaps.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

#define H_LOCK(h)       //(void) chSemWait(&(h)->sem)
#define H_UNLOCK(h)     //chSemSignal(&(h)->sem)

#define H_BLOCK(hp)     ((hp) + 1U)

#define H_LIMIT(hp)     (H_BLOCK(hp) + H_PAGES(hp))

#define H_NEXT_GET(hp)  ((hp)->free.next)
#define H_NEXT_SET(hp, next_) do { (hp)->free.next = (next_); } while (0)
#define H_NEXT_EXISTS(hp)   ((hp)->free.next != NULL)

#define H_PAGES(hp)     ((hp)->free.pages)

#define H_HEAP_GET(hp)  ((hp)->used.heap)
#define H_HEAP_SET(hp, heap_) do { (hp)->used.heap = (heap_); } while (0)

#define H_SIZE(hp)      ((hp)->used.size)

/*
 * Number of pages between two pointers in a MISRA-compatible way.
 */
#define NPAGES(p1, p2)                                                      \
  /*lint -save -e9033 [10.8] The cast is safe.*/                            \
  ((size_t)((p1) - (p2)))                                                   \
  /*lint -restore*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes a memory heap from a static memory area.
 * @note    The heap buffer base and size are adjusted if the passed buffer
 *          is not aligned to @p SC_CH_HEAP_ALIGNMENT. This mean that the
 *          effective heap size can be less than @p size.
 *
 * @param[out] heapp    pointer to the memory heap descriptor to be initialized
 * @param[in] buf       heap buffer base
 * @param[in] size      heap size
 *
 * @init
 */
void scChHeapObjectInit(sc_memory_heap_t *heapp, void *buf, size_t size, uint8_t index) {
  sc_heap_header_t *hp = (sc_heap_header_t *)SC_MEM_ALIGN_NEXT(buf, SC_CH_HEAP_ALIGNMENT);

  //chDbgCheck((heapp != NULL) && (size > 0U));

  /* Adjusting the size in case the initial block was not correctly
     aligned.*/
  /*lint -save -e9033 [10.8] Required cast operations.*/
  size -= (size_t)((uint8_t *)hp - (uint8_t *)buf);
  /*lint restore*/

  /* Initializing the heap header.*/
  //heapp->provider = NULL;
  H_NEXT_SET(&heapp->header, hp);
  H_PAGES(&heapp->header) = 0;
  H_NEXT_SET(hp, NULL);
  H_PAGES(hp) = (size - sizeof (sc_heap_header_t)) / SC_CH_HEAP_ALIGNMENT;
// #if (CH_CFG_USE_MUTEXES == TRUE) || defined(__DOXYGEN__)
//   chMtxObjectInit(&heapp->mtx);
// #else
//   chSemObjectInit(&heapp->sem, (cnt_t)1);
// #endif
}

/**
 * @brief   Allocates a block of memory from the heap by using the first-fit
 *          algorithm.
 * @details The allocated block is guaranteed to be properly aligned to the
 *          specified alignment.
 *
 * @param[in] heapp     pointer to a heap descriptor or @p NULL in order to
 *                      access the default heap.
 * @param[in] size      the size of the block to be allocated. Note that the
 *                      allocated block may be a bit bigger than the requested
 *                      size for alignment and fragmentation reasons.
 * @param[in] align     desired memory alignment
 * @return              A pointer to the aligned allocated block.
 * @retval NULL         if the block cannot be allocated.
 *
 * @api
 */
void *scChHeapAllocAligned(sc_memory_heap_t *heapp, size_t size, unsigned align) {
  sc_heap_header_t *qp, *hp, *ahp;
  size_t pages;

  //chDbgCheck((size > 0U) && MEM_IS_VALID_ALIGNMENT(align));

  // /* If an heap is not specified then the default system header is used.*/
  // if (heapp == NULL) {
  //   heapp = &default_heap;
  // }

  /* Minimum alignment is constrained by the heap header structure size.*/
  if (align < SC_CH_HEAP_ALIGNMENT) {
    align = SC_CH_HEAP_ALIGNMENT;
  }

  /* Size is converted in number of elementary allocation units.*/
  pages = SC_MEM_ALIGN_NEXT(size, SC_CH_HEAP_ALIGNMENT) / SC_CH_HEAP_ALIGNMENT;

  /* Taking heap mutex/semaphore.*/
  H_LOCK(heapp);

  /* Start of the free blocks list.*/
  qp = &heapp->header;
  while (H_NEXT_EXISTS(qp)) {

    /* Next free block.*/
    hp = H_NEXT_GET(qp);

    /* Pointer aligned to the requested alignment.*/
    ahp = (sc_heap_header_t *)SC_MEM_ALIGN_NEXT(H_BLOCK(hp), align) - 1U;

    if ((ahp < H_LIMIT(hp)) && (pages <= NPAGES(H_LIMIT(hp), ahp + 1U))) {
      /* The block is large enough to contain a correctly aligned area
         of sufficient size.*/

      if (ahp > hp) {
        /* The block is not properly aligned, must split it.*/
        size_t bpages;

        bpages = NPAGES(H_LIMIT(hp), H_BLOCK(ahp));
        H_PAGES(hp) = NPAGES(ahp, H_BLOCK(hp));
        if (bpages > pages) {
          /* The block is bigger than required, must split the excess.*/
          sc_heap_header_t *fp;

          /* Creating the excess block.*/
          fp = H_BLOCK(ahp) + pages;
          H_PAGES(fp) = (bpages - pages) - 1U;

          /* Linking the excess block.*/
          H_NEXT_SET(fp, H_NEXT_GET(hp));
          H_NEXT_SET(hp, fp);
        }

        hp = ahp;
      }
      else {
        /* The block is already properly aligned.*/

        if (H_PAGES(hp) == pages) {
          /* Exact size, getting the whole block.*/
          H_NEXT_SET(qp, H_NEXT_GET(hp));
        }
        else {
          /* The block is bigger than required, must split the excess.*/
          sc_heap_header_t *fp;

          fp = H_BLOCK(hp) + pages;
          H_NEXT_SET(fp, H_NEXT_GET(hp));
          H_PAGES(fp) = NPAGES(H_LIMIT(hp), H_BLOCK(fp));
          H_NEXT_SET(qp, fp);
        }
      }

      /* Setting in the block owner heap and size.*/
      H_SIZE(hp) = size;
      H_HEAP_SET(hp, heapp);

      /* Releasing heap mutex/semaphore.*/
      H_UNLOCK(heapp);

      /*lint -save -e9087 [11.3] Safe cast.*/
      return (void *)H_BLOCK(hp);
      /*lint -restore*/
    }

    /* Next in the free blocks list.*/
    qp = hp;
  }

  /* Releasing heap mutex/semaphore.*/
  H_UNLOCK(heapp);

  /* More memory is required, tries to get it from the associated provider
     else fails.*/
  // if (heapp->provider != NULL) {
  //   ahp = heapp->provider(pages * SC_CH_HEAP_ALIGNMENT,
  //                         align,
  //                         sizeof (sc_heap_header_t));
  //   if (ahp != NULL) {
  //     hp = ahp - 1U;
  //     H_HEAP(hp) = heapp;
  //     H_SIZE(hp) = size;

  //     /*lint -save -e9087 [11.3] Safe cast.*/
  //     return (void *)ahp;
  //     /*lint -restore*/
  //   }
  // }

  return NULL;
}

/**
 * @brief   Frees a previously allocated memory block.
 *
 * @param[in] p         pointer to the memory block to be freed
 *
 * @api
 */
void scChHeapFree(void *p) {
  sc_heap_header_t *qp, *hp;
  sc_memory_heap_t *heapp;

  //chDbgCheck((p != NULL) && MEM_IS_ALIGNED(p, SC_CH_HEAP_ALIGNMENT));

  /*lint -save -e9087 [11.3] Safe cast.*/
  hp = (sc_heap_header_t *)p - 1U;
  /*lint -restore*/
  heapp = H_HEAP_GET(hp);
  qp = &heapp->header;

  /* Size is converted in number of elementary allocation units.*/
  H_PAGES(hp) = SC_MEM_ALIGN_NEXT(H_SIZE(hp),
                               SC_CH_HEAP_ALIGNMENT) / SC_CH_HEAP_ALIGNMENT;

  /* Taking heap mutex/semaphore.*/
  H_LOCK(heapp);

  while (true) {
    //chDbgAssert((hp < qp) || (hp >= H_LIMIT(qp)), "within free block");

    if (((qp == &heapp->header) || (hp > qp)) &&
        ((!H_NEXT_EXISTS(qp)) || (hp < H_NEXT_GET(qp)))) {
      /* Insertion after qp.*/
      H_NEXT_SET(hp, H_NEXT_GET(qp));
      H_NEXT_SET(qp, hp);
      /* Verifies if the newly inserted block should be merged.*/
      if (H_LIMIT(hp) == H_NEXT_GET(hp)) {
        /* Merge with the next block.*/
        H_PAGES(hp) += H_PAGES(H_NEXT_GET(hp)) + 1U;
        H_NEXT_SET(hp, H_NEXT_GET(H_NEXT_GET(hp)));
      }
      if ((H_LIMIT(qp) == hp)) {
        /* Merge with the previous block.*/
        H_PAGES(qp) += H_PAGES(hp) + 1U;
        H_NEXT_SET(qp, H_NEXT_GET(hp));
      }
      break;
    }
    qp = H_NEXT_GET(qp);
  }

  /* Releasing heap mutex/semaphore.*/
  H_UNLOCK(heapp);

  return;
}

/**
 * @brief   Reports the heap status.
 * @note    This function is meant to be used in the test suite, it should
 *          not be really useful for the application code.
 *
 * @param[in] heapp     pointer to a heap descriptor or @p NULL in order to
 *                      access the default heap.
 * @param[in] totalp    pointer to a variable that will receive the total
 *                      fragmented free space or @p NULL
 * @param[in] largestp  pointer to a variable that will receive the largest
 *                      free free block found space or @p NULL
 * @return              The number of fragments in the heap.
 *
 * @api
 */
size_t scChHeapStatus(sc_memory_heap_t *heapp, size_t *totalp, size_t *largestp) {
  sc_heap_header_t *qp;
  size_t n, tpages, lpages;

  H_LOCK(heapp);
  tpages = 0U;
  lpages = 0U;
  n = 0U;
  qp = &heapp->header;
  while (H_NEXT_EXISTS(qp)) {
    size_t pages = H_PAGES(H_NEXT_GET(qp));

    /* Updating counters.*/
    n++;
    tpages += pages;
    if (pages > lpages) {
      lpages = pages;
    }

    qp = H_NEXT_GET(qp);
  }

  /* Writing out fragmented free memory.*/
  if (totalp != NULL) {
    *totalp = tpages * SC_CH_HEAP_ALIGNMENT;
  }

  /* Writing out unfragmented free memory.*/
  if (largestp != NULL) {
    *largestp = lpages * SC_CH_HEAP_ALIGNMENT;
  }
  H_UNLOCK(heapp);

  return n;
}

/** @} */
