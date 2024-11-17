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
 * @file    oslib/include/chmemheaps.h
 * @brief   Memory heaps macros and structures.
 *
 * @addtogroup oslib_memheaps
 * @{
 */

#ifndef AP_SCRIPTING_CHMEMHEAPS_H
#define AP_SCRIPTING_CHMEMHEAPS_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/**
 * @name    Memory alignment support macros
 * @{
 */
/**
 * @brief   Alignment mask constant.
 *
 * @param[in] a         alignment, must be a power of two
 */
#define SC_MEM_ALIGN_MASK(a)       ((size_t)(a) - 1U)

/**
 * @brief   Aligns to the previous aligned memory address.
 *
 * @param[in] p         variable to be aligned
 * @param[in] a         alignment, must be a power of two
 */
#define SC_MEM_ALIGN_PREV(p, a)                                                \
  /*lint -save -e9033 [10.8] The cast is safe.*/                            \
  ((size_t)(p) & ~SC_MEM_ALIGN_MASK(a))                                        \
  /*lint -restore*/

/**
 * @brief   Aligns to the next aligned memory address.
 *
 * @param[in] p         variable to be aligned
 * @param[in] a         alignment, must be a power of two
 */
#define SC_MEM_ALIGN_NEXT(p, a)                                                \
  /*lint -save -e9033 [10.8] The cast is safe.*/                            \
  SC_MEM_ALIGN_PREV((size_t)(p) + SC_MEM_ALIGN_MASK(a), (a))                      \
  /*lint -restore*/

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Minimum alignment used for heap.
 * @note    Cannot use the sizeof operator in this macro.
 */
// #if (SIZEOF_PTR == 4) || defined(__DOXYGEN__)
// #define SC_CH_HEAP_ALIGNMENT   8U
// #elif (SIZEOF_PTR == 2)
// #define SC_CH_HEAP_ALIGNMENT   4U
// #else
// #error "unsupported pointer size"
// #endif

#define SC_CH_HEAP_ALIGNMENT 16U

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a memory heap.
 */
typedef struct sc_memory_heap sc_memory_heap_t;

/**
 * @brief   Type of a memory heap header.
 */
typedef union sc_heap_header sc_heap_header_t;

/**
 * @brief   Memory heap block header.
 */
union sc_heap_header {
  struct {
    sc_heap_header_t       *next;      /**< @brief Next block in free list.    */
    size_t              pages;      /**< @brief Size of the area in pages.  */
  } free;
  struct {
    sc_memory_heap_t       *heap;      /**< @brief Block owner heap.           */
    size_t              size;       /**< @brief Size of the area in bytes.  */
  } used;
};

/**
 * @brief   Structure describing a memory heap.
 */
struct sc_memory_heap {
 // memgetfunc2_t         provider;   /**< @brief Memory blocks provider for
  //                                              this heap.                  */
  sc_heap_header_t         header;     /**< @brief Free blocks list header.    */
 // semaphore_t           sem;        /**< @brief Heap access semaphore.      */
};

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Allocation of an aligned static heap buffer.
 */
#define SC_CH_HEAP_AREA(name, size)                                            \
  ALIGNED_VAR(SC_CH_HEAP_ALIGNMENT)                                            \
  uint8_t name[SC_MEM_ALIGN_NEXT((size), SC_CH_HEAP_ALIGNMENT)]

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void scChHeapObjectInit(sc_memory_heap_t *heapp, void *buf, size_t size);
  void *scChHeapAllocAligned(sc_memory_heap_t *heapp, size_t size, unsigned align);
  void scChHeapFree(void *p);
  size_t scChHeapStatus(sc_memory_heap_t *heapp, size_t *totalp, size_t *largestp);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Allocates a block of memory from the heap by using the first-fit
 *          algorithm.
 * @details The allocated block is guaranteed to be properly aligned for a
 *          pointer data type.
 *
 * @param[in] heapp     pointer to a heap descriptor or @p NULL in order to
 *                      access the default heap.
 * @param[in] size      the size of the block to be allocated. Note that the
 *                      allocated block may be a bit bigger than the requested
 *                      size for alignment and fragmentation reasons.
 * @return              A pointer to the allocated block.
 * @retval NULL         if the block cannot be allocated.
 *
 * @api
 */
static inline void *scChHeapAlloc(sc_memory_heap_t *heapp, size_t size) {

  return scChHeapAllocAligned(heapp, size, SC_CH_HEAP_ALIGNMENT);
}

/**
 * @brief   Returns the size of an allocated block.
 * @note    The returned value is the requested size, the real size is the
 *          same value aligned to the next @p SC_CH_HEAP_ALIGNMENT multiple.
 *
 * @param[in] p         pointer to the memory block
 * @return              Size of the block.
 *
 * @api
 */
static inline size_t scChHeapGetSize(const void *p) {

  return ((sc_heap_header_t *)p - 1U)->used.size;
}

#endif /* AP_SCRIPTING_CHMEMHEAPS_H */

/** @} */
