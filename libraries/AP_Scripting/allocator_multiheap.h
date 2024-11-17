/*
allocator based on the existing MultiHeap. might not handle invariants.
*/

#pragma once

#include "AP_Scripting/AP_Scripting_config.h"

#if AP_SCRIPTING_ENABLED

#include "allocator_base.h"

#include <AP_Common/MultiHeap.h>

/*
  on ChibiOS allow up to 4 heaps. On other HALs only allow a single
  heap. This is needed as hal.util->heap_realloc() needs to have the
  property that heap_realloc(heap, ptr, 0) must not care if ptr comes
  from the given heap. This is true on ChibiOS, but is not true on
  other HALs
 */
#ifndef MAX_HEAPS
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#define MAX_HEAPS 4
#else
#define MAX_HEAPS 1
#endif
#endif

class AP_Scripting_MultiHeapAllocator : AP_Scripting_Allocator
{
public:
    // create a heap with a given total nominal allocation capacity.
    // returns true if succeeded.
    bool create(uint32_t capacity) {
        return _heap.create(capacity, MAX_HEAPS);
    }

    // destroy the heap. the heap does not need to be empty, though you best not
    // have a pointer into it!
    void destroy(void) {
        _heap.destroy();
    }

    // return true if the heap is allocated and available for operatons
    bool available(void) const {
        return _heap.available();
    }

    // allocate a block of a particular size, aligned to void*. behavior with
    // size of 0 is unspecified.
    void *allocate(uint32_t size) {
        return _heap.allocate(size);
    }

    // de-allocate a previously allocated block. nullptr is ok to deallocate.
    void deallocate(void *ptr) {
        _heap.deallocate(ptr);
    }

    // change allocated size of a pointer - this requires the old size, unlike
    // realloc(). not allowed to fail if new_size <= old_size. ignores old_size
    // if ptr == nullptr
    void *change_size(void *ptr, uint32_t old_size, uint32_t new_size) {
        return _heap.change_size(ptr, old_size, new_size);
    }

private:
    MultiHeap _heap;
};

#endif // AP_SCRIPTING_ENABLED
