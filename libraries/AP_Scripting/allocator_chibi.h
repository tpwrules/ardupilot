/*
allocator based on chibios.

no locking, isolated from system heap for fragmentation etc purposes.
*/

#pragma once

#include "AP_Scripting/AP_Scripting_config.h"

#if AP_SCRIPTING_ENABLED

#include "allocator_base.h"

#include "allocator_chibi_chmemheaps.h"

class AP_Scripting_ChibiAllocator : AP_Scripting_Allocator
{
public:
    AP_Scripting_ChibiAllocator() : arena(nullptr) {};

    // create a heap with a given total nominal allocation capacity.
    // returns true if succeeded.
    bool create(uint32_t capacity) {
        arena = (sc_memory_heap_t *)malloc(capacity + sizeof(sc_memory_heap_t));
        if (arena == nullptr) {
            return false;
        }
        scChHeapObjectInit(arena, arena + 1U, capacity);
        return true;
    }

    // destroy the heap. the heap does not need to be empty, though you best not
    // have a pointer into it!
    void destroy(void) {
        free(arena);
        arena = nullptr;
    }

    // return true if the heap is allocated and available for operatons
    bool available(void) const {
        return arena != nullptr;
    }

    // allocate a block of a particular size, aligned to void*. behavior with
    // size of 0 is unspecified.
    void *allocate(uint32_t size) {
        return scChHeapAlloc(arena, size);
    }

    // de-allocate a previously allocated block. nullptr is ok to deallocate.
    void deallocate(void *ptr) {
        if (ptr != nullptr) {
            scChHeapFree(ptr); // cannot handle nullptr
        }
    }

    // change allocated size of a pointer - this requires the old size, unlike
    // realloc(). not allowed to fail if new_size <= old_size. ignores old_size
    // if ptr == nullptr
    void *change_size(void *ptr, uint32_t old_size, uint32_t new_size);

private:
    sc_memory_heap_t *arena;
};

#endif // AP_SCRIPTING_ENABLED
