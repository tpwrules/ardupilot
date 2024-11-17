/*
allocator based on system malloc/free.

overhead of sizeof(void*) bytes per allocation. should be compatible with
valgrind. locked so not the best for performance.
*/

#pragma once

#include "AP_Scripting/AP_Scripting_config.h"

#if AP_SCRIPTING_ENABLED

#include "allocator_base.h"

class AP_Scripting_SystemAllocator : AP_Scripting_Allocator
{
public:
    static_assert(sizeof(void*) == sizeof(size_t), "what");

    AP_Scripting_SystemAllocator() : free_capacity(0) {};

    // create a heap with a given total nominal allocation capacity.
    // returns true if succeeded.
    bool create(uint32_t capacity) {
        free_capacity = capacity;

        return capacity != 0;
    }

    // destroy the heap. the heap does not need to be empty, though you best not
    // have a pointer into it!
    void destroy(void) {
        free_capacity = 0;
    }

    // return true if the heap is allocated and available for operatons
    bool available(void) const {
        return free_capacity != 0;
    }

    // allocate a block of a particular size, aligned to void*. behavior with
    // size of 0 is unspecified.
    void *allocate(uint32_t size);

    // de-allocate a previously allocated block. nullptr is ok to deallocate.
    void deallocate(void *ptr);

    // change allocated size of a pointer - this requires the old size, unlike
    // realloc(). not allowed to fail if new_size <= old_size. ignores old_size
    // if ptr == nullptr
    void *change_size(void *ptr, uint32_t old_size, uint32_t new_size);

private:
    uint32_t free_capacity;
};

#endif // AP_SCRIPTING_ENABLED
