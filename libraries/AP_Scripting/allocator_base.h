/*
heap support specifically for Lua (and possibly other scripting languages). note
that as scripts only run in one thread, this is NOT expected to be locked.
*/

#pragma once

#include "AP_Scripting/AP_Scripting_config.h"

#if AP_SCRIPTING_ENABLED

class AP_Scripting_Allocator
{
public:
    // create a heap with a given total nominal allocation capacity.
    // returns true if succeeded.
    bool create(uint32_t capacity);

    // destroy the heap. the heap does not need to be empty, though you best not
    // have a pointer into it!
    void destroy(void);

    // return true if the heap is allocated and available for operatons
    bool available(void) const;

    // allocate a block of a particular size, aligned to void*. behavior with
    // size of 0 is unspecified.
    void *allocate(uint32_t size);

    // de-allocate a previously allocated block. nullptr is ok to deallocate.
    void deallocate(void *ptr);

    // change allocated size of a pointer - this requires the old size, unlike
    // realloc(). not allowed to fail if new_size <= old_size. ignores old_size
    // if ptr == nullptr
    void *change_size(void *ptr, uint32_t old_size, uint32_t new_size);
};

#endif // AP_SCRIPTING_ENABLED
