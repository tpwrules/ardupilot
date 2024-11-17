#include "allocator_system.h"

#if AP_SCRIPTING_ENABLED

// allocate a block of a particular size, aligned to void*. behavior with
// size of 0 is unspecified.
void *AP_Scripting_SystemAllocator::allocate(uint32_t size) {
    size += sizeof(size_t); // should probably check for overflow
    if (free_capacity <= size) { // can't let free_capacity become 0
        return nullptr;
    }

    void* pp = malloc(size);
    if (pp == nullptr) {
        return nullptr;
    }

    *(size_t*)pp = size;
    free_capacity -= size;

    void* ptr = (void*)((size_t*)pp + 1U);
    return ptr;
}

// de-allocate a previously allocated block. nullptr is ok to deallocate.
void AP_Scripting_SystemAllocator::deallocate(void *ptr) {
    if (ptr == nullptr) {
        return;
    }

    void* pp = (void*)((size_t*)ptr - 1U);
    free_capacity += *(size_t*)pp;

    free(pp);
}

// change allocated size of a pointer - this requires the old size, unlike
// realloc(). not allowed to fail if new_size <= old_size. ignores old_size
// if ptr == nullptr
void *AP_Scripting_SystemAllocator::change_size(void *ptr, uint32_t old_size, uint32_t new_size) {
    if (ptr == nullptr) {
        return allocate(new_size);
    } else if (new_size == 0) {
        deallocate(ptr);
        return nullptr;
    } else {
        void* pp = (void*)((size_t*)ptr - 1U);
        old_size += sizeof(size_t);
        new_size += sizeof(size_t);

        old_size = *(size_t*)pp; // wish to not need it

        uint32_t delta = new_size - old_size;
        // can't let free_capacity become 0
        if (new_size > old_size && free_capacity <= delta) {
            return nullptr;
        }

        pp = realloc(pp, new_size); // may fail if new_size <= old_size :(
        if (pp == nullptr) {
            return nullptr;
        }

        free_capacity -= delta;
        *(size_t*)pp += delta;

        ptr = (void*)((size_t*)pp + 1U);
        return ptr;
    }
}

#endif // AP_SCRIPTING_ENABLED
