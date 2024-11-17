/*
allocator based on tlsf.

more internal fragmentation but better performance.
*/

#pragma once

#include "AP_Scripting/AP_Scripting_config.h"

#if AP_SCRIPTING_ENABLED

#include "allocator_base.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <stdio.h>
#endif

#include "allocator_tlsf_tlsf.h"

class AP_Scripting_TLSFAllocator : AP_Scripting_Allocator
{
public:
    AP_Scripting_TLSFAllocator() : heap(nullptr), pools(nullptr) {};

    // create a heap with a given total nominal allocation capacity.
    // returns true if succeeded.
    bool create(uint32_t capacity) {
        size_t control_size = sc_tlsf_size();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        printf("control size is %lu\n", control_size);
#endif

        if (capacity < control_size) {
            return false; // not enough even for control structures
        }

        capacity -= control_size;
        void *p = malloc(control_size);
        if (p == nullptr) {
            return false;
        }
        heap = sc_tlsf_create(p);

        while (capacity) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            uint32_t amt = capacity > 4096 ? 4096 : capacity; // exercise
#else
            uint32_t amt = capacity;
#endif
            while (amt > 0) {
                if (add_pool(amt)) {
                    capacity -= amt;
                    break;
                }
                amt *= 0.9;
            }
            if (capacity == 0)
                return true;
            if (capacity < 256)
                return false;
        }
        return false;
    }

    // destroy the heap. the heap does not need to be empty, though you best not
    // have a pointer into it!
    void destroy(void) {
        // sc_tlsf_destroy(heap); // actually a no-op
        free(heap);
        heap = nullptr;

        void **pool = pools;
        while (pool != nullptr) {
            void **next = (void**)*pool;
            free(pool);
            pool = next;
        }
        pools = nullptr;
    }

    // return true if the heap is allocated and available for operatons
    bool available(void) const {
        return heap != nullptr;
    }

    // allocate a block of a particular size, aligned to void*. behavior with
    // size of 0 is unspecified.
    void *allocate(uint32_t size) {
        // automatically chooses a pool
        void *p = sc_tlsf_malloc(heap, size);
        if (p != nullptr) {
            return p;
        }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        printf("OH OH! heaps are FULL!\n");
#endif
        if (add_pool(8*size)) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            printf("but we saved it\n");
#endif
            return allocate(size);
        }
        return nullptr;
    }

    // de-allocate a previously allocated block. nullptr is ok to deallocate.
    void deallocate(void *ptr) {
        sc_tlsf_free(heap, ptr); // can handle nullptr
    }

    // change allocated size of a pointer - this requires the old size, unlike
    // realloc(). not allowed to fail if new_size <= old_size. ignores old_size
    // if ptr == nullptr
    void *change_size(void *ptr, uint32_t old_size, uint32_t new_size) {
        (void)old_size;
        // behaves exactly like Lua needs, can't fail to shrink
        void *new_ptr = sc_tlsf_realloc(heap, ptr, new_size);
        if (new_size && new_ptr == nullptr) {
            if (add_pool(8*new_size)) {
                return sc_tlsf_realloc(heap, ptr, new_size);
            }
            return nullptr;
        }
        return new_ptr;
    }

private:
    bool add_pool(uint32_t capacity) {
        void **pool = (void **)malloc(sizeof(void*) + capacity);
        if (pool == nullptr) {
            return false;
        }

        sc_tlsf_add_pool(heap, pool + 1U, capacity);

        *pool = pools; // add to pool list head
        pools = pool;

        return true;
    }

    sc_tlsf_t heap;
    void** pools;
};

#endif // AP_SCRIPTING_ENABLED
