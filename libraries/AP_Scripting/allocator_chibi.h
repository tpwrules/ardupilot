/*
allocator based on chibios.

no locking, isolated from system heap for fragmentation etc purposes.
*/

#pragma once

#include "AP_Scripting/AP_Scripting_config.h"

#if AP_SCRIPTING_ENABLED

#include "allocator_base.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <stdio.h>
#endif

#include "allocator_chibi_chmemheaps.h"

class AP_Scripting_ChibiAllocator : AP_Scripting_Allocator
{
public:
    static_assert(sizeof(void*) == sizeof(uintptr_t), "what");
    AP_Scripting_ChibiAllocator() : heap(nullptr), num_heaps(0) {};

    // create a heap with a given total nominal allocation capacity.
    // returns true if succeeded.
    bool create(uint32_t capacity) {
        while (capacity) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            uint32_t amt = capacity > 4096 ? 4096 : capacity; // exercise
#else
            uint32_t amt = capacity;
#endif
            while (amt > 0) {
                if (add_arena(amt)) {
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
        sc_memory_heap_t **arena = heap;
        while (arena != nullptr) {
            void *next = *arena;
            free(arena);
            arena = (sc_memory_heap_t**)next;
        }
        heap = nullptr;
        num_heaps = 0;
    }

    // return true if the heap is allocated and available for operatons
    bool available(void) const {
        return heap != nullptr;
    }

    // allocate a block of a particular size, aligned to void*. behavior with
    // size of 0 is unspecified.
    void *allocate(uint32_t size) {
        sc_memory_heap_t **arena = heap;
        while (arena != nullptr) {
            sc_memory_heap_t *subheap = (sc_memory_heap_t *)(void*)((char*)arena + sizeof(sc_memory_heap_t*));
            void *p = scChHeapAlloc(subheap, size);
            if (p != nullptr) {
                return p;
            }
            arena = (sc_memory_heap_t **)*arena;
        }
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        printf("OH OH! heaps are FULL!\n");
#endif
        if (add_arena(8*size)) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            printf("but we saved it\n");
#endif
            return allocate(size);
        }
        return nullptr;
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
    bool add_arena(uint32_t capacity) {
        if (num_heaps == 255) {
            return false;
        }

        sc_memory_heap_t **arena = (sc_memory_heap_t **)malloc(sizeof(sc_memory_heap_t*) +
            sizeof(sc_memory_heap_t) + capacity);
        if (arena == nullptr) {
            return false;
        }

        sc_memory_heap_t *subheap = (sc_memory_heap_t *)(void*)((char*)arena + sizeof(sc_memory_heap_t*));
        scChHeapObjectInit(subheap, subheap + 1U, capacity, num_heaps++);

        *arena = nullptr;
        sc_memory_heap_t ***slot = &heap; // sorry
        while (*slot != nullptr) {
            slot = (sc_memory_heap_t ***)*slot;
        }
        *slot = arena;

        return true;
    }

    sc_memory_heap_t **heap;

    uint8_t num_heaps;
};

#endif // AP_SCRIPTING_ENABLED
