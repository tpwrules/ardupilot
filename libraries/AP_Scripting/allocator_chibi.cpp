#include "allocator_chibi.h"

#if AP_SCRIPTING_ENABLED

// change allocated size of a pointer - this requires the old size, unlike
// realloc(). not allowed to fail if new_size <= old_size. ignores old_size
// if ptr == nullptr
void *AP_Scripting_ChibiAllocator::change_size(void *ptr, uint32_t old_size, uint32_t new_size) {
    if (ptr == nullptr) {
        return allocate(new_size);
    } else if (new_size == 0) {
        deallocate(ptr);
        return nullptr;
    } else {
        void* new_ptr = allocate(new_size);
        if (new_ptr == nullptr) {
            if (new_size > old_size) {
                return nullptr;
            } else {
                return ptr;
            }
        }

        memcpy(new_ptr, ptr, new_size < old_size ? new_size : old_size);

        deallocate(ptr);

        return new_ptr;
    }
}

#endif // AP_SCRIPTING_ENABLED
