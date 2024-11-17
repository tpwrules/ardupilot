#pragma once

#include "AP_Scripting/AP_Scripting_config.h"

#if AP_SCRIPTING_ENABLED

#include "allocator_multiheap.h"

typedef AP_Scripting_MultiHeapAllocator AP_Scripting_CurrentAllocator;

#endif // AP_SCRIPTING_ENABLED
