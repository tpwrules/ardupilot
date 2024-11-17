#pragma once

#include "AP_Scripting/AP_Scripting_config.h"

#if AP_SCRIPTING_ENABLED

#include "allocator_multiheap.h"
#include "allocator_system.h"

typedef AP_Scripting_SystemAllocator AP_Scripting_CurrentAllocator;

#endif // AP_SCRIPTING_ENABLED
