#pragma once

#include "AP_Scripting/AP_Scripting_config.h"

#if AP_SCRIPTING_ENABLED

#include "allocator_system.h"
#include "allocator_chibi.h"

typedef AP_Scripting_ChibiAllocator AP_Scripting_CurrentAllocator;

#endif // AP_SCRIPTING_ENABLED
