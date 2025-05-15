#include "AP_Scripting_config.h"

#if AP_SCRIPTING_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Filesystem/AP_Filesystem.h>

#include "evil_flash_access.h"

#include "lua/src/lauxlib.h"

#include <AP_Scripting/AP_Scripting.h>
#include <AP_Scripting/lua_generated_bindings.h>

#include <AP_HAL_ChibiOS/hwdef/common/flash.h>

int efa_read32(lua_State *L) {
    binding_argcheck(L, 1);

    const uint32_t addr = (uint32_t)luaL_checkinteger(L, 1);

    __DSB();
    __ISB();

    const uint32_t val = *((volatile uint32_t*)addr);

    lua_pushinteger(L, (uint32_t)val);

    return 1;
}

int efa_write32(lua_State *L) {
    binding_argcheck(L, 2);

    const uint32_t addr = (uint32_t)luaL_checkinteger(L, 1);
    const uint32_t val = (uint32_t)luaL_checkinteger(L, 2);

    *((volatile uint32_t*)addr) = val;

    __DSB();
    __ISB();

    return 0;
}

int efa_erase(lua_State *L) {
    binding_argcheck(L, 1);

    const uint8_t page = (uint8_t)luaL_checkinteger(L, 1);

    const bool success = stm32_flash_erasepage(page);

    lua_pushboolean(L, success);

    return 1;
}

int efa_erased(lua_State *L) {
    binding_argcheck(L, 1);

    const uint8_t page = (uint8_t)luaL_checkinteger(L, 1);

    const bool success = stm32_flash_ispageerased(page);

    lua_pushboolean(L, success);

    return 1;
}

int efa_program(lua_State *L) {
    binding_argcheck(L, 2);

    const uint32_t addr = (uint32_t)luaL_checkinteger(L, 1);

    size_t len;
    const char* data = luaL_checklstring(L, 2, &len);

    bool success = stm32_flash_write_h7_cowboy(addr, data, len, false);

    lua_pushboolean(L, success);

    return 1;
}

int efa_programv(lua_State *L) {
    binding_argcheck(L, 2);

    const uint32_t addr = (uint32_t)luaL_checkinteger(L, 1);

    size_t len;
    const char* data = luaL_checklstring(L, 2, &len);

    bool success = stm32_flash_write_h7_cowboy(addr, data, len, true);

    lua_pushboolean(L, success);

    return 1;
}

#endif  // AP_SCRIPTING_ENABLED
