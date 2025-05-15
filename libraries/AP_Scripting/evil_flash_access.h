#pragma once

#include "lua/src/lua.hpp"

int efa_read32(lua_State *L);
int efa_write32(lua_State *L);
int efa_erase(lua_State *L);
int efa_erased(lua_State *L);
int efa_program(lua_State *L);
int efa_programv(lua_State *L);
