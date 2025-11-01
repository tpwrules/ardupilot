-- flight.lua
-- A CRSF menu for in-flight adjustments and utility functions.
-- Version 1.4: Added Save and Revert functionality.

local crsf_helper = require('crsf_helper')

-- MAVLink severity for GCS messages
local MAV_SEVERITY = {INFO = 6, WARNING = 4, ERROR = 3}
local CRSF_COMMAND_STATUS = crsf_helper.CRSF_COMMAND_STATUS

-- ####################
-- # PARAMETER BINDING
-- ####################

-- Acro Mode Parameter
-- Bit 1 (value 2) toggles "Rate-Only" (rate loop only) mode.
local acro_options_param = assert(Parameter('ACRO_OPTIONS'), "Failed to find ACRO_OPTIONS")

-- Define the bitmask for the rate-only option
local RATE_ONLY_BIT = 2 -- This is bit 1, which has a value of 2

-- Environment Mode Parameters
local ek3_rng_use_hgt = assert(Parameter('EK3_RNG_USE_HGT'), "Failed to find EK3_RNG_USE_HGT")
local ek3_src_options = assert(Parameter('EK3_SRC_OPTIONS'), "Failed to find EK3_SRC_OPTIONS")
local ek3_src1_posxy = assert(Parameter('EK3_SRC1_POSXY'), "Failed to find EK3_SRC1_POSXY")
local ek3_src1_velxy = assert(Parameter('EK3_SRC1_VELXY'), "Failed to find EK3_SRC1_VELXY")
local ek3_src1_velz = assert(Parameter('EK3_SRC1_VELZ'), "Failed to find EK3_SRC1_VELZ")

-- ####################
-- # STATE MANAGEMENT
-- ####################

-- A list of all parameters this script manages
local managed_params = {
    acro_options_param,
    ek3_rng_use_hgt,
    ek3_src_options,
    ek3_src1_posxy,
    ek3_src1_velxy,
    ek3_src1_velz,
}

-- Table to store the original parameter values on startup (or after save)
local original_values = {}

--- Populate the original value table for all managed parameters.
local function populate_original_values()
    original_values = {} -- Clear existing values
    for _, param in ipairs(managed_params) do
        table.insert(original_values, {
            param = param,
            original_value = param:get()
        })
    end
end

-- ####################
-- # PARAMETER TABLES
-- ####################

local outdoor_params = {
    {param = ek3_rng_use_hgt, value = -1},
    {param = ek3_src_options, value = 1},
    {param = ek3_src1_posxy,  value = 3},
    {param = ek3_src1_velxy,  value = 3},
    {param = ek3_src1_velz,   value = 3},
}

local indoor_params = {
    {param = ek3_rng_use_hgt, value = 3},
    {param = ek3_src_options, value = 0},
    {param = ek3_src1_posxy,  value = 0},
    {param = ek3_src1_velxy,  value = 5},
    {param = ek3_src1_velz,   value = 0},
}

-- ####################
-- # FORWARD DECLARATIONS
-- ####################
-- These hold the menu item definitions so we can update them
local acro_mode_item, env_mode_item

-- ####################
-- # MENU INITIALIZATION & SYNC
-- ####################

--- Sets the initial default value for the acro mode menu.
local function get_default_acro_mode_idx()
    local current_val = acro_options_param:get()
    -- Check if the rate-only bit (bit 1) is set
    if (current_val & RATE_ONLY_BIT) ~= 0 then
        return 1 -- "Rate"
    else
        return 2 -- "Angle"
    end
end

--- Sets the initial default value for the env mode menu.
local function get_default_env_mode_idx()
    -- Check EK3_SRC_OPTIONS as the key indicator
    -- 1 = Outdoor (GPS)
    if ek3_src_options:get() == 1 then
        return 1 -- "Outdoor"
    else
        return 2 -- "Indoor"
    end
end

--- Synchronizes the menu's displayed selection with the underlying param values.
-- Used after a revert to make the menu show the correct state.
local function sync_menu_selections_to_params()
    if acro_mode_item then
        acro_mode_item.current_idx = get_default_acro_mode_idx()
    end
    if env_mode_item then
        env_mode_item.current_idx = get_default_env_mode_idx()
    end
end

-- ####################
-- # CORE COMMANDS
-- ####################

--- Saves all currently set values to EEPROM and resets the tuning state.
local function save_all_settings()
    for _, param in ipairs(managed_params) do
        local current_value = param:get()
        param:set_and_save(current_value)
    end
    gcs:send_text(MAV_SEVERITY.INFO, "Flight settings saved.")

    -- After saving, the new values become the baseline for further tuning.
    populate_original_values()
    gcs:send_text(MAV_SEVERITY.INFO, "Tuning baseline reset to new values.")
end

--- Reverts all settings to the values they had when the script was started.
local function revert_all_settings()
    for _, item in ipairs(original_values) do
        item.param:set(item.original_value)
    end
    gcs:send_text(MAV_SEVERITY.INFO, "Flight settings reverted to startup values.")

    -- After reverting the underlying params, sync the menu display to match
    sync_menu_selections_to_params()
end

-- ####################
-- # CALLBACK FUNCTIONS
-- ####################

--- Callback for the "Acro Mode" selection.
-- @param selection (string): The new mode selected ("Rate" or "Angle").
local function on_acro_mode_change(selection)
    local current_val = acro_options_param:get()
    local new_val
    
    if selection == "Rate" then
        -- Set the rate-only bit (bit 1)
        new_val = current_val | RATE_ONLY_BIT
        gcs:send_text(MAV_SEVERITY.INFO, "Acro mode set to Rate-Only.")
    elseif selection == "Angle" then
        -- Clear the rate-only bit (bit 1)
        new_val = current_val & (~RATE_ONLY_BIT)
        gcs:send_text(MAV_SEVERITY.INFO, "Acro mode set to Angle.")
    end

    if new_val ~= nil and new_val ~= current_val then
        acro_options_param:set(new_val)
    end
end

--- Sets a batch of parameters from a table
local function set_parameters(param_table)
    for _, item in ipairs(param_table) do
        item.param:set(item.value)
    end
end

--- Callback for the "Environment" selection.
-- @param selection (string): The new mode selected ("Outdoor" or "Indoor").
local function on_env_mode_change(selection)
    if selection == "Outdoor" then
        set_parameters(outdoor_params)
        gcs:send_text(MAV_SEVERITY.INFO, "EKF set for Outdoor flight.")
    elseif selection == "Indoor" then
        set_parameters(indoor_params)
        gcs:send_text(MAV_SEVERITY.INFO, "EKF set for Indoor flight.")
    end
end

-- ####################
-- # MENU DEFINITION
-- ####################

-- Define the menu items and store them in local variables
acro_mode_item = {
    type = 'SELECTION',
    name = "Acro Mode",
    options = {"Rate", "Angle"},
    default = get_default_acro_mode_idx(),
    callback = on_acro_mode_change
}

env_mode_item = {
    type = 'SELECTION',
    name = "Environment",
    options = {"Outdoor", "Indoor"},
    default = get_default_env_mode_idx(),
    callback = on_env_mode_change
}

local menu_definition = {
    name = "Flight", -- The root menu name
    items = {
        acro_mode_item,
        env_mode_item,

        -- ====== SAVE & REVERT COMMANDS ======
        {
            type = 'COMMAND',
            name = "Save Settings",
            info = "Confirm Save (Permanent)?",
            callback = function(command_action)
                if command_action == CRSF_COMMAND_STATUS.START then
                    save_all_settings()
                    return CRSF_COMMAND_STATUS.READY, "Saved!"
                end
                -- Default state text
                return CRSF_COMMAND_STATUS.READY, "Confirm Save (Permanent)?"
            end
        },
        {
            type = 'COMMAND',
            name = "Revert Settings",
            info = "Confirm Revert?",
            callback = function(command_action)
                if command_action == CRSF_COMMAND_STATUS.START then
                    revert_all_settings()
                    return CRSF_COMMAND_STATUS.READY, "Reverted!"
                end
                -- Default state text
                return CRSF_COMMAND_STATUS.READY, "Confirm Revert?"
            end
        },
    }
}

-- ####################
-- # REGISTRATION
-- ####################

-- 1. Populate the baseline values on script start
populate_original_values()

-- 2. Register this script's menu definition with the helper.
return crsf_helper.register_menu(menu_definition)
