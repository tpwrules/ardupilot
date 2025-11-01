-- flight.lua
-- A CRSF menu for in-flight adjustments and utility functions.
-- Version 1.1: Correctly uses ACRO_OPTIONS bit 1 for Rate/Angle toggle.

local crsf_helper = require('crsf_helper')

-- MAVLink severity for GCS messages
local MAV_SEVERITY = {INFO = 6, WARNING = 4, ERROR = 3}
local CRSF_COMMAND_STATUS = crsf_helper.CRSF_COMMAND_STATUS

-- ####################
-- # PARAMETER BINDING
-- ####################

-- Bind the ACRO_OPTIONS parameter which controls Acro mode behavior
-- Bit 1 (value 2) toggles "Rate-Only" (rate loop only) mode.
local acro_options_param = assert(Parameter('ACRO_OPTIONS'), "Failed to find ACRO_OPTIONS")

-- Define the bitmask for the rate-only option
local RATE_ONLY_BIT = 2 -- This is bit 1, which has a value of 2

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
        gcs:send_text(MAV_SEVERITY.INFO, "Acro set to Rate-Only (ACRO_OPTIONS bit 1 SET)")
    elseif selection == "Angle" then
        -- Clear the rate-only bit (bit 1)
        new_val = current_val & (~RATE_ONLY_BIT)
        gcs:send_text(MAV_SEVERITY.INFO, "Acro set to Angle (ACRO_OPTIONS bit 1 CLEARED)")
    end

    if new_val ~= current_val then
        acro_options_param:set(new_val)
    end
end

-- ####################
-- # MENU INITIALIZATION
-- ####################

--- Sets the initial default value for the menu based on the current param value.
local function get_default_acro_mode_idx()
    local current_val = acro_options_param:get()
    -- Check if the rate-only bit (bit 1) is set
    if (current_val & RATE_ONLY_BIT) ~= 0 then
        return 1 -- "Rate"
    else
        return 2 -- "Angle"
    end
end

-- ####################
-- # MENU DEFINITION
-- ####################

local menu_definition = {
    name = "Flight", -- The root menu name
    items = {
        -- ====== Acro Mode Toggle ======
        {
            type = 'SELECTION',
            name = "Acro Mode",
            options = {"Rate", "Angle"},
            default = get_default_acro_mode_idx(),
            callback = on_acro_mode_change
        },
    }
}

-- ####################
-- # REGISTRATION
-- ####################

-- 1. Register this script's menu definition with the helper.
return crsf_helper.register_menu(menu_definition)