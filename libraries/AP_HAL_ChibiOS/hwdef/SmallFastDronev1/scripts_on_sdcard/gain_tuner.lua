-- gain_tuner.lua
-- A script to provide a CRSF menu for in-flight PID gain tuning and utility functions.
-- Version 3.2: Removed command confirmation step, customized execution text.
-- Allows for setting key PID gains for an entire axis at once.
-- Includes Save/Revert, stateful percentage-based adjustments, and utility functions.

local crsf_helper = require('crsf_helper')

-- MAVLink severity for GCS messages
local MAV_SEVERITY = {INFO = 6, WARNING = 4, ERROR = 3}
local CRSF_COMMAND_STATUS = crsf_helper.CRSF_COMMAND_STATUS

local TUNING_UNIT = "%"

-- State variable to hold the currently selected gains to tune.
local gains_to_tune_selection = "All" -- Default to "All"

-- ####################
-- # PARAMETER SETUP
-- ####################

-- Group parameters by axis into tables for easier handling.
-- Added a 'type' field to each gain for easy filtering.
local axis_gains = {
    Roll = {
        {param = assert(Parameter('ATC_RAT_RLL_P'), "Failed to find ATC_RAT_RLL_P"), name = "Roll P", type = "P"},
        {param = assert(Parameter('ATC_RAT_RLL_I'), "Failed to find ATC_RAT_RLL_I"), name = "Roll I", type = "I"},
        {param = assert(Parameter('ATC_RAT_RLL_D'), "Failed to find ATC_RAT_RLL_D"), name = "Roll D", type = "D"},
        {param = assert(Parameter('ATC_ANG_RLL_P'), "Failed to find ATC_ANG_RLL_P"), name = "Roll Ang P", type = "A"}
    },
    Pitch = {
        {param = assert(Parameter('ATC_RAT_PIT_P'), "Failed to find ATC_RAT_PIT_P"), name = "Pitch P", type = "P"},
        {param = assert(Parameter('ATC_RAT_PIT_I'), "Failed to find ATC_RAT_PIT_I"), name = "Pitch I", type = "I"},
        {param = assert(Parameter('ATC_RAT_PIT_D'), "Failed to find ATC_RAT_PIT_D"), name = "Pitch D", type = "D"},
        {param = assert(Parameter('ATC_ANG_PIT_P'), "Failed to find ATC_ANG_PIT_P"), name = "Pitch Ang P", type = "A"}
    },
    Yaw = {
        {param = assert(Parameter('ATC_RAT_YAW_P'), "Failed to find ATC_RAT_YAW_P"), name = "Yaw P", type = "P"},
        {param = assert(Parameter('ATC_RAT_YAW_I'), "Failed to find ATC_RAT_YAW_I"), name = "Yaw I", type = "I"},
        {param = assert(Parameter('ATC_RAT_YAW_D'), "Failed to find ATC_RAT_YAW_D"), name = "Yaw D", type = "D"},
        {param = assert(Parameter('ATC_ANG_YAW_P'), "Failed to find ATC_ANG_YAW_P"), name = "Yaw Ang P", type = "A"}
    }
}

-- Table of all other utility parameters managed by this script
local utility_params = {
    -- Batch Logging
    {param = assert(Parameter('INS_LOG_BAT_CNT'), "Failed to find INS_LOG_BAT_CNT")},
    {param = assert(Parameter('INS_LOG_BAT_LGCT'), "Failed to find INS_LOG_BAT_LGCT")},
    {param = assert(Parameter('INS_LOG_BAT_LGIN'), "Failed to find INS_LOG_BAT_LGIN")},
    {param = assert(Parameter('INS_LOG_BAT_MASK'), "Failed to find INS_LOG_BAT_MASK")},
    {param = assert(Parameter('INS_LOG_BAT_OPT'), "Failed to find INS_LOG_BAT_OPT")},
    -- Fast Attitude Logging
    {param = assert(Parameter('LOG_BITMASK'), "Failed to find LOG_BITMASK")},
    -- Autotune
    {param = assert(Parameter('AUTOTUNE_AGGR'), "Failed to find AUTOTUNE_AGGR")},
    {param = assert(Parameter('AUTOTUNE_AXES'), "Failed to find AUTOTUNE_AXES")},
    {param = assert(Parameter('AUTOTUNE_MIN_D'), "Failed to find AUTOTUNE_MIN_D")},
    {param = assert(Parameter('AUTOTUNE_GMBK'), "Failed to find AUTOTUNE_GMBK")}
}

-- Table to store the original parameter values on script startup (or after save) for the revert/baseline function
local original_gains = {}
local original_utility_values = {}

-- Populate the original value tables for all managed parameters.
local function populate_original_values()
    -- Populate gains
    original_gains = {}
    for axis_name, gains in pairs(axis_gains) do
        original_gains[axis_name] = {}
        for _, gain_info in ipairs(gains) do
            table.insert(original_gains[axis_name], {
                param = gain_info.param,
                original_value = gain_info.param:get(),
                type = gain_info.type -- Carry over the type for filtering
            })
        end
    end

    -- Populate utility params
    original_utility_values = {}
    for _, param_info in ipairs(utility_params) do
        table.insert(original_utility_values, {
            param = param_info.param,
            original_value = param_info.param:get()
        })
    end
end

-- Initial population of original values at script start
populate_original_values()

-- ####################
-- # FORWARD DECLARATIONS FOR MENU ITEMS
-- ####################
-- These need to be declared here so they can be accessed by the callback functions
local roll_gain_item, pitch_gain_item, yaw_gain_item
local default_gain_idx = 5 -- 1-based index for "Hold"

-- Forward declaration for the sync function to resolve scope issues
local sync_menu_selections_to_params

-- ####################
-- # CORE LOGIC & CALLBACKS
-- ####################

-- Generates a detailed GCS feedback string for the currently tuned gains of an axis.
local function get_gcs_feedback_for_axis(axis_name)
    local parts = {}
    local gains_table = axis_gains[axis_name]
    if not gains_table then return "" end

    for _, gain_info in ipairs(gains_table) do
        local should_report = false
        if gains_to_tune_selection == "All" then
            should_report = true
        elseif gains_to_tune_selection == "AngP" and gain_info.type == "A" then
            should_report = true
        elseif gains_to_tune_selection == gain_info.type then
            should_report = true
        end

        if should_report then
            local gain_val = gain_info.param:get()
            local type_label = gain_info.type
            if type_label == "A" then type_label = "AngP" end
            table.insert(parts, string.format("%s:%.4f", type_label, gain_val))
        end
    end

    if #parts > 0 then
        return axis_name .. " -> " .. table.concat(parts, ", ")
    end
    return ""
end


-- This function is called when a Roll/Pitch/Yaw Gain selection is changed.
-- It applies the selected percentage multiplier to the *original* baseline gains,
-- but only for the gain types selected in the Settings menu.
-- @param axis_name (string): The name of the axis to adjust ("Roll", "Pitch", or "Yaw").
-- @param selection (string): The percentage string (e.g., "+25%", "Hold", "-10%").
local function on_axis_gain_change(axis_name, selection)
    local percent_num = 0
    if selection ~= "Hold" then
        -- Strip the unit from the selection string before converting to a number
        -- The '%' in the pattern must be escaped with another '%' for gsub
        local selection_numeric_part = string.gsub(selection, "%%", "")
        percent_num = tonumber(selection_numeric_part) or 0
    end

    -- Calculate the total multiplier (e.g., "Hold" = 1.0, "+25%" = 1.25, "-10%" = 0.90)
    local total_multiplier = 1.0 + (percent_num / 100.0)

    local original_gain_table = original_gains[axis_name]
    if not original_gain_table then
        gcs:send_text(MAV_SEVERITY.WARNING, "Invalid axis: " .. tostring(axis_name))
        return
    end

    -- Iterate through the original parameters for the axis and calculate/set the new value
    for _, gain_info in ipairs(original_gain_table) do
        local should_tune = false
        if gains_to_tune_selection == "All" then
            should_tune = true
        elseif gains_to_tune_selection == "AngP" and gain_info.type == "A" then
            should_tune = true
        elseif gains_to_tune_selection == gain_info.type then
            should_tune = true
        end

        if should_tune then
            local new_value = gain_info.original_value * total_multiplier
            gain_info.param:set(new_value)
        end
    end

    -- Send a confirmation message to the GCS for the entire axis
    local gcs_message = string.format("%s Gains(%s) set to %s", axis_name, gains_to_tune_selection, selection)
    gcs:send_text(MAV_SEVERITY.INFO, gcs_message)
    
    -- Send a detailed feedback message with the new gain values
    local feedback = get_gcs_feedback_for_axis(axis_name)
    if feedback ~= "" then
        gcs:send_text(MAV_SEVERITY.INFO, feedback)
    end
end

-- Saves all currently set values to EEPROM and resets the tuning state.
local function save_all_settings()
    -- Save gains
    for _, gains in pairs(axis_gains) do
        for _, gain_info in ipairs(gains) do
            local current_value = gain_info.param:get()
            gain_info.param:set_and_save(current_value)
        end
    end

    -- Save utility params
    for _, param_info in ipairs(utility_params) do
        local current_value = param_info.param:get()
        param_info.param:set_and_save(current_value)
    end
    gcs:send_text(MAV_SEVERITY.INFO, "All tuned settings have been saved.")

    -- After saving, the new values become the baseline for further tuning.
    populate_original_values()
    gcs:send_text(MAV_SEVERITY.INFO, "Tuning baseline reset to new values.")

    -- Manually reset the menu items' internal state to the default value.
    if roll_gain_item then roll_gain_item.current_idx = default_gain_idx end
    if pitch_gain_item then pitch_gain_item.current_idx = default_gain_idx end
    if yaw_gain_item then yaw_gain_item.current_idx = default_gain_idx end
end

-- Reverts all settings to the values they had when the script was started.
local function revert_all_settings()
    -- Revert gains
    for _, gains in pairs(original_gains) do
        for _, gain_info in ipairs(gains) do
            gain_info.param:set(gain_info.original_value)
        end
    end

    -- Revert utility params
    for _, param_info in ipairs(original_utility_values) do
        param_info.param:set(param_info.original_value)
    end
    gcs:send_text(MAV_SEVERITY.INFO, "Settings reverted to startup values.")

    -- Manually reset the menu items' internal state to the default value.
    if roll_gain_item then roll_gain_item.current_idx = default_gain_idx end
    if pitch_gain_item then pitch_gain_item.current_idx = default_gain_idx end
    if yaw_gain_item then yaw_gain_item.current_idx = default_gain_idx end

    -- After reverting the underlying params, sync the menu display to match
    sync_menu_selections_to_params()
end

-- ####################
-- # COMMAND & UTILITY CALLBACKS
-- ####################

-- Sets parameters for batch logging
local function toggle_batch_logging(enable)
    if enable then
        param:set('INS_LOG_BAT_CNT', 2048)
        param:set('INS_LOG_BAT_LGCT', 32)
        param:set('INS_LOG_BAT_LGIN', 10)
        param:set('INS_LOG_BAT_MASK', 3)
        param:set('INS_LOG_BAT_OPT', 4)
        gcs:send_text(MAV_SEVERITY.INFO, "Batch logging enabled.")
    else
        param:set('INS_LOG_BAT_MASK', 0)
        gcs:send_text(MAV_SEVERITY.INFO, "Batch logging disabled.")
    end
end

-- Enables or disables fast attitude logging by modifying LOG_BITMASK
local function toggle_fast_attitude_logging(enable)
    local log_bitmask = Parameter('LOG_BITMASK')
    local current_val = log_bitmask:get()
    if enable then
        if current_val % 2 == 0 then -- It's even, so add 1 to make it odd
            log_bitmask:set(current_val + 1)
            gcs:send_text(MAV_SEVERITY.INFO, "Fast attitude logging enabled.")
        else
            gcs:send_text(MAV_SEVERITY.INFO, "Fast attitude logging already enabled.")
        end
    else
        if current_val % 2 ~= 0 then -- It's odd, so subtract 1 to make it even
            log_bitmask:set(current_val - 1)
            gcs:send_text(MAV_SEVERITY.INFO, "Fast attitude logging disabled.")
        else
            gcs:send_text(MAV_SEVERITY.INFO, "Fast attitude logging already disabled.")
        end
    end
end

-- Erases all logs from the flight controller
local function erase_logs()
    -- MAV_CMD_PREFLIGHT_STORAGE (245), param2 = 2 for erase logs
    local result = gcs:run_command_int(245, {p2=2})
    if result == 0 then -- MAV_RESULT_ACCEPTED
        gcs:send_text(MAV_SEVERITY.INFO, "Log erase command sent.")
    else
        gcs:send_text(MAV_SEVERITY.ERROR, "Log erase command failed.")
    end
end

-- Sets up parameters for an autotune session
local function setup_autotune(axes_value, name)
    param:set('AUTOTUNE_AGGR', 0.075)
    param:set('AUTOTUNE_AXES', axes_value)
    param:set('AUTOTUNE_MIN_D', 0.0003)
    gcs:send_text(MAV_SEVERITY.INFO, "Autotune configured for: " .. name)
end

-- Callback for the new Autotune Setup selection
local function on_autotune_setup_change(selection)
    if selection == "Roll/Pitch" then
        setup_autotune(3, "Roll/Pitch")
    elseif selection == "Yaw" then
        setup_autotune(4, "Yaw")
    elseif selection == "Yaw D" then
        setup_autotune(8, "Yaw D")
    elseif selection == "None" then
        param:set('AUTOTUNE_AXES', 0)
        gcs:send_text(MAV_SEVERITY.INFO, "Autotune axes disabled.")
    end
end

-- Sets the AUTOTUNE_GMBK parameter based on user selection
local function on_backoff_change(selection)
    local autotune_gmbk = Parameter('AUTOTUNE_GMBK')
    if selection == "Soft Tune" then
        autotune_gmbk:set(0.25)
        gcs:send_text(MAV_SEVERITY.INFO, "Autotune Backoff set to 0.25 (Soft).")
    elseif selection == "Firm Tune" then
        autotune_gmbk:set(0.1)
        gcs:send_text(MAV_SEVERITY.INFO, "Autotune Backoff set to 0.1 (Firm).")
    end
end

-- Callback for the Fast Attitude Log on/off selection
local function on_fast_att_log_change(selection)
    if selection == "On" then
        toggle_fast_attitude_logging(true)
    elseif selection == "Off" then
        toggle_fast_attitude_logging(false)
    end
end

-- Callback for the Batch Logging on/off selection
local function on_batch_log_change(selection)
    if selection == "On" then
        toggle_batch_logging(true)
    elseif selection == "Off" then
        toggle_batch_logging(false)
    end
end

-- ####################
-- # MENU DEFINITION
-- ####################

-- Define the shared options for the new gain selection menus
local gain_options_base = {"+50", "+25", "+10", "+5", "Hold", "-5", "-10", "-25", "-50"}

-- Build the options table with the unit appended to each value except "Hold"
local gain_options_formatted = {}
for _, v in ipairs(gain_options_base) do
    if v == "Hold" then
        table.insert(gain_options_formatted, v)
    else
        table.insert(gain_options_formatted, v .. TUNING_UNIT)
    end
end

-- Helper function to generate the suffix for the gain menu item titles
local function get_gain_suffix(selection)
    if selection == "All" then return "(All)" end
    if selection == "AngP" then return "(A)" end
    return "(" .. selection .. ")"
end

-- Callback for the "Gains to Tune" selection. Updates the state and the menu titles.
local function on_gains_to_tune_change(selection)
    gains_to_tune_selection = selection
    local suffix = get_gain_suffix(selection)

    -- Update the names. The crsf_helper will send the new name on the next refresh.
    if roll_gain_item then roll_gain_item.name = "Roll Gain " .. suffix end
    if pitch_gain_item then pitch_gain_item.name = "Pitch Gain " .. suffix end
    if yaw_gain_item then yaw_gain_item.name = "Yaw Gain " .. suffix end

    gcs:send_text(MAV_SEVERITY.INFO, "Now tuning: " .. selection .. " gains.")
end

-- Define the main gain items and store them in variables
roll_gain_item = {
    type = 'SELECTION',
    name = "Roll Gain " .. get_gain_suffix(gains_to_tune_selection),
    options = gain_options_formatted,
    default = default_gain_idx,
    unit = TUNING_UNIT,
    callback = function(selection) on_axis_gain_change("Roll", selection) end
}
pitch_gain_item = {
    type = 'SELECTION',
    name = "Pitch Gain " .. get_gain_suffix(gains_to_tune_selection),
    options = gain_options_formatted,
    default = default_gain_idx,
    unit = TUNING_UNIT,
    callback = function(selection) on_axis_gain_change("Pitch", selection) end
}
yaw_gain_item = {
    type = 'SELECTION',
    name = "Yaw Gain " .. get_gain_suffix(gains_to_tune_selection),
    options = gain_options_formatted,
    default = default_gain_idx,
    unit = TUNING_UNIT,
    callback = function(selection) on_axis_gain_change("Yaw", selection) end
}


-- This table defines the complete menu structure.
local menu_definition = {
    name = "Gain Tuner", -- The root menu name
    items = {
        -- ====== GAIN SELECTION ======
        roll_gain_item,
        pitch_gain_item,
        yaw_gain_item,

        -- ====== SETTINGS MENU (NEW) ======
        {
            type = 'MENU',
            name = "Settings",
            items = {
                {
                    type = 'SELECTION',
                    name = "Gains to Tune",
                    options = {"All", "P", "I", "D", "AngP"},
                    default = 1, -- 1-based index for "All"
                    callback = on_gains_to_tune_change
                }
            }
        },

        -- ====== AUTOTUNE MENU ======
        {
            type = 'MENU',
            name = "Autotune",
            items = {
                {
                    type = 'SELECTION',
                    name = "Setup",
                    options = {"None", "Roll/Pitch", "Yaw", "Yaw D"},
                    default = 1, -- 1-based index for "None"
                    callback = on_autotune_setup_change
                },
                {
                    type = 'SELECTION',
                    name = "Backoff",
                    options = {"Soft Tune", "Firm Tune"},
                    default = 1, -- 1-based index for "Soft Tune"
                    callback = on_backoff_change
                }
            }
        },
        -- ====== LOGGING MENU ======
        {
            type = 'MENU',
            name = "Logging",
            items = {
                {
                    type = 'SELECTION',
                    name = "Batch Logging",
                    options = {"Off", "On"},
                    default = 1, -- 1-based index for "Off"
                    callback = on_batch_log_change
                },
                {
                    type = 'SELECTION',
                    name = "Fast Attitude Log",
                    options = {"Off", "On"},
                    default = 1, -- 1-based index for "Off"
                    callback = on_fast_att_log_change
                },
                {type = 'INFO', name = "Warning", info = "Erase is final"},
                {
                    type = 'COMMAND',
                    name = "Erase All Logs",
                    info = "Hold to Erase",
                    callback = function(command_action)
                        if command_action == CRSF_COMMAND_STATUS.START then
                            erase_logs()
                        end
                        return CRSF_COMMAND_STATUS.READY, "Hold to Erase"
                    end
                },
            }
        },
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
-- # INITIALIZATION
-- ####################

-- Helper function to find a menu item by its name property
local function find_item_by_name(items_table, name)
    if not items_table then return nil end
    for _, item in ipairs(items_table) do
        -- Use string.match to find the base name, ignoring the dynamic suffix
        if item.name and string.match(item.name, "^" .. name) then
            return item
        end
    end
    return nil
end

-- Synchronizes the current selection index of menu items with their underlying parameter values.
-- This is used after a revert to ensure the UI display is correct.
sync_menu_selections_to_params = function()
    -- Autotune Setup
    local autotune_menu = find_item_by_name(menu_definition.items, "Autotune")
    if autotune_menu then
        local setup_selection = find_item_by_name(autotune_menu.items, "Setup")
        if setup_selection then
            local autotune_axes_param = assert(Parameter('AUTOTUNE_AXES'), "AUTOTUNE_AXES param not found")
            local current_axes_val = autotune_axes_param:get()
            if current_axes_val == 3 then setup_selection.current_idx = 2 -- "Roll/Pitch"
            elseif current_axes_val == 4 then setup_selection.current_idx = 3 -- "Yaw"
            elseif current_axes_val == 8 then setup_selection.current_idx = 4 -- "Yaw D"
            else setup_selection.current_idx = 1 -- "None"
            end
        end

        local backoff_selection = find_item_by_name(autotune_menu.items, "Backoff")
        if backoff_selection then
            local gmbk_param = assert(Parameter('AUTOTUNE_GMBK'), "AUTOTUNE_GMBK param not found")
            if gmbk_param:get() == 0.1 then
                backoff_selection.current_idx = 2 -- "Firm Tune"
            else
                backoff_selection.current_idx = 1 -- "Soft Tune"
            end
        end
    end

    -- Logging Settings
    local logging_menu = find_item_by_name(menu_definition.items, "Logging")
    if logging_menu then
        local batch_log_selection = find_item_by_name(logging_menu.items, "Batch Logging")
        if batch_log_selection then
            local bat_mask_param = assert(Parameter('INS_LOG_BAT_MASK'), "INS_LOG_BAT_MASK param not found")
            if bat_mask_param:get() ~= 0 then
                batch_log_selection.current_idx = 2 -- "On"
            else
                batch_log_selection.current_idx = 1 -- "Off"
            end
        end

        local fast_log_selection = find_item_by_name(logging_menu.items, "Fast Attitude Log")
        if fast_log_selection then
            local log_bitmask_param = assert(Parameter('LOG_BITMASK'), "LOG_BITMASK param not found")
            if log_bitmask_param:get() % 2 ~= 0 then
                fast_log_selection.current_idx = 2 -- "On"
            else
                fast_log_selection.current_idx = 1 -- "Off"
            end
        end
    end
end

-- Dynamically set initial menu values based on current parameter states
local function initialize_menu_defaults()
    -- Autotune Setup
    local autotune_menu = find_item_by_name(menu_definition.items, "Autotune")
    if autotune_menu then
        local setup_selection = find_item_by_name(autotune_menu.items, "Setup")
        if setup_selection then
            local autotune_axes_param = assert(Parameter('AUTOTUNE_AXES'), "AUTOTUNE_AXES param not found")
            local current_axes_val = autotune_axes_param:get()
            if current_axes_val == 3 then setup_selection.default = 2 -- "Roll/Pitch"
            elseif current_axes_val == 4 then setup_selection.default = 3 -- "Yaw"
            elseif current_axes_val == 8 then setup_selection.default = 4 -- "Yaw D"
            else setup_selection.default = 1 -- "None"
            end
        end

        local backoff_selection = find_item_by_name(autotune_menu.items, "Backoff")
        if backoff_selection then
            local gmbk_param = assert(Parameter('AUTOTUNE_GMBK'), "AUTOTUNE_GMBK param not found")
            if gmbk_param:get() == 0.1 then
                backoff_selection.default = 2 -- "Firm Tune"
            else
                backoff_selection.default = 1 -- "Soft Tune"
            end
        end
    end

    -- Logging Settings
    local logging_menu = find_item_by_name(menu_definition.items, "Logging")
    if logging_menu then
        local batch_log_selection = find_item_by_name(logging_menu.items, "Batch Logging")
        if batch_log_selection then
            local bat_mask_param = assert(Parameter('INS_LOG_BAT_MASK'), "INS_LOG_BAT_MASK param not found")
            if bat_mask_param:get() ~= 0 then
                batch_log_selection.default = 2 -- "On"
            else
                batch_log_selection.default = 1 -- "Off"
            end
        end

        local fast_log_selection = find_item_by_name(logging_menu.items, "Fast Attitude Log")
        if fast_log_selection then
            local log_bitmask_param = assert(Parameter('LOG_BITMASK'), "LOG_BITMASK param not found")
            if log_bitmask_param:get() % 2 ~= 0 then
                fast_log_selection.default = 2 -- "On"
            else
                fast_log_selection.default = 1 -- "Off"
            end
        end
    end
end

-- Run initialization
initialize_menu_defaults()

-- 1. Register this script's menu definition with the helper.
return crsf_helper.register_menu(menu_definition)
