# **ArduPilot CRSF Gain Tuner \- User Manual**

## **1\. Introduction**

The ArduPilot CRSF Gain Tuner is a powerful Lua script that provides an intuitive menu system directly on your CRSF-compatible transmitter (such as EdgeTX or OpenTX). It allows for rapid in-flight tuning of PID and Angle P gains without the need for a laptop or GCS connection.

This tool is designed for pilots who want to refine their aircraft's performance quickly and efficiently, offering features like percentage-based adjustments, stateful menus that reflect your current settings, and integrated utility functions for logging and Autotune.

## **2\. Prerequisites & Installation**

### **2.1. Prerequisites**

Before installing the script, you must enable Lua scripting in ArduPilot.

1. Connect your flight controller to a Ground Control Station (e.g., Mission Planner, QGroundControl).  
2. Navigate to the full parameter list.  
3. Set the SCR\_ENABLE parameter to 1\.  
4. For full menu functionality on your transmitter, ensure RC\_OPTIONS does not disable CRSF Telemetry (bit 8 should be 0\) and CRSF\_TELEM\_HID is enabled on your CRSF receiver.  
5. Reboot the flight controller.

### **2.2. Installation**

The Gain Tuner requires two files to be installed on your flight controller's SD card:

1. gain\_tuner.lua: The main script containing the tuning logic.  
2. crsf\_helper.lua: A necessary library that manages the CRSF menu system.

**Installation Steps:**

1. Power down your flight controller and remove the SD card.  
2. Insert the SD card into a computer.  
3. Navigate to the APM/scripts/ directory on the SD card. If this directory does not exist, create it.  
4. Copy both gain\_tuner.lua and crsf\_helper.lua into the APM/scripts/ directory.  
5. Safely eject the SD card and re-insert it into your flight controller.  
6. Power on your flight controller. The script will automatically be loaded.

## **3\. Configuration & Menu Overview**

To access the Gain Tuner, navigate to the "Lua Scripts" or "Tools" menu on your transmitter. You should see an entry named **"Gain Tuner"**.

### **3.1. Main Tuning Menu**

This is the main screen where you will spend most of your time. It provides sliders for adjusting the gains for each primary axis.

* **Roll Gain (All):** Adjusts Roll axis gains. The suffix (All) indicates it will modify P, I, D, and Angle P gains simultaneously by default.  
* **Pitch Gain (All):** Adjusts Pitch axis gains.  
* **Yaw Gain (All):** Adjusts Yaw axis gains.

The sliders are percentage-based relative to the values present when the script started (or when they were last saved). "Hold" represents 0% change.

### **3.2. Settings Menu**

This submenu allows you to select which specific gains are affected by the main sliders. This setting is now saved and will persist across reboots.

* **Gains to Tune:** Choose from:  
  * All: (Default) Adjusts P, I, D, and Angle P gains together.  
  * P: Adjusts only the Rate P gain for each axis.  
  * I: Adjusts only the Rate I gain for each axis.  
  * D: Adjusts only the Rate D gain for each axis.  
  * AngP: Adjusts only the Angle P (stabilize) gain for each axis.

When you change this setting, the titles on the main menu will update to reflect your choice (e.g., "Roll Gain (P)").

### **3.3. Autotune Menu**

This submenu provides convenient shortcuts for configuring an Autotune session. The menu will automatically display the currently configured parameter values.

* **Setup:** Selects which axes will be tuned (AUTOTUNE\_AXES).  
* **Backoff:** Sets how aggressively the new gains are applied (AUTOTUNE\_GMBK).

### **3.4. Logging Menu**

This menu provides tools for managing flight logs. The menu will reflect the current parameter settings on startup.

* **Batch Logging:** Enables high-rate, post-flight logging of IMU data.  
* **Fast Attitude Log:** Toggles the ATTITUDE\_FAST bit in the LOG\_BITMASK.  
* **Erase All Logs:** A command to wipe all flight logs from the SD card. **Use with caution, as this cannot be undone.**

### **3.5. Save & Revert Menu**

This menu is for managing all of your tuned settings.

* **Save All Settings:** Writes all current PID gains and any changed Autotune/Logging parameters to permanent memory. After saving, these new values become the baseline for future adjustments.  
* **Revert All Settings:** Resets all PID gains and Autotune/Logging parameters back to the values they had when the script first started (or the last time they were saved). The entire menu will update to reflect the reverted state.

## **4\. Usage Workflow \- A Typical Tuning Session**

1. **Pre-flight:** Install the scripts and ensure your aircraft has a stable baseline tune.  
2. **Access Menu:** In flight, access the "Gain Tuner" menu on your transmitter.  
3. **Select Gains:** Go to Settings \-\> Gains to Tune and select which gains you want to adjust (e.g., P for initial tuning).  
4. **Make Adjustments:** Return to the main menu. Fly the aircraft and make small adjustments using the sliders. For example, increase Roll Gain (P) by \+5% and observe the response.  
5. **Iterate:** Continue making small adjustments to P, I, and D gains across the axes until you are satisfied with the aircraft's response.  
6. **Save:** Once you are happy with the tune, navigate to Save & Revert \-\> Save All Settings. This will make your changes permanent.  
7. **Post-flight:** Use the Logging menu to manage logs for offline analysis if needed.

**Happy Tuning\!**