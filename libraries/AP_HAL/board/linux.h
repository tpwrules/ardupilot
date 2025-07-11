#pragma once

#include <hwdef.h>

#define HAL_BOARD_NAME "Linux"
#define HAL_CPU_CLASS HAL_CPU_CLASS_1000
#define HAL_MEM_CLASS HAL_MEM_CLASS_1000
#define HAL_OS_SOCKETS 1
#define HAL_STORAGE_SIZE            16384
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE

// FIXME: use of this define should go away:
#ifndef CONFIG_HAL_BOARD_SUBTYPE
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_LINUX_NONE
#endif  // CONFIG_HAL_BOARD_SUBTYPE

#ifndef HAL_PROGRAM_SIZE_LIMIT_KB
#define HAL_PROGRAM_SIZE_LIMIT_KB 4096
#endif  // HAL_PROGRAM_SIZE_LIMIT_KB

#ifndef HAL_OPTFLOW_PX4FLOW_I2C_ADDRESS
    #define HAL_OPTFLOW_PX4FLOW_I2C_ADDRESS 0x42
#endif

#ifndef HAL_OPTFLOW_PX4FLOW_I2C_BUS
    #define HAL_OPTFLOW_PX4FLOW_I2C_BUS 1
#endif

#define HAL_HAVE_BOARD_VOLTAGE 1
#define HAL_HAVE_SAFETY_SWITCH 0


#ifndef HAL_HAVE_SERVO_VOLTAGE
    #define HAL_HAVE_SERVO_VOLTAGE 0
#endif

#ifndef AP_STATEDIR
    #define HAL_BOARD_STATE_DIRECTORY "/var/lib/ardupilot"
#else
    #define HAL_BOARD_STATE_DIRECTORY AP_STATEDIR
#endif

#ifndef HAL_BOARD_LOG_DIRECTORY
    #define HAL_BOARD_LOG_DIRECTORY HAL_BOARD_STATE_DIRECTORY "/logs"
#endif

#ifndef HAL_BOARD_TERRAIN_DIRECTORY
    #define HAL_BOARD_TERRAIN_DIRECTORY HAL_BOARD_STATE_DIRECTORY "/terrain"
#endif

#ifndef HAL_BOARD_STORAGE_DIRECTORY
    #define HAL_BOARD_STORAGE_DIRECTORY HAL_BOARD_STATE_DIRECTORY
#endif

#ifndef HAL_BOARD_CAN_IFACE_NAME
    #define HAL_BOARD_CAN_IFACE_NAME "can0"
#endif

// if bus masks are not setup above then use these defaults
#ifndef HAL_LINUX_I2C_BUS_MASK
    #define HAL_LINUX_I2C_BUS_MASK 0xFFFF
#endif

#ifndef HAL_LINUX_I2C_INTERNAL_BUS_MASK
    #define HAL_LINUX_I2C_INTERNAL_BUS_MASK 0xFFFF
#endif

#ifndef HAL_LINUX_I2C_EXTERNAL_BUS_MASK
    #define HAL_LINUX_I2C_EXTERNAL_BUS_MASK 0xFFFF
#endif

// only include if compiling C++ code
#ifdef __cplusplus
#include <AP_HAL_Linux/Semaphores.h>
#define HAL_Semaphore Linux::Semaphore
#define HAL_BinarySemaphore Linux::BinarySemaphore
#endif

#ifndef HAL_HAVE_HARDWARE_DOUBLE
#define HAL_HAVE_HARDWARE_DOUBLE 1
#endif

#ifndef HAL_WITH_EKF_DOUBLE
#define HAL_WITH_EKF_DOUBLE HAL_HAVE_HARDWARE_DOUBLE
#endif

#ifndef HAL_GYROFFT_ENABLED
#define HAL_GYROFFT_ENABLED 0
#endif

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NONE
// we can use virtual CAN on native builds
#define HAL_LINUX_USE_VIRTUAL_CAN 1
#else
#define HAL_LINUX_USE_VIRTUAL_CAN 0
#endif

#ifndef HAL_OS_POSIX_IO
#define HAL_OS_POSIX_IO 1
#endif

#ifndef HAL_INS_RATE_LOOP
#define HAL_INS_RATE_LOOP 1
#endif

#ifndef HAL_LINUX_GPIO_AERO_ENABLED
#define HAL_LINUX_GPIO_AERO_ENABLED 0
#endif

#ifndef HAL_LINUX_GPIO_BBB_ENABLED
#define HAL_LINUX_GPIO_BBB_ENABLED 0
#endif

#ifndef HAL_LINUX_GPIO_BEBOP_ENABLED
#define HAL_LINUX_GPIO_BEBOP_ENABLED 0
#endif

#ifndef HAL_LINUX_GPIO_DISCO_ENABLED
#define HAL_LINUX_GPIO_DISCO_ENABLED 0
#endif

#ifndef HAL_LINUX_GPIO_EDGE_ENABLED
#define HAL_LINUX_GPIO_EDGE_ENABLED 0
#endif

#ifndef HAL_LINUX_GPIO_NAVIGATOR_ENABLED
#define HAL_LINUX_GPIO_NAVIGATOR_ENABLED 0
#endif

#ifndef HAL_LINUX_GPIO_NAVIO_ENABLED
#define HAL_LINUX_GPIO_NAVIO_ENABLED 0
#endif

#ifndef HAL_LINUX_GPIO_NAVIO2_ENABLED
#define HAL_LINUX_GPIO_NAVIO2_ENABLED 0
#endif

#ifndef HAL_LINUX_GPIO_RPI_ENABLED
#define HAL_LINUX_GPIO_RPI_ENABLED 0
#endif

#ifndef HAL_LINUX_GPIO_SYSFS_ENABLED
#define HAL_LINUX_GPIO_SYSFS_ENABLED 0
#endif

#ifndef HAL_LINUX_GPIO_PILOTPI_ENABLED
#define HAL_LINUX_GPIO_PILOTPI_ENABLED 0
#endif
