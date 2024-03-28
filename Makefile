################################################################################
# \file Makefile
# \version 1.0
#
# \brief
# Bluetooth LE Voice Remote Solution Demo Application Makefile.
#
################################################################################
# \copyright
# Copyright 2018-2020 Cypress Semiconductor Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################


################################################################################
# Basic Configuration
################################################################################

# Type of ModusToolbox Makefile Options include:
#
# COMBINED    -- Top Level Makefile usually for single standalone application
# APPLICATION -- Top Level Makefile usually for multi project application
# PROJECT     -- Project Makefile under Application
#
MTB_TYPE=COMBINED

# Target board/hardware (BSP).
# To change the target, it is recommended to use the Library manager 
# ('make modlibs' from command line), which will also update Eclipse IDE launch 
# configurations. If TARGET is manually edited, ensure TARGET_<BSP>.mtb with a 
# valid URL exists in the application, run 'make getlibs' to fetch BSP contents
# and update or regenerate launch configurations for your IDE.
TARGET=CYW920829-VR

# Name of application (used to derive name of final linked file).
# 
# If APPNAME is edited, ensure to update or regenerate launch 
# configurations for your IDE.
APPNAME=mtb-example-btstack-freertos-cyw20829-voice-remote

# Name of toolchain to use. Options include:
#
# GCC_ARM -- GCC provided with ModusToolbox software
# ARM     -- ARM Compiler (must be installed separately)
# IAR     -- IAR Compiler (must be installed separately)
#
# See also: CY_COMPILER_PATH below
TOOLCHAIN=GCC_ARM

# Default build configuration. Options include:
#
# Debug   -- build with minimal optimizations, focus on debugging.
# Release -- build with full optimizations
# Custom -- build with custom configuration, set the optimization flag in CFLAGS
# 
# If CONFIG is manually edited, ensure to update or regenerate launch configurations 
# for your IDE.
CONFIG=Custom

# If set to "true" or "1", display full command-lines when building.
VERBOSE=

#APPTYPE=flash

################################################################################
# Advanced Configuration
################################################################################

# Enable optional code that is ordinarily disabled by default.
#
# Available components depend on the specific targeted hardware and firmware
# in use. In general, if you have
#
#    COMPONENTS=foo bar
#
# ... then code in directories named COMPONENT_foo and COMPONENT_bar will be
# added to the build
#
COMPONENTS=FREERTOS WICED_BLE

# Like COMPONENTS, but disable optional code that was enabled by default.
DISABLE_COMPONENTS=

# By default the build system automatically looks in the Makefile's directory
# tree for source code and builds it. The SOURCES variable can be used to
# manually add source code to the build process from a location not searched
# by default, or otherwise not found by the build system.
SOURCES=

# Like SOURCES, but for include directories. Value should be paths to
# directories (without a leading -I).
#INCLUDES=./configs

# Choose one of the Peer device(Codec) for your testing. CySmart and Windows
# shall support Non-voice features.
ATV_ADPCM = 1
OPUS_CODEC = 2

ANALOG_MIC = 1
PDM_MIC = 2

ENABLE_RED_LED = 0
ENABLE_GREEN_LED = 0

ENABLE_BT_SPY = 0

# The following variable controls which codec is used. And BT-Configurator opens
# the respective design.cybt. Make clean will not delete the GeneratedSource
# directory. Needs to be cleaned manually.
ENABLE_CODEC = $(ATV_ADPCM)

ifeq ($(TARGET), $(filter $(TARGET), APP_CYW920829M2EVK-02))
CY_IGNORE+=./app_hw/app_hw_keyscan.c ./app_hw/app_hw_batmon.c
ENABLE_MIC = $(PDM_MIC)
ENABLE_RED_LED = 0
ENABLE_GREEN_LED = 0

ifeq ($(ENABLE_MIC) , $(ANALOG_MIC))
$(error CYW920829M2EVK-02 BLE Remote app support digital mic only. )
endif

endif

ifeq ($(TARGET), $(filter $(TARGET), APP_CYW920829-VR))
DEFINES+=VOICE_REMOTE
CY_IGNORE+=./app_m2_evk
ENABLE_MIC = $(ANALOG_MIC)

ifeq ($(ENABLE_RED_LED),1)
DEFINES+=RED_LED_ENABLE
endif

ifeq ($(ENABLE_GREEN_LED),1)
DEFINES+=GREEN_LED_ENABLE
endif

ifneq ($(ENABLE_RED_LED),1)
ifneq ($(ENABLE_GREEN_LED),1)
CY_IGNORE+=./app_hw/app_hw_gpio.c
endif
endif

endif

ifeq ($(ENABLE_CODEC),$(ATV_ADPCM))

# Add additional defines to the build process (without a leading -D).
DEFINES+=CY_RETARGET_IO_CONVERT_LF_TO_CRLF CY_RTOS_AWARE STACK_INSIDE_FREE_RTOS CONFIG_ADPCM_CODEC ATV_ADPCM BATTERY_SERVICE TEST_PATCH_RAM_DOWNLOAD DEBUG
CY_IGNORE+=./opus-1.3.1 ./app_audio/cy_opus.c ./app_bt/app_bt_hid_bsa.c ./app_bt_configs/bsa_opus
LINKER_SCRIPT=./linker_file/cyw20829_ns_flash_adpcm.ld
ifeq ($(ENABLE_MIC),$(PDM_MIC))
DEFINES+=PDM_MIC
CY_IGNORE+=./app_audio/adc_mic.c
else ifeq ($(ENABLE_MIC),$(ANALOG_MIC))
CY_IGNORE+=./app_audio/pdm_mic.c
endif

INCLUDES=./app_configs
CFLAGS+=-Og
else ifeq ($(ENABLE_CODEC),$(OPUS_CODEC))

DEFINES+=CY_RETARGET_IO_CONVERT_LF_TO_CRLF CY_RTOS_AWARE STACK_INSIDE_FREE_RTOS HAVE_CONFIG_H OPUS_ARM_ASM CONFIG_OPUS_CODEC BSA_OPUS BATTERY_SERVICE TEST_PATCH_RAM_DOWNLOAD DEBUG
CY_IGNORE+=./app_audio/cy_adpcm.c ./app_audio/adpcm.h ./app_audio/adpcm.c ./app_bt/app_bt_hid_atv.c ./app_bt_configs/atv_adpcm

ifeq ($(ENABLE_MIC),$(PDM_MIC))
DEFINES+=PDM_MIC
CY_IGNORE+=./app_audio/adc_mic.c
else ifeq ($(ENABLE_MIC),$(ANALOG_MIC))
CY_IGNORE+=./app_audio/pdm_mic.c
endif

INCLUDES=./app_configs
CFLAGS+=-O2
LINKER_SCRIPT=./linker_file/cyw20829_ns_flash_cbus.ld
endif

CY_IGNORE+=./templates

ifeq ($(ENABLE_BT_SPY),1)
DEFINES+=ENABLE_BT_SPY_LOG DEBUG_UART_BAUDRATE=3000000
endif
# Select softfp or hardfp floating point. Default is softfp.
VFP_SELECT=

# Additional / custom C compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
#CFLAGS=

# Additional / custom C++ compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
CXXFLAGS=

# Additional / custom assembler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
ASFLAGS=

# Additional / custom linker flags.
LDFLAGS=

# Additional / custom libraries to link in to the application.
LDLIBS=

# Path to the linker script to use (if empty, use the default linker script).
#LINKER_SCRIPT=

# Custom pre-build commands to run.
PREBUILD=

# Custom post-build commands to run.
POSTBUILD=

# Supported Compilers
CY_TOOLCHAIN_ARM_NOT_SUPPORTED = true
CY_TOOLCHAIN_IAR_NOT_SUPPORTED = true

################################################################################
# Paths
################################################################################

# Relative path to the project directory (default is the Makefile's directory).
#
# This controls where automatic source code discovery looks for code.
CY_APP_PATH=

# Relative path to the shared repo location.
#
# All .mtb files have the format, <URI>#<COMMIT>#<LOCATION>. If the <LOCATION> field 
# begins with $$ASSET_REPO$$, then the repo is deposited in the path specified by 
# the CY_GETLIBS_SHARED_PATH variable. The default location is one directory level 
# above the current app directory.
# This is used with CY_GETLIBS_SHARED_NAME variable, which specifies the directory name.
CY_GETLIBS_SHARED_PATH=../

# Directory name of the shared repo location.
#
CY_GETLIBS_SHARED_NAME=mtb_shared

# Absolute path to the compiler's "bin" directory.
#
# The default depends on the selected TOOLCHAIN (GCC_ARM uses the ModusToolbox
# IDE provided compiler by default).
CY_COMPILER_PATH=


# Locate ModusToolbox IDE helper tools folders in default installation
# locations for Windows, Linux, and macOS.
CY_WIN_HOME=$(subst \,/,$(USERPROFILE))
CY_TOOLS_PATHS ?= $(wildcard \
    $(CY_WIN_HOME)/ModusToolbox/tools_* \
    $(HOME)/ModusToolbox/tools_* \
    /Applications/ModusToolbox/tools_*)

# If you install ModusToolbox IDE in a custom location, add the path to its
# "tools_X.Y" folder (where X and Y are the version number of the tools
# folder).
CY_TOOLS_PATHS+=

# Default to the newest installed tools folder, or the users override (if it's
# found).
CY_TOOLS_DIR=$(lastword $(sort $(wildcard $(CY_TOOLS_PATHS))))

ifeq ($(CY_TOOLS_DIR),)
$(error Unable to find any of the available CY_TOOLS_PATHS -- $(CY_TOOLS_PATHS). On Windows, use forward slashes.)
endif

$(info Tools Directory: $(CY_TOOLS_DIR))

include $(CY_TOOLS_DIR)/make/start.mk
