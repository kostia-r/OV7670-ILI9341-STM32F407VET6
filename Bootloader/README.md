# Custom Bootloader for STM32F407: Reliable SD Card-Based Firmware Updater & Self-Updater

This project is a dual-purpose bootloader for STM32F407 microcontrollers that not only updates application firmware but also updates itself. By utilizing an SD card, the bootloader can autonomously scan for binary files, determine the most recent versions, validate them with CRC32, and flash them into the MCU's memory—ensuring both the application and bootloader remain up-to-date and reliable.  Designed for simplicity and robustness, this solution ensures the firmware remains up-to-date and reliable.

## Features
* __Dual Update Capability:__ Updates both the application and the bootloader itself.
* __Automatic Version Selection:__ Scans the SD card and selects the latest version based on embedded version information.
* __CRC32 Validation:__ Ensures data integrity before flashing.
* __Simple Trigger Mechanism:__ Updates are initiated via buttons press during power-on.
* __Sequential Update Logic:__ Prioritizes bootloader updates; application updates occur on the next boot.
* __Small size:__ ~ 23K.

## Instructions
### Generating Binary Files with Embedded CRC Checksum
1. __Open the projects F407VET6_OV7670_ILI9341_HAL (application) and Bootloader in CubeIDE.__
2. __Define the desired version numbers using macros:__
     * For the application: `APP_MAJOR_VERSION`, `APP_MINOR_VERSION`.
     * For the bootloader: `BL_MAJOR_VERSION`, `BL_MINOR_VERSION`.
4. __Build the projects and retrieve the generated binary files from their respective Debug folders.__

### Updating the Firmware (Application & Bootloader)
1. __Ensure Bootloader is Pre-Flashed__
     * The bootloader must already be loaded into the MCU's flash memory.
2. __Prepare the SD Card__
     * Place the generated application (.bin) and/or bootloader binary files in the root directory of the SD card.
     * Insert the SD card into the device.
3. __Initiate Update Process__
     * Press and hold both buttons on the device while powering it on.
4. __Bootloader Scans & Updates__
     * File Selection Logic:
       * If multiple binaries for the application or bootloader are found, the bootloader selects the most recent version.
       * Updates occur only if the new version is higher than the current one in memory, or if the existing firmware is missing/corrupted.
       * Bootloader updates are prioritized over application updates. Application updates will occur on the next run of the bootloader.
     * Validation: Selected binary files are verified using their embedded CRC32 checksum.
5. __Flashing & Status Indication__
     * Successful bootloader update: LED blinks 5 times.
     * Successful application update: LED blinks 10 times.
     * Update failure: LED blinks continuously.
6. __Handover to Application__
     * After completing the update process, the bootloader transfers control to the application firmware.

### Notes
1. Ensure the versioning macros (`APP_MAJOR_VERSION`, `APP_MINOR_VERSION`, etc.) are incremented appropriately during builds to maintain proper version tracking.
2. Use high-quality SD cards to minimize the risk of data corruption during the update process.

## Technology stack:
Bootloader, Bootloader Updater, STM32F407VET6, SDIO, FLASH, FatFS, CMSIS, HAL.
