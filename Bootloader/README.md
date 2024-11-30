# Bootloader for STM32F407: Reliable SD Card-Based Firmware Updater & Self-Updater

This project is a dual-purpose bootloader for STM32F407 microcontrollers that not only updates application firmware but also updates itself. By utilizing an SD card, the bootloader can autonomously scan for binary files, determine the most recent versions, validate them with CRC32, and flash them into the MCU's memory—ensuring both the application and bootloader remain up-to-date and reliable.  Designed for simplicity and robustness, this solution ensures the firmware remains up-to-date and reliable.

Features
Dual Update Capability: Updates both the application and the bootloader itself.
Automatic Version Selection: Scans the SD card and selects the latest version based on embedded version information.
CRC32 Validation: Ensures data integrity before flashing.
Simple Trigger Mechanism: Updates are initiated via button press during power-on.
Sequential Update Logic: Prioritizes bootloader updates; application updates occur on the next boot.
Small size: ~ 23K.

Instructions
Generating Binary Files with Embedded CRC Checksum
Open the projects F407VET6_OV7670_ILI9341_HAL and Bootloader in CubeIDE.
In each project's settings, define the desired major and minor version numbers using the macros APP_MAJOR_VERSION, APP_MINOR_VERSION, or BL_MAJOR_VERSION, BL_MINOR_VERSION.
Build the projects. Retrieve the generated binary files from the Debug folders.
Updating the Device Firmware Using the Bootloader
Ensure the Bootloader is Already Flashed
The bootloader must be pre-loaded into the MCU's flash memory.

Prepare the SD Card
Place the generated binary files in the root directory of the SD card and insert the card into the device.

Trigger the Bootloader Update

Press and hold both buttons on the device while powering it on.
File Scanning and Selection

The bootloader scans the SD card for binary files in the root directory.
If multiple application binaries are found, it selects the one with the highest version embedded in the file, provided it is newer than the currently flashed application or the current application is corrupted/missing.
Similarly, if multiple bootloader binaries are found, it selects the most recent version, provided it is newer than the currently flashed bootloader.
If both application and bootloader binaries are present, the bootloader binary will be updated first. The application binary will be updated during the next bootloader run.
Validation and Update Process

The selected binary file is validated using its embedded CRC32 checksum. If the checksum matches, the firmware is written to the MCU's flash memory.
After a successful bootloader update, the LED blinks 5 times. After an application update, the LED blinks 10 times.
In case of an error during the update, the LED blinks continuously.
Bootloader Handover
Once the update process is complete, the bootloader transfers control to the application.

Notes
Ensure the versioning macros (APP_MAJOR_VERSION, APP_MINOR_VERSION, etc.) are incremented appropriately during builds to maintain proper version tracking.
Use high-quality SD cards to minimize the risk of data corruption during the update process.

## Technology stack:
Bootloader, Bootloader Updater, STM32F407VET6, SDIO, FLASH, FatFS, CMSIS, HAL.
