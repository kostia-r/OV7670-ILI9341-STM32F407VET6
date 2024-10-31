# CAMERA APPLICATION

OV7670 (DCMI) -> STM32F407VET6 -> TFT ILI9341 (SPI) -> SD Card (SDIO)

Camera application on a bare-metal platform that streams video from an OV7670 camera to an ILI9341 SPI display, with photo capture and saving to an SD card. The system uses double-buffering to reduce memory consumption, achieving a frame rate of ~13 FPS by utilizing DMA for efficient data transfers. User input is handled by a single button with three press modes. Photos are saved in JPEG format on the SD card via SDIO and FatFS, using the  LIBJPEG Encoder library.

Tehnology stack: Embedded Graphics, STM32F407VET6, OV7670, ILI9341, DCMI, SPI, I2C, SDIO, DMA, TIM, ISR, BUTTON, FatFS, LIBJPEG, HAL, CMSIS, Finite State Machines.
