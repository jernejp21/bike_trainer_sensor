# Bike trainer sensor
This is an open source sensor for bike trainers, such as Tacx Boost, Kurt Kinetc Road, Elite Qubo Fluid etc. Purpose of this project is to be able to replace the sensor on your trainer if it stops working, or if the company shuts down the trainer app. This happened to me with Kurt Kinetec, and since they have propriarety BLE service, the sensor is useless.

This sensor uses Bluetooth LE protocol with Fitness Machine Service (FTMS), which is standard service and compatible with all trainer apps, like MyWhoosh, Zwift, TrainerRoad and others.

The uploaded firmware is for Kinetic Road Trainer T2700. FTMS provides only data for the Indoor bike -> Instantaneous power.

# How to use this?
## Prepare the nRF52840 dongle for FOTA

1. Download `mcuboot-<commit number>.hex` from releases. Use hash function to check if the file is correct.
2. Insert nRF52840 dongle into USB port on your PC and click reset button on the dongle, to get into nRF5 bootloader (red LED should be pulsing).
3. With [nRF Connect for Desktop](https://www.nordicsemi.com/Products/Development-tools/nrf-connect-for-desktop/download) write `mcuboot-<commit numbr>.hex` to the dongle.
4. Next, unplug the dongle, hold down white button and plug the dongle back to the USB port. Green LED should be on, showing the program is now waiting in the MCUboot. Write `bike_trainer-<commit number>`.signed.bin to the dongle with [AuTerm](https://github.com/thedjnK/AuTerm).
5. Check if everything is working with the nRF Connect Android or iOS app.

## Prepare the nrf52840 dongle for battery power supply

1. Before doing this, make sure you can do FOTA. Once the board is configued for battery power, it cannot be connected to the USB port.

# Firmware development

## SDK Installation

[Video instructions](https://youtube.com/playlist?list=PLx_tBuQ_KSqEt7NK-H7Lu78lT2OijwIMl)

1. Install nRF Command Line Tools ([Download](https://bit.ly/2YgBGC5))
2. Install Visual Studio Code ([Download](https://code.visualstudio.com/Download))
3. **Make sure to install SDK version 2.9.0!**

## Development setup

1. After cloning git repository, open `Firmware.code-workspace` with VS Code.
2. Go to nRF Connect and create *build configuration*. Under APPLICATIONS add build the configuration.
   - Board: `nrf52840dk_nrf52840` or other board.
   - Build directory name: `release` or `debug`
   - Select *Extra Kconfig fragmetns*: `debug-overlay.conf` or `release-overlay.conf`
3. If using dongle, make sure you disable partition manager with `SB_CONFIG_PARTITION_MANAGER=n` in *sysbuild.conf* file.

For API and how to, check [Nordic's developer website](https://docs.nordicsemi.com/bundle/ncs-2.9.0/page/nrf/index.html).