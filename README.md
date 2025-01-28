# Bike trainer sensor
This is an open source sensor for bike trainers, such as Tacx Boost, Kurt Kinetc Road, Elite Qubo Fluid etc. Purpose of this project is to be able to replace the sensor on your trainer if it stops working, or if the company shuts down trainer app. This happened to me with Kurt Kinetec, and since they have propriarety BLE service, the sensor is useless.

This sensor uses Bluetooth LE protocol with Cycling Speed and Cadence Profile, which is standard profile and compatible with all trainer apps, like Zwift, TrainerRoad and others.

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
   - Select *Base configuration files*: `prj_release.conf` or `prj_debug.conf`
   - Select *Extra Kconfig fragmetns*: `bt.conf`

For API and how to, check [Nordic's developer website](https://docs.nordicsemi.com/bundle/ncs-2.9.0/page/nrf/index.html).