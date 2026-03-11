# Boiling Matter

This creates a DS20B18 temperature sensing device from a ESP32-C6
board for battery-powered sensing operation configured with Matter
over Thread.

See the [docs](https://docs.espressif.com/projects/esp-matter/en/latest/esp32/developing.html) for more information about building and flashing the firmware.

## Basic setup

Pull the `esp-idf` and `esp-matter` submodules and install them according
to the instructions in their programming guides. The development environment
can then be setup using

```bash
. esp-idf/export.sh
. esp-matter/export.sh
```

Generate the configuration with

```bash
idf.py set-target esp32c6
```

Build, flash and monitor the device with

```bash
idf.py flash monitor
```

The device can be commissioned by scanning using the test qr code in the
`esp-matter` programming guide.

