# Boiling Matter

This creates a DS18B20 temperature sensing device from a ESP32-C6
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

## Configuration

Sensor and power settings live under `Boiling Matter Configuration` in
`idf.py menuconfig`:

| Option | Default | Effect |
| --- | --- | --- |
| `SENSOR_READ_INTERVAL` | 30000 ms | Temperature sampling period |
| `TEMPERATURE_REPORT_DEADBAND` | 30 (0.30 °C) | Minimum change before a temperature report |
| `DS18B20_RESOLUTION` | 10 bits | Sensor resolution (conversion time and energy) |
| `BATTERY_READ_INTERVAL_SEC` | 900 s | Battery sampling period |
| `BATTERY_REPORT_DEADBAND` | 2 (1 %) | Minimum battery percentage change before a report |

Every Matter report keeps the device in fast-poll active mode for the ICD
active mode threshold (5 s), so the deadbands have the biggest effect on
battery life.

`sdkconfig.defaults` only applies to a newly generated `sdkconfig`. After
pulling changes to it, regenerate the configuration (this also clears the
build directory):

```bash
idf.py set-target esp32c6
```

Logging defaults to WARN because console output keeps the CPU awake. Raise
`Component config → Log → Log Level → Default log verbosity` in menuconfig
when debugging.

