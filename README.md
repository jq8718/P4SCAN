# ESP32-P4 Camera + USB Audio Project

This workspace contains a starter ESP-IDF project for the ESP32-P4 Function EV Board Camera Subboard using:

- CSI camera path via the camera subboard
- HUSB-based USB device path for a USB Audio Class (UAC) implementation
- 16MB external flash and 4MB octal PSRAM defaults

## What is included

- A CSI camera initialization path using the Espressif camera component
- A dedicated USB audio entry point for future TinyUSB UAC descriptor and PCM streaming work
- Hardware-oriented defaults in [sdkconfig.defaults](sdkconfig.defaults)

## Build

From this directory:

```bash
idf.py set-target esp32p4
idf.py build
```

## Notes

- The camera pin mapping is left as a configurable placeholder because the exact subboard wiring can differ by revision.
- The current USB audio code is a scaffold for the UAC integration path and should be expanded with TinyUSB descriptors and PCM data handling on the target hardware.
