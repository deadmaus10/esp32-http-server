# ESP32 HTTP Server Example

ESP32 web server example: control 2 LEDs from a web page hosted on the ESP32.

Use [Wokwi for Visual Studio Code](https://marketplace.visualstudio.com/items?itemName=wokwi.wokwi-vscode) to simulate this project.

## Building

This is a [PlatformIO](https://platformio.org) project. To build it, [install PlatformIO](https://docs.platformio.org/en/latest/core/installation/index.html), and then run the following command:

```
pio run
```

## Hardware-Free Tests

The repository includes a native PlatformIO test suite for firmware logic that
does not require the ESP32 hardware. This gives you a fast regression net for
the parts of the firmware that are safe to validate on a workstation.

Run the host-side tests with:

```
pio test -e native
```

The same host-side tests and the ESP32 firmware build also run automatically in
GitHub Actions on every push and pull request.

The native suite currently covers:

- engineering scaling and gain-code helpers
- alarm hysteresis behavior
- automatic measurement session rollover before the 32-bit sample timer limit
- upload-wait and retry behavior before auto-restarting a measurement
- remote-control cooldown and poll backoff logic
- AM1 header and frame layout regression checks

You can still verify that the full firmware builds for the target device with:

```
pio run -e esp32
```

## Simulating

To simulate this project, install [Wokwi for VS Code](https://marketplace.visualstudio.com/items?itemName=wokwi.wokwi-vscode). Open the project directory in Visual Studio Code, press **F1** and select "Wokwi: Start Simulator".

Once the simulation is running, open http://localhost:8180 in your web browser to interact with the simulated HTTP server.

## Measurement exports

The firmware stores measurement sessions as binary `.am1` files (header plus
packed samples). To inspect or convert captures on your workstation, install
the Python tooling requirements and use the helper script in `scripts/`:

```
python3 -m pip install numpy
```

### Inspect a capture

```
python3 scripts/am1tool.py info /path/to/sess_2024-05-18_09-55-12.am1
```

The `info` subcommand prints the session metadata (start time, configured
gains, engineering scaling, etc.) along with sample counts and timing stats.

### Convert to CSV

```
python3 scripts/am1tool.py csv /path/to/sess_2024-05-18_09-55-12.am1 --cols full --output sess.csv
```

Use `--cols raw`, `--cols rawmv`, or `--cols full` (default) to match the CSV
column sets exposed by `/export_csv` on the device. If you omit `--output` the
tool streams CSV rows to standard output, which allows piping into other tools
for offline analysis.
