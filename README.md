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

## TLS certificate renewal and domain migration

SSLClient 1.6.11 initializes certificate verification with its own compilation
timestamp. The firmware now supplies the current UTC system time before every
TLS connection, including retries. A correct timestamp in the device log alone
did not previously mean that certificate verification used that time. Cached
library objects can retain an old compilation date even in a newer firmware build.

After boot, a device without a sane system clock reports `tls_time_unsynced`
and waits for time synchronization instead of using the library build date.
Keep NTP (UDP port 123) available to the instrument. TLS failures now include
`bearssl_err` in addition to the Arduino write error (`ssl_err`): certificate
validation can fail while `ssl_err` is zero. BearSSL code 54 indicates a
certificate validity-date failure; 62 indicates an untrusted certificate chain.

The September 2026 domain-migration check used the live
`dashboard.albasqueeze.com` certificate chain, the bundled BearSSL TLS 1.2
profile, and the firmware trust anchors. Validation returned 54 with a June 18,
2026 verification date and 0 with the current date. The server also accepted
`ECDHE-RSA-AES128-GCM-SHA256` with an RSA/SHA-256 handshake signature.

Build and install the updated application firmware on both instruments. Keep
the server URL, device IDs, API keys, and command secrets unchanged. The fix
preserves certificate-chain and hostname verification; physical-device
connectivity must be checked after installation.

## Fixed production destination

Production firmware uses `https://dashboard.albasqueeze.com` automatically.
The URL is immutable in the application configuration, overrides any previously
saved destination at boot, and is displayed read-only in the AP portal. Submitted
`serverUrl` overrides are ignored. Only the saved URL is migrated; device IDs,
API keys, command secrets, and all other settings remain intact. Existing
instruments need only the firmware update, with no manual URL change.

New or factory-reset instruments still need their individual credentials and
cloud/remote settings provisioned; this firmware does not embed shared secrets.
