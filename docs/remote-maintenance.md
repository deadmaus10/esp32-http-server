# Restart diagnostics, remote logs, and remote OTA

Install this firmware once through USB or the existing authenticated local OTA portal.
Older firmware cannot accept the new remote commands. Keep the current partition layout
(`default.csv`, two application slots). Use the application `firmware.bin`, never a merged
flash/bootloader image. Hardware validation on a spare instrument is required before
updating a field instrument.

## Restart diagnostics

The firmware saves the four latest boot records in the `restartdiag` NVS namespace.
Each record includes the boot counter, time (or uptime if time is unavailable), ESP32
reset reason, planned restart description, and the last retained runtime breadcrumb.
It writes history at boot and stores a planned reason immediately before intentional
restarts; periodic diagnostics do not write NVS. Both `/status` and cloud telemetry
include `restart_history`, `restart_history_stored`, `firmware_build`, and
`maintenance_capabilities`. The dashboard displays restart history near the update controls.

The runtime breadcrumb uses RTC no-init memory with a version and checksum, allowing
it to survive software resets, panics, and watchdog resets when RTC memory survives.
It is deliberately discarded on power-on and brownout; the actual reset reason is still
saved in NVS on the subsequent boot. A breadcrumb is the last periodic sample, not a
crash backtrace. A power cut before the boot record is saved cannot be recorded.

## Dashboard/server deployment

Replace only `server/index.php` and `server/assets/dashboard.js` on the production
server with the versions from this commit. Preserve configuration, credentials,
`.htaccess`, database, and uploads. No database migration is needed. The existing
upload browser supports the diagnostic files. Reload the dashboard after deployment.

The new command buttons become enabled when fresh telemetry says the device supports
them, is online, and is in standby. The firmware independently rejects both commands
while sampling, a measurement task exists, an automatic resume/upload is pending,
a reboot is scheduled, or another OTA is active. Rejected commands are acknowledged;
they are not deferred to interrupt a later session. No automatic log transfers or
updates run during measurements.

## Remote logs

Click **Upload Device Logs**, then **Open Upload Browser** and open `diagnostics`.
The command sends restart history and RAM logs as `restart.json` in a timestamped
capture folder. In standby it also snapshots all available `log0.log` through
`log4.log` before uploading them under `diagnostics/boot_<counter>/`. SD snapshots
are temporary and source logs are preserved. Repeating this command during the same
boot replaces those five uploaded log files; timestamped summaries remain separate.
When SD storage is unavailable only the summary and RAM log are uploaded.
Check the command acknowledgement: `logs_uploaded_N_failed_0` is success;
`logs_uploaded_ram_only` means SD logs were unavailable. A partial failure may still
leave useful files in the browser; retry from standby. Logs and firmware download
are served through different routes; diagnostic downloads retain existing admin auth.

## Remote firmware update

1. Build with `python -m platformio run -e esp32` and obtain
   `.pio/build/esp32/firmware.bin`. Validate the image on a spare device first.
2. Upload the application binary to the production web root as
   `/firmware/albasqueeze-<version>.bin`. Keep versioned names immutable.
3. Compute `shasum -a 256 firmware.bin` on the same binary.
4. In standby, enter the `/firmware/...bin` path and SHA-256 in the dashboard and
   click **Install Firmware**. Monitor the command acknowledgement and telemetry
   `firmware_build`/restart history after reconnecting.

The file must be served directly by HTTPS on `dashboard.albasqueeze.com:443`, with a
valid certificate, HTTP 200 and a decimal Content-Length. Redirects, compressed/chunked
responses, nested paths, query strings and external URLs are rejected. Firmware files
are public downloads; they must contain no embedded credentials. No API key is sent
in the download request. The admin-authenticated backend signs the path and SHA-256
with the existing per-device HMAC command secret. The instrument verifies this signature
before executing the command and compares the downloaded SHA-256 before activating
the inactive application slot. The image must fit the actual inactive partition.
A truncated image, bad digest, failed flash write, 15-second receive stall, or ten-minute
total download deadline aborts the update without selecting the new slot or rebooting.
An accepted image is finalized by the ESP32 Update library and then a reboot is scheduled.
This is integrity verification, not an additional firmware signing key or automatic
application-health rollback; a valid but defective application still needs USB recovery.

## API examples

Existing authenticated command endpoint:
`POST /admin/devices/<device_id>/commands` with `X-ADMIN-TOKEN`.

```json
{"action":"upload_logs"}
```

```json
{"action":"ota_update","params":"{\"path\":\"/firmware/albasqueeze-v2.bin\",\"sha256\":\"<64 hexadecimal characters>\"}"}
```

Never queue an update for a running field measurement. Keep the admin token and
per-device command secret private. A delivered successful OTA acknowledgement can
be lost to a network failure; confirm the reported build after reconnection rather
than blindly repeating an update.
