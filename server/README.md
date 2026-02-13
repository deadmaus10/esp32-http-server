# ESP32 Remote Backend (cPanel Ready)

This backend is designed for your current firmware remote-control flow:

- Device pushes telemetry to `serverUrl` (for example `https://playground.martinfuri.hu/remote`)
- Device polls commands from `GET /api/v1/devices/{device_id}/commands`
- Device ACKs command result to `POST /api/v1/devices/{device_id}/commands/{command_id}/ack`
- Device authenticates with `X-API-KEY`
- Command signatures are generated with `HMAC-SHA256(cmd_secret, canonical_payload)`

Implementation stack: plain PHP + SQLite (no Composer required).

## 1. Deploy to cPanel

1. Upload the contents of this folder to:
   - `public_html/remote`
2. Confirm PHP 8.1+ and SQLite extension are enabled in cPanel.
3. Ensure these paths are writable by PHP:
   - `public_html/remote/storage/`
   - `public_html/remote/storage/uploads/`
4. Keep `public_html/remote/.htaccess` as provided (routes all requests to `index.php`).

## 2. Configure credentials

Edit `config.php`:

- Set a real admin token:
  - `admin_token => 'long-random-secret'`
  - Dashboard accepts this token via `X-ADMIN-TOKEN`, `X-API-KEY`, or `Authorization: Bearer <token>`
- Configure dashboard defaults:
  - `dashboard_device_id => 'tank-node-01'` (or leave empty to use first configured device)
  - `offline_after_sec => 180`
  - `dashboard_poll_sec => 5`
  - `dashboard_title => 'Device Remote Control'`
- Add your device entry in `devices`:
  - key: exact firmware `device_id`
  - `api_key`: must match firmware `apiKey`
  - `cmd_secret`: must match firmware `cmdSecret`

Example:

```php
'devices' => [
    'tank-node-01' => [
        'api_key' => 'k_live_123456',
        'cmd_secret' => '8fd13f1f1f1f1f1f1f1f1f1f1f1f1f1f',
    ],
],
```

## 3. Firmware settings

Set these in your device config:

- `serverUrl`: one of:
  - `https://playground.martinfuri.hu/remote`
  - `https://playground.martinfuri.hu/remote/`
  - `https://playground.martinfuri.hu/remote/index.php`
  - `https://playground.martinfuri.hu/remote/ingest`
- `apiKey`: same as backend `api_key`
- `deviceId`: same as backend device key
- `cmdSecret`: same as backend `cmd_secret`
- `remoteEnabled`: `true`

### Important path behavior

- New firmware (with base-path-aware remote URL builder) uses:
  - `https://playground.martinfuri.hu/remote/api/v1/...`
- Legacy firmware may call:
  - `https://playground.martinfuri.hu/api/v1/...`

If you run legacy firmware, use `deploy/root-htaccess-legacy.sample` in `public_html/.htaccess`.

## 4. Verify backend is up

```bash
curl -sS https://playground.martinfuri.hu/remote/health
```

Expected:

```json
{"ok":true,"time":"...","db":"remote.sqlite"}
```

## 5. Open customer dashboard

Dashboard URL:

- `https://playground.martinfuri.hu/remote/dashboard`

Customer flow:

1. Open dashboard URL.
2. Enter `admin_token` from backend config (paste token value only, without surrounding quotes).
3. Use on-screen buttons:
   - Start Measurement
   - Stop Measurement
   - Reboot Device
4. Monitor:
   - Online/offline health
   - Last telemetry payload
   - Recent command ACK history

## 6. Queue remote commands (from your home PC)

All admin endpoints require `X-ADMIN-TOKEN`.

### Queue `measure_start`

```bash
curl -sS -X POST \
  https://playground.martinfuri.hu/remote/admin/devices/tank-node-01/commands \
  -H 'Content-Type: application/json' \
  -H 'X-ADMIN-TOKEN: long-random-secret' \
  -d '{"action":"measure_start","params":"rate=920"}'
```

### Queue `measure_stop`

```bash
curl -sS -X POST \
  https://playground.martinfuri.hu/remote/admin/devices/tank-node-01/commands \
  -H 'Content-Type: application/json' \
  -H 'X-ADMIN-TOKEN: long-random-secret' \
  -d '{"action":"measure_stop"}'
```

### Queue `reboot`

```bash
curl -sS -X POST \
  https://playground.martinfuri.hu/remote/admin/devices/tank-node-01/commands \
  -H 'Content-Type: application/json' \
  -H 'X-ADMIN-TOKEN: long-random-secret' \
  -d '{"action":"reboot"}'
```

## 7. Monitor command execution + telemetry

### List configured devices

```bash
curl -sS \
  https://playground.martinfuri.hu/remote/admin/devices \
  -H 'X-ADMIN-TOKEN: long-random-secret'
```

### List recent commands and ACK status

```bash
curl -sS \
  'https://playground.martinfuri.hu/remote/admin/devices/tank-node-01/commands?limit=20' \
  -H 'X-ADMIN-TOKEN: long-random-secret'
```

### Get recent telemetry samples

```bash
curl -sS \
  'https://playground.martinfuri.hu/remote/admin/devices/tank-node-01/telemetry?limit=10' \
  -H 'X-ADMIN-TOKEN: long-random-secret'
```

### Dashboard JSON snapshot used by UI

```bash
curl -sS \
  'https://playground.martinfuri.hu/remote/admin/devices/tank-node-01/dashboard' \
  -H 'X-ADMIN-TOKEN: long-random-secret'
```

## 8. Device-facing API summary

### Telemetry ingest

- `POST /ingest`
- Auth: `X-API-KEY`
- Body: JSON telemetry payload from firmware
- Response: `202` with `{"ok":true}`

### File upload (measurement logs)

- `POST /ingest?upload=1&name=<filename>`
- Auth: `X-API-KEY`
- Body: `application/octet-stream`
- Response: `201` with stored metadata

### Command poll

- `GET /api/v1/devices/{device_id}/commands?after_id=<id>&limit=1`
- Auth: `X-API-KEY`
- Response:
  - `200` + command JSON
  - `204` if no command

### Command ACK

- `POST /api/v1/devices/{device_id}/commands/{command_id}/ack`
- Auth: `X-API-KEY`
- Body: ACK payload from firmware (`ok`, `result`, `time`, `status`)
- Response: `200` with `{"ok":true}`

## 9. Dashboard/admin API summary

### Dashboard page

- `GET /dashboard`
- Returns HTML UI shell for remote customer control.

### Dashboard data

- `GET /admin/devices/{device_id}/dashboard`
- Auth: `X-ADMIN-TOKEN` or `Authorization: Bearer ...`
- Response includes:
  - `health` (`online`, `last_seen`, `last_seen_age_sec`, `pending_commands`, `measurement_active`)
  - `latest_telemetry`
  - `recent_commands`

## 10. Optional CLI helper (server-side)

If you have shell access on the server:

```bash
php scripts/enqueue_command.php --device tank-node-01 --action measure_start --params 'rate=920'
```

## 11. Production hardening checklist

- Replace `admin_token` with a long random value.
- Use high-entropy `api_key` and `cmd_secret` per device.
- Limit access to `/admin/*` (IP allow-list in cPanel or additional reverse proxy auth).
- Back up `storage/remote.sqlite` regularly.
- Enable HTTPS only.
