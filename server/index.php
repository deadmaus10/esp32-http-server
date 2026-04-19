<?php
declare(strict_types=1);

$config = require __DIR__ . '/config.php';
$localConfigFile = __DIR__ . '/config.local.php';
if (is_file($localConfigFile)) {
    $localConfig = require $localConfigFile;
    if (is_array($localConfig)) {
        $config = array_replace_recursive($config, $localConfig);
    }
}

date_default_timezone_set((string)($config['timezone'] ?? 'UTC'));

$devices = is_array($config['devices'] ?? null) ? $config['devices'] : [];
$basePath = trim((string)($config['base_path'] ?? ''), '/');
$adminToken = (string)($config['admin_token'] ?? '');
$dbPath = (string)($config['db_path'] ?? (__DIR__ . '/storage/remote.sqlite'));
$uploadDir = (string)($config['upload_dir'] ?? (__DIR__ . '/storage/uploads'));
$dashboardTitle = trim((string)($config['dashboard_title'] ?? 'Device Remote Control'));
if ($dashboardTitle === '') {
    $dashboardTitle = 'Device Remote Control';
}
$offlineAfterSec = (int)($config['offline_after_sec'] ?? 180);
if ($offlineAfterSec < 10) {
    $offlineAfterSec = 10;
}
$dashboardPollSec = (int)($config['dashboard_poll_sec'] ?? 5);
if ($dashboardPollSec < 2) {
    $dashboardPollSec = 2;
}
$dashboardDeviceId = resolveDashboardDeviceId(
    trim((string)($config['dashboard_device_id'] ?? '')),
    $devices
);
$dashboardDeviceIds = array_values(array_map('strval', array_keys($devices)));

ensureDirectory(dirname($dbPath));
ensureDirectory($uploadDir);

$pdo = openDatabase($dbPath);
initDatabase($pdo);

$method = strtoupper((string)($_SERVER['REQUEST_METHOD'] ?? 'GET'));
$rawPath = (string)(parse_url((string)($_SERVER['REQUEST_URI'] ?? '/'), PHP_URL_PATH) ?? '/');
if ($rawPath === '') {
    $rawPath = '/';
}
$pathCandidates = buildPathCandidates($rawPath, $basePath);

if ($method === 'OPTIONS') {
    respondJson(204, ['ok' => true]);
}

if ($method === 'GET' && pathEquals($pathCandidates, '/')) {
    respondJson(200, [
        'ok' => true,
        'service' => 'esp32-remote-backend',
        'time' => gmdate('c'),
    ]);
}

if ($method === 'GET' && pathEquals($pathCandidates, '/health')) {
    respondJson(200, [
        'ok' => true,
        'time' => gmdate('c'),
        'db' => basename($dbPath),
    ]);
}

if ($method === 'GET' && pathEquals($pathCandidates, '/dashboard')) {
    $requestedDeviceId = trim((string)($_GET['device_id'] ?? ''));
    $activeDeviceId = resolveActiveDeviceId($requestedDeviceId, $dashboardDeviceId, $devices);
    $bootstrap = [
        'basePath' => ($basePath === '') ? '' : ('/' . $basePath),
        'dashboardTitle' => $dashboardTitle,
        'deviceId' => $activeDeviceId,
        'deviceIds' => $dashboardDeviceIds,
        'pollSec' => $dashboardPollSec,
        'offlineAfterSec' => $offlineAfterSec,
        'apiDashboardUrlTemplate' => publicPath($basePath, '/admin/devices/{device_id}/dashboard'),
        'apiCommandsUrlTemplate' => publicPath($basePath, '/admin/devices/{device_id}/commands'),
        'uploadsUrl' => publicPath($basePath, '/uploads'),
    ];
    renderDashboardPage($bootstrap, publicPath($basePath, '/assets/dashboard.css'), publicPath($basePath, '/assets/dashboard.js'));
}

if ($method === 'GET' && pathEquals($pathCandidates, '/uploads')) {
    $requestedDeviceId = trim((string)($_GET['device_id'] ?? ''));
    $activeDeviceId = resolveActiveDeviceId($requestedDeviceId, $dashboardDeviceId, $devices);
    $bootstrap = [
        'basePath' => ($basePath === '') ? '' : ('/' . $basePath),
        'dashboardTitle' => $dashboardTitle,
        'deviceId' => $activeDeviceId,
        'deviceIds' => $dashboardDeviceIds,
        'apiUploadsUrlTemplate' => publicPath($basePath, '/admin/devices/{device_id}/uploads'),
        'dashboardUrl' => publicPath($basePath, '/dashboard'),
    ];
    renderUploadsPage($bootstrap, publicPath($basePath, '/assets/dashboard.css'), publicPath($basePath, '/assets/uploads.js'));
}

if ($method === 'POST' && pathEquals($pathCandidates, '/ingest')) {
    $isUpload = isset($_GET['upload']) && (string)$_GET['upload'] === '1';
    $hintDeviceId = resolveIngestDeviceIdHint($isUpload);
    $device = requireDeviceByApiKey($devices, requestHeader('X-API-KEY'), $hintDeviceId);
    handleIngestOrUpload($pdo, $uploadDir, (string)$device['device_id']);
}

if ($method === 'POST' && routeMatches($pathCandidates, '#^/api/v1/devices/([^/]+)/telemetry$#', $matches)) {
    $deviceId = urldecode($matches[1]);
    requireDeviceForPath($devices, $deviceId, requestHeader('X-API-KEY'));
    handleTelemetry($pdo, $deviceId);
}

if ($method === 'GET' && routeMatches($pathCandidates, '#^/api/v1/devices/([^/]+)/commands$#', $matches)) {
    $deviceId = urldecode($matches[1]);
    requireDeviceForPath($devices, $deviceId, requestHeader('X-API-KEY'));
    touchDeviceActivity($pdo, $deviceId, 'poll');

    $afterId = isset($_GET['after_id']) ? max(0, (int)$_GET['after_id']) : 0;
    $limit = isset($_GET['limit']) ? (int)$_GET['limit'] : 1;
    if ($limit < 1) {
        $limit = 1;
    }
    if ($limit > 10) {
        $limit = 10;
    }

    $stmt = $pdo->prepare(
        'SELECT id, action, params, issued_at, nonce, sig
         FROM commands
         WHERE device_id = :device_id AND id > :after_id AND acked_at IS NULL
         ORDER BY id ASC
         LIMIT :limit'
    );
    $stmt->bindValue(':device_id', $deviceId, PDO::PARAM_STR);
    $stmt->bindValue(':after_id', $afterId, PDO::PARAM_INT);
    $stmt->bindValue(':limit', $limit, PDO::PARAM_INT);
    $stmt->execute();
    $row = $stmt->fetch(PDO::FETCH_ASSOC);

    if (!$row) {
        respondNoContent();
    }

    $markStmt = $pdo->prepare('UPDATE commands SET delivered_at = COALESCE(delivered_at, :delivered_at) WHERE id = :id');
    $markStmt->execute([
        ':delivered_at' => gmdate('c'),
        ':id' => (int)$row['id'],
    ]);

    respondJson(200, [
        'id' => (string)$row['id'],
        'action' => (string)$row['action'],
        'params' => (string)$row['params'],
        'issued_at' => (string)$row['issued_at'],
        'nonce' => (string)$row['nonce'],
        'sig' => (string)$row['sig'],
    ]);
}

if ($method === 'POST' && routeMatches($pathCandidates, '#^/api/v1/devices/([^/]+)/commands/([^/]+)/ack$#', $matches)) {
    $deviceId = urldecode($matches[1]);
    $commandId = urldecode($matches[2]);
    requireDeviceForPath($devices, $deviceId, requestHeader('X-API-KEY'));
    touchDeviceActivity($pdo, $deviceId, 'ack');

    if (!ctype_digit($commandId)) {
        respondJson(400, ['ok' => false, 'error' => 'invalid_command_id']);
    }

    $rawBody = requestBody();
    $ackPayload = decodeJsonBody($rawBody);
    if (!is_array($ackPayload)) {
        respondJson(400, ['ok' => false, 'error' => 'ack_json_must_be_object']);
    }

    $existsStmt = $pdo->prepare('SELECT id FROM commands WHERE id = :id AND device_id = :device_id LIMIT 1');
    $existsStmt->execute([
        ':id' => (int)$commandId,
        ':device_id' => $deviceId,
    ]);
    if (!$existsStmt->fetch(PDO::FETCH_ASSOC)) {
        respondJson(404, ['ok' => false, 'error' => 'command_not_found']);
    }

    $ackOk = !empty($ackPayload['ok']) ? 1 : 0;
    $ackResult = isset($ackPayload['result']) ? (string)$ackPayload['result'] : '';
    $ackJson = json_encode($ackPayload, JSON_UNESCAPED_SLASHES | JSON_UNESCAPED_UNICODE);
    if (!is_string($ackJson)) {
        $ackJson = '{}';
    }

    $ackStmt = $pdo->prepare(
        'UPDATE commands
         SET acked_at = :acked_at,
             ack_ok = :ack_ok,
             ack_result = :ack_result,
             ack_payload_json = :ack_payload_json
         WHERE id = :id AND device_id = :device_id'
    );
    $ackStmt->execute([
        ':acked_at' => gmdate('c'),
        ':ack_ok' => $ackOk,
        ':ack_result' => $ackResult,
        ':ack_payload_json' => $ackJson,
        ':id' => (int)$commandId,
        ':device_id' => $deviceId,
    ]);

    respondJson(200, ['ok' => true]);
}

if (pathStartsWith($pathCandidates, '/admin/')) {
    requireAdminToken($adminToken);

    if ($method === 'GET' && pathEquals($pathCandidates, '/admin/devices')) {
        $rows = [];
        $pendingStmt = $pdo->prepare('SELECT COUNT(*) AS c FROM commands WHERE device_id = :device_id AND acked_at IS NULL');
        $lastSeenStmt = $pdo->prepare('SELECT received_at FROM telemetry WHERE device_id = :device_id ORDER BY id DESC LIMIT 1');

        foreach ($devices as $deviceId => $deviceCfg) {
            $pendingStmt->execute([':device_id' => $deviceId]);
            $pending = (int)$pendingStmt->fetchColumn();

            $lastSeenStmt->execute([':device_id' => $deviceId]);
            $lastSeen = $lastSeenStmt->fetchColumn();

            $rows[] = [
                'device_id' => $deviceId,
                'pending_commands' => $pending,
                'last_seen' => is_string($lastSeen) ? $lastSeen : null,
            ];
        }

        respondJson(200, ['ok' => true, 'devices' => $rows]);
    }

    if ($method === 'GET' && routeMatches($pathCandidates, '#^/admin/devices/([^/]+)/dashboard$#', $matches)) {
        $deviceId = urldecode($matches[1]);
        if (!isset($devices[$deviceId])) {
            respondJson(404, ['ok' => false, 'error' => 'unknown_device']);
        }

        $commandLimit = isset($_GET['command_limit']) ? (int)$_GET['command_limit'] : 20;
        if ($commandLimit < 5) {
            $commandLimit = 5;
        }
        if ($commandLimit > 100) {
            $commandLimit = 100;
        }

        $commandBeforeId = null;
        if (array_key_exists('command_before_id', $_GET)) {
            $rawBefore = trim((string)$_GET['command_before_id']);
            if ($rawBefore !== '') {
                if (!ctype_digit($rawBefore)) {
                    respondJson(400, ['ok' => false, 'error' => 'invalid_command_before_id']);
                }
                $beforeValue = (int)$rawBefore;
                if ($beforeValue > 0) {
                    $commandBeforeId = $beforeValue;
                }
            }
        }

        $snapshot = loadDashboardSnapshot($pdo, $basePath, $deviceId, $offlineAfterSec, $commandLimit, $commandBeforeId);
        respondJson(200, ['ok' => true] + $snapshot);
    }

    if ($method === 'POST' && routeMatches($pathCandidates, '#^/admin/devices/([^/]+)/commands$#', $matches)) {
        $deviceId = urldecode($matches[1]);
        if (!isset($devices[$deviceId])) {
            respondJson(404, ['ok' => false, 'error' => 'unknown_device']);
        }

        $cmdSecret = (string)($devices[$deviceId]['cmd_secret'] ?? '');
        if ($cmdSecret === '') {
            respondJson(500, ['ok' => false, 'error' => 'device_missing_cmd_secret']);
        }

        $payload = decodeJsonBody(requestBody());
        if (!is_array($payload)) {
            respondJson(400, ['ok' => false, 'error' => 'json_body_required']);
        }

        $action = trim((string)($payload['action'] ?? ''));
        $allowedActions = ['measure_start', 'measure_stop', 'reboot'];
        if (!in_array($action, $allowedActions, true)) {
            respondJson(400, ['ok' => false, 'error' => 'unsupported_action']);
        }

        $params = isset($payload['params']) ? trim((string)$payload['params']) : '';
        $issuedAt = isset($payload['issued_at']) ? trim((string)$payload['issued_at']) : gmdate('c');
        $nonce = isset($payload['nonce']) ? trim((string)$payload['nonce']) : '';
        if ($nonce === '') {
            $nonce = bin2hex(random_bytes(12));
        }

        $createdAt = gmdate('c');

        $pdo->beginTransaction();
        try {
            $insertStmt = $pdo->prepare(
                'INSERT INTO commands (device_id, action, params, issued_at, nonce, sig, created_at)
                 VALUES (:device_id, :action, :params, :issued_at, :nonce, :sig, :created_at)'
            );
            $insertStmt->execute([
                ':device_id' => $deviceId,
                ':action' => $action,
                ':params' => $params,
                ':issued_at' => $issuedAt,
                ':nonce' => $nonce,
                ':sig' => '',
                ':created_at' => $createdAt,
            ]);

            $commandId = (string)$pdo->lastInsertId();
            $canonical = canonicalCommandPayload($commandId, $action, $params, $issuedAt, $nonce);
            $sig = hash_hmac('sha256', $canonical, $cmdSecret);

            $updateStmt = $pdo->prepare('UPDATE commands SET sig = :sig WHERE id = :id');
            $updateStmt->execute([
                ':sig' => $sig,
                ':id' => (int)$commandId,
            ]);

            $pdo->commit();
        } catch (Throwable $e) {
            if ($pdo->inTransaction()) {
                $pdo->rollBack();
            }
            respondJson(500, ['ok' => false, 'error' => 'failed_to_queue_command']);
        }

        respondJson(201, [
            'ok' => true,
            'command' => [
                'id' => $commandId,
                'action' => $action,
                'params' => $params,
                'issued_at' => $issuedAt,
                'nonce' => $nonce,
                'sig' => $sig,
            ],
        ]);
    }

    if ($method === 'GET' && routeMatches($pathCandidates, '#^/admin/devices/([^/]+)/commands$#', $matches)) {
        $deviceId = urldecode($matches[1]);
        if (!isset($devices[$deviceId])) {
            respondJson(404, ['ok' => false, 'error' => 'unknown_device']);
        }

        $limit = isset($_GET['limit']) ? (int)$_GET['limit'] : 50;
        if ($limit < 1) {
            $limit = 1;
        }
        if ($limit > 200) {
            $limit = 200;
        }

        $stmt = $pdo->prepare(
            'SELECT id, action, params, issued_at, nonce, sig, created_at, delivered_at, acked_at, ack_ok, ack_result
             FROM commands
             WHERE device_id = :device_id
             ORDER BY id DESC
             LIMIT :limit'
        );
        $stmt->bindValue(':device_id', $deviceId, PDO::PARAM_STR);
        $stmt->bindValue(':limit', $limit, PDO::PARAM_INT);
        $stmt->execute();

        $rows = [];
        while ($row = $stmt->fetch(PDO::FETCH_ASSOC)) {
            $rows[] = adminCommandRow($basePath, $deviceId, $row);
        }

        respondJson(200, ['ok' => true, 'commands' => $rows]);
    }

    if ($method === 'POST' && routeMatches($pathCandidates, '#^/admin/devices/([^/]+)/commands/reset$#', $matches)) {
        $deviceId = urldecode($matches[1]);
        if (!isset($devices[$deviceId])) {
            respondJson(404, ['ok' => false, 'error' => 'unknown_device']);
        }

        $rawBody = requestBody();
        if (trim($rawBody) === '') {
            $payload = [];
        } else {
            $payload = decodeJsonBody($rawBody);
            if (!is_array($payload)) {
                respondJson(400, ['ok' => false, 'error' => 'json_body_required']);
            }
        }

        $mode = strtolower(trim((string)($payload['mode'] ?? 'acked')));
        if ($mode !== 'acked' && $mode !== 'all') {
            respondJson(400, ['ok' => false, 'error' => 'invalid_mode']);
        }

        $beforeId = null;
        if (array_key_exists('before_id', $payload)) {
            $beforeRaw = trim((string)$payload['before_id']);
            if ($beforeRaw !== '') {
                if (!ctype_digit($beforeRaw)) {
                    respondJson(400, ['ok' => false, 'error' => 'invalid_before_id']);
                }
                $beforeParsed = (int)$beforeRaw;
                if ($beforeParsed > 0) {
                    $beforeId = $beforeParsed;
                }
            }
        }

        if ($mode === 'acked') {
            if (is_int($beforeId)) {
                $deleteStmt = $pdo->prepare(
                    'DELETE FROM commands
                     WHERE device_id = :device_id
                       AND acked_at IS NOT NULL
                       AND id <= :before_id'
                );
                $deleteStmt->bindValue(':device_id', $deviceId, PDO::PARAM_STR);
                $deleteStmt->bindValue(':before_id', $beforeId, PDO::PARAM_INT);
                $deleteStmt->execute();
            } else {
                $deleteStmt = $pdo->prepare(
                    'DELETE FROM commands
                     WHERE device_id = :device_id
                       AND acked_at IS NOT NULL'
                );
                $deleteStmt->execute([':device_id' => $deviceId]);
            }
        } else {
            if (is_int($beforeId)) {
                $deleteStmt = $pdo->prepare(
                    'DELETE FROM commands
                     WHERE device_id = :device_id
                       AND id <= :before_id'
                );
                $deleteStmt->bindValue(':device_id', $deviceId, PDO::PARAM_STR);
                $deleteStmt->bindValue(':before_id', $beforeId, PDO::PARAM_INT);
                $deleteStmt->execute();
            } else {
                $deleteStmt = $pdo->prepare(
                    'DELETE FROM commands
                     WHERE device_id = :device_id'
                );
                $deleteStmt->execute([':device_id' => $deviceId]);
            }
        }

        respondJson(200, [
            'ok' => true,
            'device_id' => $deviceId,
            'mode' => $mode,
            'before_id' => is_int($beforeId) ? (string)$beforeId : null,
            'deleted' => $deleteStmt->rowCount(),
        ]);
    }

    if ($method === 'POST' && routeMatches($pathCandidates, '#^/admin/devices/([^/]+)/commands/([^/]+)/delete$#', $matches)) {
        $deviceId = urldecode($matches[1]);
        if (!isset($devices[$deviceId])) {
            respondJson(404, ['ok' => false, 'error' => 'unknown_device']);
        }

        $commandIdRaw = trim(urldecode($matches[2]));
        if ($commandIdRaw === '' || !ctype_digit($commandIdRaw)) {
            respondJson(400, ['ok' => false, 'error' => 'invalid_command_id']);
        }

        $commandId = (int)$commandIdRaw;
        if ($commandId <= 0) {
            respondJson(400, ['ok' => false, 'error' => 'invalid_command_id']);
        }

        $findStmt = $pdo->prepare(
            'SELECT id, acked_at
             FROM commands
             WHERE id = :id
               AND device_id = :device_id
             LIMIT 1'
        );
        $findStmt->execute([
            ':id' => $commandId,
            ':device_id' => $deviceId,
        ]);
        $commandRow = $findStmt->fetch(PDO::FETCH_ASSOC);
        if (!is_array($commandRow)) {
            respondJson(404, ['ok' => false, 'error' => 'command_not_found']);
        }

        if (!commandRowIsPending($commandRow['acked_at'] ?? null)) {
            respondJson(409, ['ok' => false, 'error' => 'command_not_pending']);
        }

        $deleteStmt = $pdo->prepare(
            'DELETE FROM commands
             WHERE id = :id
               AND device_id = :device_id
               AND acked_at IS NULL'
        );
        $deleteStmt->execute([
            ':id' => $commandId,
            ':device_id' => $deviceId,
        ]);
        if ($deleteStmt->rowCount() < 1) {
            respondJson(409, ['ok' => false, 'error' => 'command_not_pending']);
        }

        respondJson(200, [
            'ok' => true,
            'device_id' => $deviceId,
            'command_id' => (string)$commandId,
            'deleted' => true,
        ]);
    }

    if ($method === 'GET' && routeMatches($pathCandidates, '#^/admin/devices/([^/]+)/uploads$#', $matches)) {
        $deviceId = urldecode($matches[1]);
        if (!isset($devices[$deviceId])) {
            respondJson(404, ['ok' => false, 'error' => 'unknown_device']);
        }

        $currentPath = sanitizeUploadRelativePath((string)($_GET['path'] ?? ''));
        $entries = loadUploadsForDevice($pdo, $deviceId, $uploadDir);
        $listing = buildUploadsListing($entries, $deviceId, $currentPath, $basePath);

        respondJson(200, [
            'ok' => true,
            'device_id' => $deviceId,
            'path' => $currentPath,
            'parent_path' => parentUploadPath($currentPath),
            'folders' => $listing['folders'],
            'files' => $listing['files'],
            // Backward-compatible alias kept for older clients.
            'uploads' => $listing['files'],
        ]);
    }

    if ($method === 'GET' && routeMatches($pathCandidates, '#^/admin/devices/([^/]+)/uploads/folder-download$#', $matches)) {
        $deviceId = urldecode($matches[1]);
        if (!isset($devices[$deviceId])) {
            respondJson(404, ['ok' => false, 'error' => 'unknown_device']);
        }

        if (!class_exists('ZipArchive')) {
            respondJson(500, ['ok' => false, 'error' => 'zip_extension_missing']);
        }

        $folderPath = sanitizeUploadRelativePath((string)($_GET['path'] ?? ''));
        if ($folderPath === '') {
            respondJson(400, ['ok' => false, 'error' => 'missing_folder_path']);
        }

        $entries = loadUploadsForDevice($pdo, $deviceId, $uploadDir);
        $prefix = $folderPath . '/';
        $matchesRows = [];
        foreach ($entries as $entry) {
            $relPath = (string)($entry['relative_path'] ?? '');
            if ($relPath === '' || !str_starts_with($relPath, $prefix)) {
                continue;
            }
            $storedPath = (string)($entry['stored_path_real'] ?? '');
            if ($storedPath === '' || !is_file($storedPath)) {
                continue;
            }
            $matchesRows[] = $entry;
        }

        if (count($matchesRows) === 0) {
            respondJson(404, ['ok' => false, 'error' => 'folder_has_no_files']);
        }

        $tmpBase = tempnam(sys_get_temp_dir(), 'uplzip_');
        if (!is_string($tmpBase) || $tmpBase === '') {
            respondJson(500, ['ok' => false, 'error' => 'zip_temp_create_failed']);
        }
        $tmpZip = $tmpBase . '.zip';
        @unlink($tmpZip);
        if (!@rename($tmpBase, $tmpZip)) {
            @unlink($tmpBase);
            respondJson(500, ['ok' => false, 'error' => 'zip_temp_prepare_failed']);
        }

        $zip = new ZipArchive();
        $opened = $zip->open($tmpZip, ZipArchive::CREATE | ZipArchive::OVERWRITE);
        if ($opened !== true) {
            @unlink($tmpZip);
            respondJson(500, ['ok' => false, 'error' => 'zip_open_failed']);
        }

        foreach ($matchesRows as $entry) {
            $storedPath = (string)($entry['stored_path_real'] ?? '');
            $relPath = (string)($entry['relative_path'] ?? '');
            if ($storedPath === '' || $relPath === '') {
                continue;
            }

            $archiveName = $relPath;
            if (str_starts_with($archiveName, $prefix)) {
                $archiveName = substr($archiveName, strlen($prefix));
            }
            $archiveName = ltrim(str_replace('\\', '/', $archiveName), '/');
            if ($archiveName === '') {
                continue;
            }
            $zip->addFile($storedPath, $archiveName);
        }

        $zip->close();

        $zipName = sanitizeFileName(str_replace('/', '_', $folderPath) . '.zip');
        if ($zipName === '') {
            $zipName = 'measurement_bundle.zip';
        }

        $fh = fopen($tmpZip, 'rb');
        if ($fh === false) {
            @unlink($tmpZip);
            respondJson(500, ['ok' => false, 'error' => 'zip_open_stream_failed']);
        }

        $size = filesize($tmpZip);
        if ($size === false) {
            fclose($fh);
            @unlink($tmpZip);
            respondJson(500, ['ok' => false, 'error' => 'zip_size_failed']);
        }

        http_response_code(200);
        header('Content-Type: application/zip');
        header('Content-Length: ' . (string)$size);
        header('Content-Disposition: attachment; filename="' . $zipName . '"');
        header('Cache-Control: no-store');
        fpassthru($fh);
        fclose($fh);
        @unlink($tmpZip);
        exit;
    }

    if ($method === 'POST' && routeMatches($pathCandidates, '#^/admin/devices/([^/]+)/uploads/([^/]+)/delete$#', $matches)) {
        $deviceId = urldecode($matches[1]);
        $uploadId = urldecode($matches[2]);
        if (!isset($devices[$deviceId])) {
            respondJson(404, ['ok' => false, 'error' => 'unknown_device']);
        }
        if (!ctype_digit($uploadId)) {
            respondJson(400, ['ok' => false, 'error' => 'invalid_upload_id']);
        }

        $stmt = $pdo->prepare(
            'SELECT id, filename, stored_path
             FROM uploads
             WHERE device_id = :device_id
               AND id = :id
             LIMIT 1'
        );
        $stmt->execute([
            ':device_id' => $deviceId,
            ':id' => (int)$uploadId,
        ]);
        $row = $stmt->fetch(PDO::FETCH_ASSOC);
        if (!is_array($row)) {
            respondJson(404, ['ok' => false, 'error' => 'upload_not_found']);
        }

        $storedPath = (string)($row['stored_path'] ?? '');
        $storedPathReal = resolveUploadStoredPath($storedPath, $uploadDir);

        $sharedStmt = $pdo->prepare(
            'SELECT COUNT(*) FROM uploads WHERE device_id = :device_id AND stored_path = :stored_path AND id <> :id'
        );
        $sharedStmt->execute([
            ':device_id' => $deviceId,
            ':stored_path' => $storedPath,
            ':id' => (int)$uploadId,
        ]);
        $sharedCount = (int)$sharedStmt->fetchColumn();

        $deleteStmt = $pdo->prepare(
            'DELETE FROM uploads
             WHERE device_id = :device_id
               AND id = :id'
        );
        $deleteStmt->execute([
            ':device_id' => $deviceId,
            ':id' => (int)$uploadId,
        ]);
        if ($deleteStmt->rowCount() < 1) {
            respondJson(404, ['ok' => false, 'error' => 'upload_not_found']);
        }

        $fileExisted = false;
        $fileRemoved = false;
        if (is_string($storedPathReal) && is_file($storedPathReal)) {
            $fileExisted = true;
            if ($sharedCount <= 0) {
                $fileRemoved = @unlink($storedPathReal);
                if (!$fileRemoved) {
                    respondJson(500, ['ok' => false, 'error' => 'upload_file_delete_failed']);
                }
            } else {
                $fileRemoved = false;
            }
        }

        respondJson(200, [
            'ok' => true,
            'device_id' => $deviceId,
            'upload_id' => (string)$uploadId,
            'deleted' => true,
            'file_existed' => $fileExisted,
            'file_removed' => $fileExisted ? $fileRemoved : false,
        ]);
    }

    if ($method === 'GET' && routeMatches($pathCandidates, '#^/admin/devices/([^/]+)/uploads/([^/]+)/download$#', $matches)) {
        $deviceId = urldecode($matches[1]);
        $uploadId = urldecode($matches[2]);
        if (!isset($devices[$deviceId])) {
            respondJson(404, ['ok' => false, 'error' => 'unknown_device']);
        }
        if (!ctype_digit($uploadId)) {
            respondJson(400, ['ok' => false, 'error' => 'invalid_upload_id']);
        }

        $stmt = $pdo->prepare(
            'SELECT id, filename, stored_path, bytes
             FROM uploads
             WHERE device_id = :device_id
               AND id = :id
             LIMIT 1'
        );
        $stmt->execute([
            ':device_id' => $deviceId,
            ':id' => (int)$uploadId,
        ]);
        $row = $stmt->fetch(PDO::FETCH_ASSOC);
        if (!is_array($row)) {
            respondJson(404, ['ok' => false, 'error' => 'upload_not_found']);
        }

        $storedPath = (string)($row['stored_path'] ?? '');
        $storedPathReal = resolveUploadStoredPath($storedPath, $uploadDir);
        if (!is_string($storedPathReal) || !is_file($storedPathReal)) {
            markUploadStorageStatus($pdo, (int)$uploadId, 'missing');
            respondJson(404, ['ok' => false, 'error' => 'upload_file_missing']);
        }

        $relativePath = uploadRelativePathFromRow($row, $deviceId, $uploadDir);
        $downloadName = sanitizeFileName(basename($relativePath));
        if ($downloadName === '') {
            $downloadName = sanitizeFileName((string)($row['filename'] ?? ''));
        }
        if ($downloadName === '') {
            $downloadName = 'upload_' . (string)$row['id'] . '.bin';
        }

        $fileSize = filesize($storedPathReal);
        if ($fileSize === false) {
            respondJson(500, ['ok' => false, 'error' => 'upload_filesize_failed']);
        }

        $fh = fopen($storedPathReal, 'rb');
        if ($fh === false) {
            respondJson(500, ['ok' => false, 'error' => 'upload_open_failed']);
        }

        http_response_code(200);
        header('Content-Type: application/octet-stream');
        header('Content-Length: ' . (string)$fileSize);
        header('Content-Disposition: attachment; filename="' . $downloadName . '"');
        header('Cache-Control: no-store');
        fpassthru($fh);
        fclose($fh);
        exit;
    }

    if ($method === 'GET' && routeMatches($pathCandidates, '#^/admin/devices/([^/]+)/telemetry$#', $matches)) {
        $deviceId = urldecode($matches[1]);
        if (!isset($devices[$deviceId])) {
            respondJson(404, ['ok' => false, 'error' => 'unknown_device']);
        }

        $limit = isset($_GET['limit']) ? (int)$_GET['limit'] : 20;
        if ($limit < 1) {
            $limit = 1;
        }
        if ($limit > 200) {
            $limit = 200;
        }

        $stmt = $pdo->prepare(
            'SELECT id, received_at, payload_json
             FROM telemetry
             WHERE device_id = :device_id
             ORDER BY id DESC
             LIMIT :limit'
        );
        $stmt->bindValue(':device_id', $deviceId, PDO::PARAM_STR);
        $stmt->bindValue(':limit', $limit, PDO::PARAM_INT);
        $stmt->execute();

        $rows = [];
        while ($row = $stmt->fetch(PDO::FETCH_ASSOC)) {
            $payload = json_decode((string)$row['payload_json'], true);
            $rows[] = [
                'id' => (string)$row['id'],
                'received_at' => (string)$row['received_at'],
                'payload' => is_array($payload) ? $payload : null,
            ];
        }

        respondJson(200, ['ok' => true, 'telemetry' => $rows]);
    }

    respondJson(404, ['ok' => false, 'error' => 'admin_route_not_found']);
}

respondJson(404, ['ok' => false, 'error' => 'route_not_found']);

function handleIngestOrUpload(PDO $pdo, string $uploadDir, string $deviceId): void
{
    $isUpload = isset($_GET['upload']) && (string)$_GET['upload'] === '1';
    if ($isUpload) {
        handleUpload($pdo, $uploadDir, $deviceId);
    }
    handleTelemetry($pdo, $deviceId);
}

function handleUpload(PDO $pdo, string $uploadDir, string $deviceId): void
{
    $rawBody = requestBody();
    $rawName = (string)($_GET['name'] ?? 'upload.bin');
    $parsed = parseUploadNameParts($rawName);
    $relativePath = $parsed['relative_path'];
    if ($relativePath === '') {
        respondJson(400, ['ok' => false, 'error' => 'missing_upload_name']);
    }

    $deviceDir = $uploadDir . '/' . sanitizeSegment($deviceId);
    ensureDirectory($deviceDir);

    $storedPath = $deviceDir . '/' . $relativePath;
    ensureDirectory(dirname($storedPath));

    $bytes = file_put_contents($storedPath, $rawBody, LOCK_EX);
    if ($bytes === false) {
        respondJson(500, ['ok' => false, 'error' => 'upload_store_failed']);
    }

    $receivedAt = gmdate('c');
    $sourceIp = clientIp();
    $existingStmt = $pdo->prepare(
        'SELECT id
         FROM uploads
         WHERE device_id = :device_id
           AND filename = :filename
         ORDER BY id DESC
         LIMIT 1'
    );
    $existingStmt->execute([
        ':device_id' => $deviceId,
        ':filename' => $relativePath,
    ]);
    $existing = $existingStmt->fetch(PDO::FETCH_ASSOC);

    if (is_array($existing) && isset($existing['id']) && ctype_digit((string)$existing['id'])) {
        $uploadId = (int)$existing['id'];
        $updateStmt = $pdo->prepare(
            'UPDATE uploads
             SET stored_path = :stored_path,
                 bytes = :bytes,
                 received_at = :received_at,
                 source_ip = :source_ip,
                 storage_status = :storage_status,
                 storage_checked_at = :storage_checked_at
             WHERE id = :id'
        );
        $updateStmt->execute([
            ':stored_path' => $storedPath,
            ':bytes' => (int)$bytes,
            ':received_at' => $receivedAt,
            ':source_ip' => $sourceIp,
            ':storage_status' => 'present',
            ':storage_checked_at' => $receivedAt,
            ':id' => $uploadId,
        ]);
    } else {
        $insertStmt = $pdo->prepare(
            'INSERT INTO uploads (device_id, filename, stored_path, bytes, received_at, source_ip, storage_status, storage_checked_at)
             VALUES (:device_id, :filename, :stored_path, :bytes, :received_at, :source_ip, :storage_status, :storage_checked_at)'
        );
        $insertStmt->execute([
            ':device_id' => $deviceId,
            ':filename' => $relativePath,
            ':stored_path' => $storedPath,
            ':bytes' => (int)$bytes,
            ':received_at' => $receivedAt,
            ':source_ip' => $sourceIp,
            ':storage_status' => 'present',
            ':storage_checked_at' => $receivedAt,
        ]);
    }

    respondJson(201, [
        'ok' => true,
        'stored' => $relativePath,
        'bytes' => (int)$bytes,
    ]);
}

function handleTelemetry(PDO $pdo, string $deviceId): void
{
    $rawBody = requestBody();
    if ($rawBody === '') {
        respondJson(400, ['ok' => false, 'error' => 'empty_body']);
    }

    $payload = decodeJsonBody($rawBody);
    if (!is_array($payload)) {
        respondJson(400, ['ok' => false, 'error' => 'telemetry_json_must_be_object']);
    }

    $stmt = $pdo->prepare(
        'INSERT INTO telemetry (device_id, received_at, payload_json, source_ip)
         VALUES (:device_id, :received_at, :payload_json, :source_ip)'
    );
    $stmt->execute([
        ':device_id' => $deviceId,
        ':received_at' => gmdate('c'),
        ':payload_json' => $rawBody,
        ':source_ip' => clientIp(),
    ]);
    touchDeviceActivity($pdo, $deviceId, 'telemetry');

    respondJson(202, ['ok' => true]);
}

function commandRowIsPending($ackedAt): bool
{
    return is_null($ackedAt) || (is_string($ackedAt) && trim($ackedAt) === '');
}

function adminCommandRow(string $basePath, string $deviceId, array $row): array
{
    $id = (string)($row['id'] ?? '');
    $ackedAt = $row['acked_at'] ?? null;
    $pending = commandRowIsPending($ackedAt);

    return [
        'id' => $id,
        'action' => (string)($row['action'] ?? ''),
        'params' => (string)($row['params'] ?? ''),
        'issued_at' => (string)($row['issued_at'] ?? ''),
        'nonce' => (string)($row['nonce'] ?? ''),
        'sig' => (string)($row['sig'] ?? ''),
        'created_at' => (string)($row['created_at'] ?? ''),
        'delivered_at' => $row['delivered_at'] ?? null,
        'acked_at' => $ackedAt,
        'ack_ok' => is_null($row['ack_ok'] ?? null) ? null : ((int)$row['ack_ok'] === 1),
        'ack_result' => is_null($row['ack_result'] ?? null) ? null : (string)$row['ack_result'],
        'can_delete' => $pending,
        'delete_url' => $pending && $id !== ''
            ? publicPath(
                $basePath,
                '/admin/devices/' . rawurlencode($deviceId) . '/commands/' . rawurlencode($id) . '/delete'
            )
            : null,
    ];
}

function loadDashboardSnapshot(PDO $pdo, string $basePath, string $deviceId, int $offlineAfterSec, int $commandLimit, ?int $commandBeforeId = null): array
{
    $nowTs = time();
    $nowIso = gmdate('c');

    $pendingStmt = $pdo->prepare('SELECT COUNT(*) FROM commands WHERE device_id = :device_id AND acked_at IS NULL');
    $pendingStmt->execute([':device_id' => $deviceId]);
    $pendingCommands = (int)$pendingStmt->fetchColumn();

    $telemetryStmt = $pdo->prepare(
        'SELECT id, received_at, payload_json
         FROM telemetry
         WHERE device_id = :device_id
         ORDER BY id DESC
         LIMIT 1'
    );
    $telemetryStmt->execute([':device_id' => $deviceId]);
    $telemetryRow = $telemetryStmt->fetch(PDO::FETCH_ASSOC);

    $lastSeen = null;
    $lastSeenSource = null;
    $latestTelemetry = null;
    $lastSeenAgeSec = null;
    $measurementActive = null;
    $expectedTelemetrySec = null;
    $adaptiveOfflineAfterSec = $offlineAfterSec;
    if (is_array($telemetryRow)) {
        $lastSeen = (string)$telemetryRow['received_at'];
        $lastSeenSource = 'telemetry';
        $payload = json_decode((string)$telemetryRow['payload_json'], true);
        $latestTelemetry = [
            'received_at' => $lastSeen,
            'payload' => is_array($payload) ? $payload : null,
        ];

        $lastSeenTs = parseIsoTimestamp($lastSeen);
        if (is_int($lastSeenTs)) {
            $lastSeenAgeSec = max(0, $nowTs - $lastSeenTs);
        }

        if (is_array($payload)) {
            $measurementActive = payloadBool($payload, ['meas', 'active']);
            $period = payloadInt($payload, ['cloud', 'period']);
            if (is_int($period) && $period > 0) {
                $expectedTelemetrySec = $period;
            }
        }
    }

    $activityStmt = $pdo->prepare(
        'SELECT last_seen_at, last_source
         FROM device_activity
         WHERE device_id = :device_id
         LIMIT 1'
    );
    $activityStmt->execute([':device_id' => $deviceId]);
    $activityRow = $activityStmt->fetch(PDO::FETCH_ASSOC);
    if (is_array($activityRow)) {
        $activitySeen = (string)($activityRow['last_seen_at'] ?? '');
        $activitySeenTs = parseIsoTimestamp($activitySeen);
        $currentLastSeenTs = is_string($lastSeen) ? parseIsoTimestamp($lastSeen) : null;
        if (is_int($activitySeenTs) && (!is_int($currentLastSeenTs) || $activitySeenTs > $currentLastSeenTs)) {
            $lastSeen = $activitySeen;
            $lastSeenSource = trim((string)($activityRow['last_source'] ?? 'activity'));
            if ($lastSeenSource === '') {
                $lastSeenSource = 'activity';
            }
            $lastSeenAgeSec = max(0, $nowTs - $activitySeenTs);
        }
    }

    $telemetryTimesStmt = $pdo->prepare(
        'SELECT received_at
         FROM telemetry
         WHERE device_id = :device_id
         ORDER BY id DESC
         LIMIT 6'
    );
    $telemetryTimesStmt->execute([':device_id' => $deviceId]);
    $telemetryTs = [];
    while ($row = $telemetryTimesStmt->fetch(PDO::FETCH_ASSOC)) {
        $ts = parseIsoTimestamp((string)($row['received_at'] ?? ''));
        if (is_int($ts)) {
            $telemetryTs[] = $ts;
        }
    }
    if (count($telemetryTs) >= 2) {
        $maxGapSec = 0;
        $count = count($telemetryTs);
        for ($i = 0; $i < $count - 1; $i++) {
            $gap = $telemetryTs[$i] - $telemetryTs[$i + 1];
            if ($gap > $maxGapSec) {
                $maxGapSec = $gap;
            }
        }
        if ($maxGapSec > 0) {
            $expectedTelemetrySec = is_int($expectedTelemetrySec)
                ? max($expectedTelemetrySec, $maxGapSec)
                : $maxGapSec;
        }
    }

    if (is_int($expectedTelemetrySec) && $expectedTelemetrySec > 0) {
        $adaptiveMin = (int)ceil($expectedTelemetrySec * 3.0);
        if ($adaptiveMin < 30) {
            $adaptiveMin = 30;
        }
        if ($adaptiveMin > 3600) {
            $adaptiveMin = 3600;
        }
        if ($adaptiveOfflineAfterSec < $adaptiveMin) {
            $adaptiveOfflineAfterSec = $adaptiveMin;
        }
    }

    $commandFetchLimit = $commandLimit + 1;
    if (is_int($commandBeforeId) && $commandBeforeId > 0) {
        $recentStmt = $pdo->prepare(
            'SELECT id, action, params, issued_at, created_at, delivered_at, acked_at, ack_ok, ack_result
             FROM commands
             WHERE device_id = :device_id
               AND id < :before_id
             ORDER BY id DESC
             LIMIT :limit'
        );
        $recentStmt->bindValue(':device_id', $deviceId, PDO::PARAM_STR);
        $recentStmt->bindValue(':before_id', $commandBeforeId, PDO::PARAM_INT);
        $recentStmt->bindValue(':limit', $commandFetchLimit, PDO::PARAM_INT);
        $recentStmt->execute();
    } else {
        $recentStmt = $pdo->prepare(
            'SELECT id, action, params, issued_at, created_at, delivered_at, acked_at, ack_ok, ack_result
             FROM commands
             WHERE device_id = :device_id
             ORDER BY id DESC
             LIMIT :limit'
        );
        $recentStmt->bindValue(':device_id', $deviceId, PDO::PARAM_STR);
        $recentStmt->bindValue(':limit', $commandFetchLimit, PDO::PARAM_INT);
        $recentStmt->execute();
    }

    $recentCommands = [];
    while ($row = $recentStmt->fetch(PDO::FETCH_ASSOC)) {
        $recentCommands[] = adminCommandRow($basePath, $deviceId, $row);
    }
    $commandsHasMore = false;
    if (count($recentCommands) > $commandLimit) {
        $commandsHasMore = true;
        array_pop($recentCommands);
    }
    $commandsNextBeforeId = null;
    if ($commandsHasMore && !empty($recentCommands)) {
        $tail = $recentCommands[count($recentCommands) - 1];
        $commandsNextBeforeId = (string)$tail['id'];
    }

    $online = is_int($lastSeenAgeSec) && $lastSeenAgeSec <= $adaptiveOfflineAfterSec;

    return [
        'device_id' => $deviceId,
        'now' => $nowIso,
        'health' => [
            'online' => $online,
            'last_seen' => $lastSeen,
            'last_seen_source' => $lastSeenSource,
            'last_seen_age_sec' => $lastSeenAgeSec,
            'offline_after_sec' => $adaptiveOfflineAfterSec,
            'expected_telemetry_sec' => $expectedTelemetrySec,
            'pending_commands' => $pendingCommands,
            'measurement_active' => $measurementActive,
        ],
        'latest_telemetry' => $latestTelemetry,
        'recent_commands' => $recentCommands,
        'commands_page' => [
            'limit' => $commandLimit,
            'before_id' => (is_int($commandBeforeId) && $commandBeforeId > 0) ? (string)$commandBeforeId : null,
            'has_more' => $commandsHasMore,
            'next_before_id' => $commandsNextBeforeId,
        ],
    ];
}

function parseIsoTimestamp(string $value): ?int
{
    try {
        $dt = new DateTimeImmutable($value);
        return $dt->getTimestamp();
    } catch (Throwable $e) {
        return null;
    }
}

function payloadBool(array $payload, array $path): ?bool
{
    $cur = $payload;
    foreach ($path as $segment) {
        if (!is_array($cur) || !array_key_exists($segment, $cur)) {
            return null;
        }
        $cur = $cur[$segment];
    }

    if (is_bool($cur)) {
        return $cur;
    }
    if (is_int($cur) || is_float($cur)) {
        return ((int)$cur) !== 0;
    }
    if (is_string($cur)) {
        $value = strtolower(trim($cur));
        if ($value === 'true' || $value === '1' || $value === 'yes' || $value === 'on') {
            return true;
        }
        if ($value === 'false' || $value === '0' || $value === 'no' || $value === 'off') {
            return false;
        }
    }
    return null;
}

function payloadInt(array $payload, array $path): ?int
{
    $cur = $payload;
    foreach ($path as $segment) {
        if (!is_array($cur) || !array_key_exists($segment, $cur)) {
            return null;
        }
        $cur = $cur[$segment];
    }

    if (is_int($cur)) {
        return $cur;
    }
    if (is_float($cur)) {
        return (int)$cur;
    }
    if (is_string($cur)) {
        $value = trim($cur);
        if ($value === '' || !preg_match('/^-?\d+$/', $value)) {
            return null;
        }
        return (int)$value;
    }
    return null;
}

function resolveDashboardDeviceId(string $configuredDeviceId, array $devices): string
{
    if ($configuredDeviceId !== '' && isset($devices[$configuredDeviceId])) {
        return $configuredDeviceId;
    }
    foreach ($devices as $deviceId => $deviceCfg) {
        return (string)$deviceId;
    }
    return '';
}

function resolveActiveDeviceId(string $requestedDeviceId, string $fallbackDeviceId, array $devices): string
{
    if ($requestedDeviceId !== '' && isset($devices[$requestedDeviceId])) {
        return $requestedDeviceId;
    }
    if ($fallbackDeviceId !== '' && isset($devices[$fallbackDeviceId])) {
        return $fallbackDeviceId;
    }
    foreach ($devices as $deviceId => $deviceCfg) {
        return (string)$deviceId;
    }
    return '';
}

function touchDeviceActivity(PDO $pdo, string $deviceId, string $source): void
{
    $deviceId = trim($deviceId);
    if ($deviceId === '') {
        return;
    }

    $source = trim($source);
    if ($source === '') {
        $source = 'activity';
    }

    $now = gmdate('c');
    $updateStmt = $pdo->prepare(
        'UPDATE device_activity
         SET last_seen_at = :last_seen_at,
             last_source = :last_source
         WHERE device_id = :device_id'
    );
    $updateStmt->execute([
        ':last_seen_at' => $now,
        ':last_source' => $source,
        ':device_id' => $deviceId,
    ]);

    if ($updateStmt->rowCount() > 0) {
        return;
    }

    $insertStmt = $pdo->prepare(
        'INSERT INTO device_activity (device_id, last_seen_at, last_source)
         VALUES (:device_id, :last_seen_at, :last_source)'
    );
    try {
        $insertStmt->execute([
            ':device_id' => $deviceId,
            ':last_seen_at' => $now,
            ':last_source' => $source,
        ]);
    } catch (Throwable $e) {
        // Ignore rare concurrent insert races; liveness update is best-effort.
    }
}

function publicPath(string $basePath, string $route): string
{
    $normalizedRoute = normalizePath($route);
    if ($basePath === '') {
        return $normalizedRoute;
    }
    $prefix = '/' . trim($basePath, '/');
    if ($normalizedRoute === '/') {
        return $prefix;
    }
    return $prefix . $normalizedRoute;
}

function renderDashboardPage(array $bootstrap, string $cssHref, string $jsSrc): void
{
    $title = htmlspecialchars((string)($bootstrap['dashboardTitle'] ?? 'Device Remote Control'), ENT_QUOTES, 'UTF-8');
    $cssHrefEsc = htmlspecialchars($cssHref, ENT_QUOTES, 'UTF-8');
    $jsSrcEsc = htmlspecialchars($jsSrc, ENT_QUOTES, 'UTF-8');
    $bootstrapJson = json_encode(
        $bootstrap,
        JSON_UNESCAPED_SLASHES | JSON_UNESCAPED_UNICODE | JSON_HEX_TAG | JSON_HEX_APOS | JSON_HEX_QUOT | JSON_HEX_AMP
    );
    if (!is_string($bootstrapJson)) {
        $bootstrapJson = '{}';
    }

    $html = <<<HTML
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{$title}</title>
  <link rel="stylesheet" href="{$cssHrefEsc}">
</head>
<body>
  <div class="bg-shape bg-shape-a"></div>
  <div class="bg-shape bg-shape-b"></div>

  <div class="container">
    <header class="panel topbar reveal">
      <div>
        <p class="eyebrow">Remote Control</p>
        <h1 id="pageTitle">{$title}</h1>
        <p class="sub">Control measurements and monitor health in real time.</p>
      </div>
      <div class="meta-grid">
        <div class="meta-cell">
          <span class="meta-label">Device</span>
          <code id="deviceIdLabel">--</code>
        </div>
        <div class="meta-cell">
          <span class="meta-label">Switch device</span>
          <select id="deviceSelect"></select>
        </div>
        <div class="meta-cell">
          <span class="meta-label">Last refresh</span>
          <span id="lastRefreshLabel">--</span>
        </div>
      </div>
    </header>

    <section class="panel auth reveal">
      <label for="tokenInput">Admin Token</label>
      <div class="auth-row">
        <input id="tokenInput" type="password" autocomplete="off" placeholder="Enter X-ADMIN-TOKEN">
        <button id="unlockBtn" class="btn btn-primary" type="button">Unlock</button>
        <button id="clearTokenBtn" class="btn btn-soft" type="button">Clear</button>
      </div>
      <p id="tokenState" class="hint">Locked. Enter token to enable commands.</p>
    </section>

    <section id="alertBox" class="alert hidden reveal" role="status" aria-live="polite"></section>

    <main class="grid">
      <section class="panel reveal">
        <h2>Health</h2>
        <div class="cards">
          <article class="card">
            <span class="card-label">Connection</span>
            <strong id="onlineState" class="status unknown">UNKNOWN</strong>
          </article>
          <article class="card">
            <span class="card-label">Last seen</span>
            <strong id="lastSeenAge">--</strong>
          </article>
          <article class="card">
            <span class="card-label">Measurement</span>
            <strong id="measurementState">--</strong>
          </article>
          <article class="card">
            <span class="card-label">Pending commands</span>
            <strong id="pendingCommands">0</strong>
          </article>
        </div>
      </section>

      <section class="panel reveal">
        <h2>Runtime Diagnostics</h2>
        <div class="cards">
          <article class="card">
            <span class="card-label">Free heap</span>
            <strong id="heapFree">--</strong>
          </article>
          <article class="card">
            <span class="card-label">Min heap</span>
            <strong id="heapMin">--</strong>
          </article>
          <article class="card">
            <span class="card-label">Heap frag</span>
            <strong id="heapFrag">--</strong>
          </article>
          <article class="card">
            <span class="card-label">CPU busy</span>
            <strong id="cpuBusy">--</strong>
          </article>
          <article class="card">
            <span class="card-label">Loop CPU</span>
            <strong id="loopCpu">--</strong>
          </article>
          <article class="card">
            <span class="card-label">Meas CPU</span>
            <strong id="measCpu">--</strong>
          </article>
          <article class="card">
            <span class="card-label">Loop stall</span>
            <strong id="loopBlock">--</strong>
          </article>
          <article class="card">
            <span class="card-label">Tasks</span>
            <strong id="taskCount">--</strong>
          </article>
          <article class="card">
            <span class="card-label">Loop stack</span>
            <strong id="loopStack">--</strong>
          </article>
          <article class="card">
            <span class="card-label">Meas stack</span>
            <strong id="measStack">--</strong>
          </article>
        </div>
        <p id="runtimeHint" class="hint">No runtime diagnostics yet.</p>
        <p id="prevRunHint" class="hint">No previous-run breadcrumb yet.</p>
      </section>

      <section class="panel reveal">
        <h2>Commands</h2>
        <div class="commands">
          <button id="startBtn" class="btn btn-success" type="button">Start Measurement</button>
          <button id="stopBtn" class="btn btn-danger" type="button">Stop Measurement</button>
          <button id="rebootBtn" class="btn btn-warning" type="button">Reboot Device</button>
          <button id="refreshBtn" class="btn btn-soft" type="button">Refresh Now</button>
          <a id="uploadsLink" class="btn btn-primary" href="#">Open Upload Browser</a>
        </div>
      </section>

      <section class="panel panel-wide reveal">
        <h2>Recent Commands</h2>
        <div class="table-wrap">
          <table>
            <thead>
              <tr>
                <th>ID</th>
                <th>Action</th>
                <th>Status</th>
                <th>Created</th>
                <th>ACK</th>
                <th>Result</th>
                <th>Manage</th>
              </tr>
            </thead>
            <tbody id="commandRows">
              <tr><td colspan="7" class="muted no-commands-cell" data-label="Info">No commands yet.</td></tr>
            </tbody>
          </table>
        </div>
        <div class="pager">
          <button id="cmdPageNewerBtn" class="btn btn-soft pager-btn" type="button">Newer</button>
          <span id="cmdPageInfo" class="hint">Page 1</span>
          <button id="cmdPageOlderBtn" class="btn btn-soft pager-btn" type="button">Older</button>
        </div>
      </section>

      <section class="panel panel-wide reveal">
        <h2>Latest Telemetry</h2>
        <pre id="telemetryOut" class="telemetry muted">No telemetry received yet.</pre>
      </section>
    </main>
  </div>

  <script>window.DASHBOARD_BOOTSTRAP = {$bootstrapJson};</script>
  <script src="{$jsSrcEsc}"></script>
</body>
</html>
HTML;

    respondHtml(200, $html);
}

function renderUploadsPage(array $bootstrap, string $cssHref, string $jsSrc): void
{
    $title = htmlspecialchars((string)($bootstrap['dashboardTitle'] ?? 'Device Remote Control'), ENT_QUOTES, 'UTF-8');
    $cssHrefEsc = htmlspecialchars($cssHref, ENT_QUOTES, 'UTF-8');
    $jsSrcEsc = htmlspecialchars($jsSrc, ENT_QUOTES, 'UTF-8');
    $dashboardUrl = htmlspecialchars((string)($bootstrap['dashboardUrl'] ?? '/dashboard'), ENT_QUOTES, 'UTF-8');
    $bootstrapJson = json_encode(
        $bootstrap,
        JSON_UNESCAPED_SLASHES | JSON_UNESCAPED_UNICODE | JSON_HEX_TAG | JSON_HEX_APOS | JSON_HEX_QUOT | JSON_HEX_AMP
    );
    if (!is_string($bootstrapJson)) {
        $bootstrapJson = '{}';
    }

    $html = <<<HTML
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{$title} - Upload Browser</title>
  <link rel="stylesheet" href="{$cssHrefEsc}">
</head>
<body>
  <div class="bg-shape bg-shape-a"></div>
  <div class="bg-shape bg-shape-b"></div>

  <div class="container">
    <header class="panel topbar reveal">
      <div>
        <p class="eyebrow">Remote Files</p>
        <h1 id="uploadsPageTitle">{$title}</h1>
        <p class="sub">Browse uploaded measurement files and download them to your PC.</p>
      </div>
      <div class="meta-grid">
        <div class="meta-cell">
          <span class="meta-label">Device</span>
          <code id="uploadsDeviceIdLabel">--</code>
        </div>
        <div class="meta-cell">
          <span class="meta-label">Switch device</span>
          <select id="uploadsDeviceSelect"></select>
        </div>
        <div class="meta-cell">
          <span class="meta-label">Last refresh</span>
          <span id="uploadsLastRefreshLabel">--</span>
        </div>
      </div>
    </header>

    <section class="panel auth reveal">
      <label for="uploadsTokenInput">Admin Token</label>
      <div class="auth-row">
        <input id="uploadsTokenInput" type="password" autocomplete="off" placeholder="Enter X-ADMIN-TOKEN">
        <button id="uploadsUnlockBtn" class="btn btn-primary" type="button">Unlock</button>
        <button id="uploadsClearTokenBtn" class="btn btn-soft" type="button">Clear</button>
      </div>
      <p id="uploadsTokenState" class="hint">Locked. Enter token to load files.</p>
    </section>

    <section id="uploadsAlertBox" class="alert hidden reveal" role="status" aria-live="polite"></section>

    <section class="panel reveal">
      <h2>Actions</h2>
      <div class="commands">
        <button id="uploadsRefreshBtn" class="btn btn-soft" type="button">Refresh Files</button>
        <button id="uploadsUpBtn" class="btn btn-soft" type="button">Go Up</button>
        <a id="uploadsDashboardLink" class="btn btn-primary" href="{$dashboardUrl}">Back To Dashboard</a>
      </div>
      <p class="hint">Current path: <code id="uploadsPathLabel">/</code></p>
    </section>

    <section class="panel panel-wide reveal">
      <h2>Uploaded Files</h2>
      <div class="table-wrap">
        <table>
          <thead>
            <tr>
              <th>Type</th>
              <th>Name</th>
              <th>Status</th>
              <th>Size</th>
              <th>Updated</th>
              <th>Actions</th>
            </tr>
          </thead>
          <tbody id="uploadRows">
            <tr><td colspan="6" class="muted no-commands-cell" data-label="Info">No uploads yet.</td></tr>
          </tbody>
        </table>
      </div>
    </section>
  </div>

  <script>window.UPLOADS_BOOTSTRAP = {$bootstrapJson};</script>
  <script src="{$jsSrcEsc}"></script>
</body>
</html>
HTML;

    respondHtml(200, $html);
}

function canonicalCommandPayload(string $id, string $action, string $params, string $issuedAt, string $nonce): string
{
    return '{'
        . '"id":"' . jsonEscape($id) . '",'
        . '"action":"' . jsonEscape($action) . '",'
        . '"params":"' . jsonEscape($params) . '",'
        . '"issued_at":"' . jsonEscape($issuedAt) . '",'
        . '"nonce":"' . jsonEscape($nonce) . '"'
        . '}';
}

function jsonEscape(string $value): string
{
    $encoded = json_encode(
        $value,
        JSON_UNESCAPED_SLASHES | JSON_UNESCAPED_UNICODE | JSON_INVALID_UTF8_SUBSTITUTE
    );
    if (!is_string($encoded) || strlen($encoded) < 2) {
        return '';
    }
    return substr($encoded, 1, -1);
}

function openDatabase(string $dbPath): PDO
{
    $pdo = new PDO('sqlite:' . $dbPath);
    $pdo->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
    $pdo->setAttribute(PDO::ATTR_DEFAULT_FETCH_MODE, PDO::FETCH_ASSOC);
    $pdo->exec('PRAGMA journal_mode = WAL');
    $pdo->exec('PRAGMA busy_timeout = 5000');
    return $pdo;
}

function initDatabase(PDO $pdo): void
{
    $pdo->exec(
        'CREATE TABLE IF NOT EXISTS telemetry (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            device_id TEXT NOT NULL,
            received_at TEXT NOT NULL,
            payload_json TEXT NOT NULL,
            source_ip TEXT
        )'
    );

    $pdo->exec(
        'CREATE TABLE IF NOT EXISTS uploads (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            device_id TEXT NOT NULL,
            filename TEXT NOT NULL,
            stored_path TEXT NOT NULL,
            bytes INTEGER NOT NULL,
            received_at TEXT NOT NULL,
            source_ip TEXT,
            storage_status TEXT NOT NULL DEFAULT "present",
            storage_checked_at TEXT
        )'
    );
    ensureUploadsSchema($pdo);

    $pdo->exec(
        'CREATE TABLE IF NOT EXISTS commands (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            device_id TEXT NOT NULL,
            action TEXT NOT NULL,
            params TEXT NOT NULL DEFAULT "",
            issued_at TEXT NOT NULL,
            nonce TEXT NOT NULL,
            sig TEXT NOT NULL,
            created_at TEXT NOT NULL,
            delivered_at TEXT,
            acked_at TEXT,
            ack_ok INTEGER,
            ack_result TEXT,
            ack_payload_json TEXT
        )'
    );

    $pdo->exec(
        'CREATE TABLE IF NOT EXISTS device_activity (
            device_id TEXT PRIMARY KEY,
            last_seen_at TEXT NOT NULL,
            last_source TEXT NOT NULL
        )'
    );

    $pdo->exec('CREATE INDEX IF NOT EXISTS idx_commands_device_id ON commands (device_id, id)');
    $pdo->exec('CREATE INDEX IF NOT EXISTS idx_commands_pending ON commands (device_id, acked_at, id)');
    $pdo->exec('CREATE INDEX IF NOT EXISTS idx_telemetry_device_id ON telemetry (device_id, id)');
    $pdo->exec('CREATE INDEX IF NOT EXISTS idx_uploads_device_id ON uploads (device_id, id)');
}

function ensureUploadsSchema(PDO $pdo): void
{
    $columns = [];
    $stmt = $pdo->query('PRAGMA table_info(uploads)');
    while ($row = $stmt->fetch(PDO::FETCH_ASSOC)) {
        $name = isset($row['name']) ? (string)$row['name'] : '';
        if ($name !== '') {
            $columns[$name] = true;
        }
    }

    if (!isset($columns['storage_status'])) {
        $pdo->exec('ALTER TABLE uploads ADD COLUMN storage_status TEXT NOT NULL DEFAULT "present"');
    }
    if (!isset($columns['storage_checked_at'])) {
        $pdo->exec('ALTER TABLE uploads ADD COLUMN storage_checked_at TEXT');
    }

    $pdo->exec('UPDATE uploads SET storage_status = "present" WHERE storage_status IS NULL OR TRIM(storage_status) = ""');
}

function normalizeUploadStorageStatus(string $status): string
{
    $status = strtolower(trim($status));
    if ($status === 'missing') {
        return 'missing';
    }
    return 'present';
}

function markUploadStorageStatus(PDO $pdo, int $uploadId, string $status): void
{
    if ($uploadId <= 0) {
        return;
    }
    $status = normalizeUploadStorageStatus($status);
    $stmt = $pdo->prepare(
        'UPDATE uploads
         SET storage_status = :storage_status,
             storage_checked_at = :storage_checked_at
         WHERE id = :id'
    );
    $stmt->execute([
        ':storage_status' => $status,
        ':storage_checked_at' => gmdate('c'),
        ':id' => $uploadId,
    ]);
}

function parseUploadNameParts(string $rawName): array
{
    $raw = trim(str_replace('\\', '/', $rawName));
    if ($raw === '') {
        return [
            'directory' => '',
            'filename' => '',
            'relative_path' => '',
        ];
    }

    $raw = preg_replace('#/+#', '/', $raw);
    if (!is_string($raw)) {
        $raw = '';
    }
    $raw = trim($raw, '/');
    if ($raw === '') {
        return [
            'directory' => '',
            'filename' => '',
            'relative_path' => '',
        ];
    }

    $parts = explode('/', $raw);
    $dirParts = [];
    for ($i = 0; $i < count($parts) - 1; $i++) {
        $segment = trim((string)$parts[$i]);
        if ($segment === '' || $segment === '.' || $segment === '..') {
            continue;
        }
        $dirParts[] = sanitizeSegment($segment);
    }

    $leafRaw = trim((string)$parts[count($parts) - 1]);
    if ($leafRaw === '.' || $leafRaw === '..') {
        $leafRaw = '';
    }
    $filename = sanitizeFileName($leafRaw);
    if ($filename === '') {
        return [
            'directory' => '',
            'filename' => '',
            'relative_path' => '',
        ];
    }

    $directory = implode('/', $dirParts);
    $relativePath = ($directory !== '') ? ($directory . '/' . $filename) : $filename;

    return [
        'directory' => $directory,
        'filename' => $filename,
        'relative_path' => $relativePath,
    ];
}

function sanitizeUploadRelativePath(string $rawPath): string
{
    $raw = trim(str_replace('\\', '/', $rawPath));
    if ($raw === '') {
        return '';
    }

    $raw = preg_replace('#/+#', '/', $raw);
    if (!is_string($raw)) {
        return '';
    }

    $raw = trim($raw, '/');
    if ($raw === '') {
        return '';
    }

    $segments = explode('/', $raw);
    $safe = [];
    foreach ($segments as $segment) {
        $segment = trim((string)$segment);
        if ($segment === '' || $segment === '.' || $segment === '..') {
            continue;
        }
        $safe[] = sanitizeSegment($segment);
    }

    return implode('/', $safe);
}

function parentUploadPath(string $path): ?string
{
    $path = sanitizeUploadRelativePath($path);
    if ($path === '') {
        return null;
    }
    $pos = strrpos($path, '/');
    if ($pos === false) {
        return '';
    }
    return substr($path, 0, $pos);
}

function resolveUploadStoredPath(string $storedPath, string $uploadDir): ?string
{
    $storedPath = trim($storedPath);
    if ($storedPath === '') {
        return null;
    }

    $uploadDirReal = realpath($uploadDir);
    $storedPathReal = realpath($storedPath);
    if (!is_string($uploadDirReal) || !is_string($storedPathReal)) {
        return null;
    }

    $prefix = rtrim($uploadDirReal, DIRECTORY_SEPARATOR) . DIRECTORY_SEPARATOR;
    if ($storedPathReal === $uploadDirReal || str_starts_with($storedPathReal, $prefix)) {
        return $storedPathReal;
    }

    return null;
}

function uploadRelativePathFromRow(array $row, string $deviceId, string $uploadDir): string
{
    $filename = isset($row['filename']) ? (string)$row['filename'] : '';
    $parsed = parseUploadNameParts($filename);
    if ($parsed['relative_path'] !== '') {
        return $parsed['relative_path'];
    }

    $storedPath = isset($row['stored_path']) ? trim((string)$row['stored_path']) : '';
    if ($storedPath !== '') {
        $deviceDir = rtrim(str_replace('\\', '/', $uploadDir), '/') . '/' . sanitizeSegment($deviceId) . '/';
        $storedNorm = str_replace('\\', '/', $storedPath);
        if (str_starts_with($storedNorm, $deviceDir)) {
            $relative = substr($storedNorm, strlen($deviceDir));
            $fromPath = parseUploadNameParts($relative);
            if ($fromPath['relative_path'] !== '') {
                return $fromPath['relative_path'];
            }
        }
        $storedBase = basename($storedNorm);
        $storedFile = sanitizeFileName($storedBase);
        if ($storedFile !== '') {
            return $storedFile;
        }
    }

    return '';
}

function loadUploadsForDevice(PDO $pdo, string $deviceId, string $uploadDir): array
{
    $stmt = $pdo->prepare(
        'SELECT id, filename, stored_path, bytes, received_at, storage_status, storage_checked_at
         FROM uploads
         WHERE device_id = :device_id
         ORDER BY id DESC'
    );
    $stmt->execute([':device_id' => $deviceId]);

    $out = [];
    $seen = [];
    while ($row = $stmt->fetch(PDO::FETCH_ASSOC)) {
        $relativePath = uploadRelativePathFromRow($row, $deviceId, $uploadDir);
        if ($relativePath === '' || isset($seen[$relativePath])) {
            continue;
        }
        $seen[$relativePath] = true;

        $id = (int)($row['id'] ?? 0);
        if ($id <= 0) {
            continue;
        }

        $storedPath = (string)($row['stored_path'] ?? '');
        $storedPathReal = resolveUploadStoredPath($storedPath, $uploadDir);
        $fileExists = is_string($storedPathReal) && is_file($storedPathReal);

        $observedStatus = $fileExists ? 'present' : 'missing';
        $recordedStatus = normalizeUploadStorageStatus((string)($row['storage_status'] ?? ''));
        $recordedCheckedAt = isset($row['storage_checked_at']) ? (string)$row['storage_checked_at'] : '';
        if ($recordedStatus !== $observedStatus || trim($recordedCheckedAt) === '') {
            markUploadStorageStatus($pdo, $id, $observedStatus);
        }

        $out[] = [
            'id' => $id,
            'relative_path' => $relativePath,
            'filename' => (string)($row['filename'] ?? ''),
            'stored_path' => $storedPath,
            'stored_path_real' => $storedPathReal,
            'bytes' => (int)($row['bytes'] ?? 0),
            'received_at' => (string)($row['received_at'] ?? ''),
            'storage_status' => $observedStatus,
            'file_exists' => $fileExists,
        ];
    }

    return $out;
}

function buildUploadsListing(array $entries, string $deviceId, string $currentPath, string $basePath): array
{
    $currentPath = sanitizeUploadRelativePath($currentPath);
    $prefix = ($currentPath === '') ? '' : ($currentPath . '/');

    $folderMap = [];
    $files = [];

    foreach ($entries as $entry) {
        $relativePath = (string)($entry['relative_path'] ?? '');
        if ($relativePath === '' || ($prefix !== '' && !str_starts_with($relativePath, $prefix))) {
            continue;
        }

        $remaining = ($prefix === '') ? $relativePath : substr($relativePath, strlen($prefix));
        if ($remaining === '' || $remaining === false) {
            continue;
        }

        $slashPos = strpos($remaining, '/');
        if ($slashPos !== false) {
            $folderName = substr($remaining, 0, $slashPos);
            if ($folderName === '') {
                continue;
            }
            $folderPath = ($currentPath === '') ? $folderName : ($currentPath . '/' . $folderName);
            if (!isset($folderMap[$folderPath])) {
                $folderMap[$folderPath] = [
                    'name' => $folderName,
                    'path' => $folderPath,
                    'file_count' => 0,
                    'bytes' => 0,
                    'received_at' => '',
                    'missing_count' => 0,
                ];
            }
            $folderMap[$folderPath]['file_count']++;
            $folderMap[$folderPath]['bytes'] += (int)($entry['bytes'] ?? 0);
            if (((string)($entry['storage_status'] ?? '')) === 'missing') {
                $folderMap[$folderPath]['missing_count']++;
            }
            $receivedAt = (string)($entry['received_at'] ?? '');
            if ($receivedAt !== '' && $receivedAt > $folderMap[$folderPath]['received_at']) {
                $folderMap[$folderPath]['received_at'] = $receivedAt;
            }
            continue;
        }

        $id = (int)($entry['id'] ?? 0);
        if ($id <= 0) {
            continue;
        }

        $files[] = [
            'id' => (string)$id,
            'type' => 'file',
            'name' => basename($relativePath),
            'path' => $relativePath,
            'bytes' => (int)($entry['bytes'] ?? 0),
            'received_at' => (string)($entry['received_at'] ?? ''),
            'storage_status' => normalizeUploadStorageStatus((string)($entry['storage_status'] ?? '')),
            'file_exists' => !empty($entry['file_exists']),
            'download_url' => publicPath(
                $basePath,
                '/admin/devices/' . rawurlencode($deviceId) . '/uploads/' . rawurlencode((string)$id) . '/download'
            ),
            'delete_url' => publicPath(
                $basePath,
                '/admin/devices/' . rawurlencode($deviceId) . '/uploads/' . rawurlencode((string)$id) . '/delete'
            ),
        ];
    }

    ksort($folderMap, SORT_NATURAL | SORT_FLAG_CASE);
    usort($files, static function (array $a, array $b): int {
        return strnatcasecmp((string)($a['name'] ?? ''), (string)($b['name'] ?? ''));
    });

    $folders = [];
    foreach ($folderMap as $folder) {
        $folderPath = (string)$folder['path'];
        $downloadUrl = publicPath(
            $basePath,
            '/admin/devices/' . rawurlencode($deviceId) . '/uploads/folder-download'
        ) . '?path=' . rawurlencode($folderPath);

        $status = ((int)($folder['missing_count'] ?? 0) > 0) ? 'missing' : 'present';
        $folders[] = [
            'type' => 'folder',
            'name' => (string)$folder['name'],
            'path' => $folderPath,
            'file_count' => (int)$folder['file_count'],
            'bytes' => (int)$folder['bytes'],
            'received_at' => (string)$folder['received_at'],
            'storage_status' => $status,
            'download_url' => $downloadUrl,
        ];
    }

    return [
        'folders' => $folders,
        'files' => $files,
    ];
}

function buildPathCandidates(string $rawPath, string $basePath): array
{
    $path = normalizePath($rawPath);
    $candidates = [$path];

    if ($basePath !== '') {
        $basePrefix = '/' . trim($basePath, '/');
        $candidates[] = stripPrefix($path, $basePrefix);
        $candidates[] = stripPrefix($path, $basePrefix . '/public');
    }

    $candidates[] = stripPrefix($path, '/public');

    $clean = [];
    foreach ($candidates as $candidate) {
        $candidate = normalizePath($candidate);
        $clean[$candidate] = true;
    }

    return array_keys($clean);
}

function normalizePath(string $path): string
{
    if ($path === '') {
        return '/';
    }
    if ($path[0] !== '/') {
        $path = '/' . $path;
    }
    if (strlen($path) > 1) {
        $path = rtrim($path, '/');
        if ($path === '') {
            $path = '/';
        }
    }
    return $path;
}

function stripPrefix(string $path, string $prefix): string
{
    if ($prefix === '' || $prefix === '/') {
        return $path;
    }
    if ($path === $prefix) {
        return '/';
    }
    if (str_starts_with($path, $prefix . '/')) {
        return substr($path, strlen($prefix));
    }
    return $path;
}

function pathEquals(array $candidates, string $route): bool
{
    $route = normalizePath($route);
    return in_array($route, $candidates, true);
}

function routeMatches(array $candidates, string $pattern, ?array &$out = null): bool
{
    foreach ($candidates as $candidate) {
        if (preg_match($pattern, $candidate, $matches) === 1) {
            $out = $matches;
            return true;
        }
    }
    return false;
}

function pathStartsWith(array $candidates, string $prefix): bool
{
    $prefix = normalizePath($prefix);
    foreach ($candidates as $candidate) {
        if ($candidate === $prefix || str_starts_with($candidate, $prefix . '/')) {
            return true;
        }
    }
    return false;
}

function requireDeviceByApiKey(array $devices, string $apiKey, string $hintDeviceId = ''): array
{
    $apiKey = trim($apiKey);
    if ($apiKey === '') {
        respondJson(401, ['ok' => false, 'error' => 'missing_api_key']);
    }

    $matchingDeviceIds = [];
    foreach ($devices as $deviceId => $deviceCfg) {
        $expectedKey = (string)($deviceCfg['api_key'] ?? '');
        if ($expectedKey !== '' && hash_equals($expectedKey, $apiKey)) {
            $matchingDeviceIds[] = (string)$deviceId;
        }
    }

    if (count($matchingDeviceIds) === 0) {
        respondJson(401, ['ok' => false, 'error' => 'invalid_api_key']);
    }

    $hintDeviceId = trim($hintDeviceId);
    if ($hintDeviceId !== '') {
        if (!isset($devices[$hintDeviceId])) {
            respondJson(404, ['ok' => false, 'error' => 'unknown_device']);
        }

        $expectedHintKey = (string)($devices[$hintDeviceId]['api_key'] ?? '');
        if ($expectedHintKey === '' || !hash_equals($expectedHintKey, $apiKey)) {
            respondJson(401, ['ok' => false, 'error' => 'invalid_api_key_for_device']);
        }

        return [
            'device_id' => $hintDeviceId,
            'config' => $devices[$hintDeviceId],
        ];
    }

    if (count($matchingDeviceIds) > 1) {
        respondJson(400, [
            'ok' => false,
            'error' => 'ambiguous_api_key_device_id_required',
            'hint' => 'Provide device_id via query (?device_id=...) or header (X-DEVICE-ID).',
        ]);
    }

    $deviceId = $matchingDeviceIds[0];
    return [
        'device_id' => $deviceId,
        'config' => $devices[$deviceId],
    ];
}

function requireDeviceForPath(array $devices, string $deviceId, string $apiKey): void
{
    if (!isset($devices[$deviceId])) {
        respondJson(404, ['ok' => false, 'error' => 'unknown_device']);
    }

    $apiKey = trim($apiKey);
    if ($apiKey === '') {
        respondJson(401, ['ok' => false, 'error' => 'missing_api_key']);
    }

    $expectedKey = (string)($devices[$deviceId]['api_key'] ?? '');
    if ($expectedKey === '' || !hash_equals($expectedKey, $apiKey)) {
        respondJson(401, ['ok' => false, 'error' => 'invalid_api_key']);
    }
}

function requireAdminToken(string $adminToken): void
{
    $adminToken = normalizeToken($adminToken);
    if ($adminToken === '' || $adminToken === '__CHANGE_ME__') {
        respondJson(503, ['ok' => false, 'error' => 'admin_token_not_configured']);
    }

    $provided = extractProvidedAdminToken();

    if ($provided === '' || !hash_equals($adminToken, $provided)) {
        respondJson(401, ['ok' => false, 'error' => 'invalid_admin_token']);
    }
}

function extractProvidedAdminToken(): string
{
    $candidates = [
        requestHeader('X-ADMIN-TOKEN'),
        requestHeader('X-API-KEY'),
    ];

    $authHeader = trim(requestHeader('Authorization'));
    if (preg_match('/^Bearer\\s+(.+)$/i', $authHeader, $matches) === 1) {
        $candidates[] = $matches[1];
    }

    foreach ($candidates as $candidate) {
        $token = normalizeToken((string)$candidate);
        if ($token !== '') {
            return $token;
        }
    }

    return '';
}

function normalizeToken(string $value): string
{
    $token = trim(str_replace("\xEF\xBB\xBF", '', $value));
    if (strlen($token) >= 2) {
        $first = $token[0];
        $last = $token[strlen($token) - 1];
        if (($first === '"' && $last === '"') || ($first === "'" && $last === "'")) {
            $token = trim(substr($token, 1, -1));
        }
    }
    return $token;
}

function requestHeader(string $name): string
{
    static $headers = null;

    if ($headers === null) {
        $headers = [];
        if (function_exists('getallheaders')) {
            foreach (getallheaders() as $key => $value) {
                $headers[strtolower((string)$key)] = trim((string)$value);
            }
        }

        foreach ($_SERVER as $key => $value) {
            if (!str_starts_with($key, 'HTTP_')) {
                continue;
            }
            $header = strtolower(str_replace('_', '-', substr($key, 5)));
            if (!isset($headers[$header])) {
                $headers[$header] = trim((string)$value);
            }
        }
    }

    return (string)($headers[strtolower($name)] ?? '');
}

function requestBody(): string
{
    static $body = null;
    if ($body === null) {
        $raw = file_get_contents('php://input');
        $body = is_string($raw) ? $raw : '';
    }
    return $body;
}

function decodeJsonBody(string $rawBody)
{
    if ($rawBody === '') {
        return null;
    }

    try {
        return json_decode($rawBody, true, 512, JSON_THROW_ON_ERROR);
    } catch (Throwable $e) {
        respondJson(400, ['ok' => false, 'error' => 'invalid_json']);
    }
}

function resolveIngestDeviceIdHint(bool $isUpload): string
{
    $headerHint = trim(requestHeader('X-DEVICE-ID'));
    if ($headerHint !== '') {
        return $headerHint;
    }

    $queryHint = trim((string)($_GET['device_id'] ?? ''));
    if ($queryHint !== '') {
        return $queryHint;
    }

    if ($isUpload) {
        return '';
    }

    $rawBody = requestBody();
    if ($rawBody === '') {
        return '';
    }

    $payload = json_decode($rawBody, true);
    if (!is_array($payload)) {
        return '';
    }

    $bodyHint = trim((string)($payload['device_id'] ?? ''));
    if ($bodyHint !== '') {
        return $bodyHint;
    }

    return '';
}

function sanitizeSegment(string $value): string
{
    $clean = preg_replace('/[^A-Za-z0-9._-]/', '_', $value);
    if (!is_string($clean) || $clean === '') {
        return 'device';
    }
    return $clean;
}

function sanitizeFileName(string $name): string
{
    $name = basename(trim($name));
    $name = preg_replace('/[^A-Za-z0-9._-]/', '_', $name);
    if (!is_string($name)) {
        return '';
    }
    return trim($name, '._');
}

function clientIp(): string
{
    $xff = trim((string)($_SERVER['HTTP_X_FORWARDED_FOR'] ?? ''));
    if ($xff !== '') {
        $parts = explode(',', $xff);
        return trim($parts[0]);
    }
    return trim((string)($_SERVER['REMOTE_ADDR'] ?? ''));
}

function ensureDirectory(string $path): void
{
    if (is_dir($path)) {
        return;
    }
    if (!mkdir($path, 0775, true) && !is_dir($path)) {
        respondJson(500, ['ok' => false, 'error' => 'mkdir_failed', 'path' => $path]);
    }
}

function respondNoContent(): void
{
    http_response_code(204);
    header('Cache-Control: no-store');
    exit;
}

function respondHtml(int $code, string $html): void
{
    http_response_code($code);
    header('Content-Type: text/html; charset=utf-8');
    header('Cache-Control: no-store');
    echo $html;
    exit;
}

function respondJson(int $code, array $payload): void
{
    http_response_code($code);
    header('Content-Type: application/json; charset=utf-8');
    header('Cache-Control: no-store');

    if ($code === 204) {
        exit;
    }

    $json = json_encode($payload, JSON_UNESCAPED_SLASHES | JSON_UNESCAPED_UNICODE);
    if (!is_string($json)) {
        $json = '{"ok":false,"error":"json_encode_failed"}';
    }

    echo $json;
    exit;
}
