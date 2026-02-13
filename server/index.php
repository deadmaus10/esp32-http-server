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
    $bootstrap = [
        'basePath' => ($basePath === '') ? '' : ('/' . $basePath),
        'dashboardTitle' => $dashboardTitle,
        'deviceId' => $dashboardDeviceId,
        'pollSec' => $dashboardPollSec,
        'offlineAfterSec' => $offlineAfterSec,
        'apiDashboardUrlTemplate' => publicPath($basePath, '/admin/devices/{device_id}/dashboard'),
        'apiCommandsUrlTemplate' => publicPath($basePath, '/admin/devices/{device_id}/commands'),
    ];
    renderDashboardPage($bootstrap, publicPath($basePath, '/assets/dashboard.css'), publicPath($basePath, '/assets/dashboard.js'));
}

if ($method === 'POST' && pathEquals($pathCandidates, '/ingest')) {
    $device = requireDeviceByApiKey($devices, requestHeader('X-API-KEY'));
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

        $snapshot = loadDashboardSnapshot($pdo, $deviceId, $offlineAfterSec, $commandLimit);
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
            $rows[] = [
                'id' => (string)$row['id'],
                'action' => (string)$row['action'],
                'params' => (string)$row['params'],
                'issued_at' => (string)$row['issued_at'],
                'nonce' => (string)$row['nonce'],
                'sig' => (string)$row['sig'],
                'created_at' => (string)$row['created_at'],
                'delivered_at' => $row['delivered_at'],
                'acked_at' => $row['acked_at'],
                'ack_ok' => is_null($row['ack_ok']) ? null : ((int)$row['ack_ok'] === 1),
                'ack_result' => $row['ack_result'],
            ];
        }

        respondJson(200, ['ok' => true, 'commands' => $rows]);
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
    $name = sanitizeFileName((string)($_GET['name'] ?? 'upload.bin'));
    if ($name === '') {
        respondJson(400, ['ok' => false, 'error' => 'missing_upload_name']);
    }

    $deviceDir = $uploadDir . '/' . sanitizeSegment($deviceId);
    ensureDirectory($deviceDir);

    $storedFileName = gmdate('Ymd_His') . '_' . $name;
    $storedPath = $deviceDir . '/' . $storedFileName;
    $bytes = file_put_contents($storedPath, $rawBody, LOCK_EX);
    if ($bytes === false) {
        respondJson(500, ['ok' => false, 'error' => 'upload_store_failed']);
    }

    $stmt = $pdo->prepare(
        'INSERT INTO uploads (device_id, filename, stored_path, bytes, received_at, source_ip)
         VALUES (:device_id, :filename, :stored_path, :bytes, :received_at, :source_ip)'
    );
    $stmt->execute([
        ':device_id' => $deviceId,
        ':filename' => $name,
        ':stored_path' => $storedPath,
        ':bytes' => (int)$bytes,
        ':received_at' => gmdate('c'),
        ':source_ip' => clientIp(),
    ]);

    respondJson(201, [
        'ok' => true,
        'stored' => basename($storedPath),
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

    respondJson(202, ['ok' => true]);
}

function loadDashboardSnapshot(PDO $pdo, string $deviceId, int $offlineAfterSec, int $commandLimit): array
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
    $latestTelemetry = null;
    $lastSeenAgeSec = null;
    $measurementActive = null;
    $expectedTelemetrySec = null;
    $adaptiveOfflineAfterSec = $offlineAfterSec;
    if (is_array($telemetryRow)) {
        $lastSeen = (string)$telemetryRow['received_at'];
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

    $recentStmt = $pdo->prepare(
        'SELECT id, action, params, issued_at, created_at, delivered_at, acked_at, ack_ok, ack_result
         FROM commands
         WHERE device_id = :device_id
         ORDER BY id DESC
         LIMIT :limit'
    );
    $recentStmt->bindValue(':device_id', $deviceId, PDO::PARAM_STR);
    $recentStmt->bindValue(':limit', $commandLimit, PDO::PARAM_INT);
    $recentStmt->execute();

    $recentCommands = [];
    while ($row = $recentStmt->fetch(PDO::FETCH_ASSOC)) {
        $recentCommands[] = [
            'id' => (string)$row['id'],
            'action' => (string)$row['action'],
            'params' => (string)$row['params'],
            'issued_at' => (string)$row['issued_at'],
            'created_at' => (string)$row['created_at'],
            'delivered_at' => $row['delivered_at'],
            'acked_at' => $row['acked_at'],
            'ack_ok' => is_null($row['ack_ok']) ? null : ((int)$row['ack_ok'] === 1),
            'ack_result' => is_null($row['ack_result']) ? null : (string)$row['ack_result'],
        ];
    }

    $online = is_int($lastSeenAgeSec) && $lastSeenAgeSec <= $adaptiveOfflineAfterSec;

    return [
        'device_id' => $deviceId,
        'now' => $nowIso,
        'health' => [
            'online' => $online,
            'last_seen' => $lastSeen,
            'last_seen_age_sec' => $lastSeenAgeSec,
            'offline_after_sec' => $adaptiveOfflineAfterSec,
            'expected_telemetry_sec' => $expectedTelemetrySec,
            'pending_commands' => $pendingCommands,
            'measurement_active' => $measurementActive,
        ],
        'latest_telemetry' => $latestTelemetry,
        'recent_commands' => $recentCommands,
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
    if ($configuredDeviceId !== '') {
        return $configuredDeviceId;
    }
    foreach ($devices as $deviceId => $deviceCfg) {
        return (string)$deviceId;
    }
    return '';
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
        <h2>Commands</h2>
        <div class="commands">
          <button id="startBtn" class="btn btn-success" type="button">Start Measurement</button>
          <button id="stopBtn" class="btn btn-danger" type="button">Stop Measurement</button>
          <button id="rebootBtn" class="btn btn-warning" type="button">Reboot Device</button>
          <button id="refreshBtn" class="btn btn-soft" type="button">Refresh Now</button>
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
              </tr>
            </thead>
            <tbody id="commandRows">
              <tr><td colspan="6" class="muted">No commands yet.</td></tr>
            </tbody>
          </table>
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
            source_ip TEXT
        )'
    );

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

    $pdo->exec('CREATE INDEX IF NOT EXISTS idx_commands_device_id ON commands (device_id, id)');
    $pdo->exec('CREATE INDEX IF NOT EXISTS idx_commands_pending ON commands (device_id, acked_at, id)');
    $pdo->exec('CREATE INDEX IF NOT EXISTS idx_telemetry_device_id ON telemetry (device_id, id)');
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

function requireDeviceByApiKey(array $devices, string $apiKey): array
{
    $apiKey = trim($apiKey);
    if ($apiKey === '') {
        respondJson(401, ['ok' => false, 'error' => 'missing_api_key']);
    }

    foreach ($devices as $deviceId => $deviceCfg) {
        $expectedKey = (string)($deviceCfg['api_key'] ?? '');
        if ($expectedKey !== '' && hash_equals($expectedKey, $apiKey)) {
            return [
                'device_id' => $deviceId,
                'config' => $deviceCfg,
            ];
        }
    }

    respondJson(401, ['ok' => false, 'error' => 'invalid_api_key']);
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
