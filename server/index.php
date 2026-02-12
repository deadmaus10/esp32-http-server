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
    $adminToken = trim($adminToken);
    if ($adminToken === '' || $adminToken === '__CHANGE_ME__') {
        respondJson(503, ['ok' => false, 'error' => 'admin_token_not_configured']);
    }

    $provided = trim(requestHeader('X-ADMIN-TOKEN'));
    if ($provided === '') {
        $authHeader = trim(requestHeader('Authorization'));
        if (preg_match('/^Bearer\\s+(.+)$/i', $authHeader, $matches) === 1) {
            $provided = trim($matches[1]);
        }
    }

    if ($provided === '' || !hash_equals($adminToken, $provided)) {
        respondJson(401, ['ok' => false, 'error' => 'invalid_admin_token']);
    }
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
