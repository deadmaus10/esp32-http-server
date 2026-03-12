#!/usr/bin/env php
<?php
declare(strict_types=1);

$config = require dirname(__DIR__) . '/config.php';
$localConfig = dirname(__DIR__) . '/config.local.php';
if (is_file($localConfig)) {
    $override = require $localConfig;
    if (is_array($override)) {
        $config = array_replace_recursive($config, $override);
    }
}

$args = parseArgs($argv);
$deviceId = trim((string)($args['device'] ?? ''));
$action = trim((string)($args['action'] ?? ''));
$params = trim((string)($args['params'] ?? ''));

if ($deviceId === '' || $action === '') {
    fwrite(STDERR, "Usage: php scripts/enqueue_command.php --device <id> --action <measure_start|measure_stop|reboot> [--params 'rate=920']\n");
    exit(1);
}

$devices = is_array($config['devices'] ?? null) ? $config['devices'] : [];
if (!isset($devices[$deviceId])) {
    fwrite(STDERR, "Unknown device: {$deviceId}\n");
    exit(1);
}

$allowed = ['measure_start', 'measure_stop', 'reboot'];
if (!in_array($action, $allowed, true)) {
    fwrite(STDERR, "Unsupported action: {$action}\n");
    exit(1);
}

$cmdSecret = (string)($devices[$deviceId]['cmd_secret'] ?? '');
if ($cmdSecret === '') {
    fwrite(STDERR, "Missing cmd_secret for {$deviceId}\n");
    exit(1);
}

$dbPath = (string)($config['db_path'] ?? (dirname(__DIR__) . '/storage/remote.sqlite'));
$pdo = new PDO('sqlite:' . $dbPath);
$pdo->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
$pdo->setAttribute(PDO::ATTR_DEFAULT_FETCH_MODE, PDO::FETCH_ASSOC);

$issuedAt = gmdate('c');
$nonce = bin2hex(random_bytes(12));
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

    $id = (string)$pdo->lastInsertId();
    $canonical = canonicalCommandPayload($id, $action, $params, $issuedAt, $nonce);
    $sig = hash_hmac('sha256', $canonical, $cmdSecret);

    $updateStmt = $pdo->prepare('UPDATE commands SET sig = :sig WHERE id = :id');
    $updateStmt->execute([
        ':sig' => $sig,
        ':id' => (int)$id,
    ]);

    $pdo->commit();
} catch (Throwable $e) {
    if ($pdo->inTransaction()) {
        $pdo->rollBack();
    }
    fwrite(STDERR, 'Failed: ' . $e->getMessage() . "\n");
    exit(1);
}

echo json_encode([
    'ok' => true,
    'device_id' => $deviceId,
    'id' => $id,
    'action' => $action,
    'params' => $params,
    'issued_at' => $issuedAt,
    'nonce' => $nonce,
], JSON_UNESCAPED_SLASHES) . "\n";

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
    $encoded = json_encode($value, JSON_UNESCAPED_SLASHES | JSON_UNESCAPED_UNICODE | JSON_INVALID_UTF8_SUBSTITUTE);
    if (!is_string($encoded) || strlen($encoded) < 2) {
        return '';
    }
    return substr($encoded, 1, -1);
}

function parseArgs(array $argv): array
{
    $out = [];
    for ($i = 1; $i < count($argv); $i++) {
        $arg = (string)$argv[$i];
        if (!str_starts_with($arg, '--')) {
            continue;
        }

        $arg = substr($arg, 2);
        $parts = explode('=', $arg, 2);
        if (count($parts) === 2) {
            $out[$parts[0]] = $parts[1];
            continue;
        }

        $key = $parts[0];
        $value = '';
        if (isset($argv[$i + 1]) && !str_starts_with((string)$argv[$i + 1], '--')) {
            $value = (string)$argv[$i + 1];
            $i++;
        }
        $out[$key] = $value;
    }
    return $out;
}
