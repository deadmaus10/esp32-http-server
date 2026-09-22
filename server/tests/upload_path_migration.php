<?php
declare(strict_types=1);

// Load only the resolver, without initializing the production database/routes.
$source = file_get_contents(__DIR__ . '/../index.php');
$start = strpos($source, 'function resolveUploadStoredPath(');
$end = strpos($source, 'function uploadRelativePathFromRow(', $start);
eval(substr($source, $start, $end - $start));

$root = sys_get_temp_dir() . '/upload-migration-' . bin2hex(random_bytes(6));
$uploads = $root . '/storage/uploads';
mkdir($uploads . '/device/session', 0700, true);
$file = $uploads . '/device/session/part.am1';
file_put_contents($file, 'measurement');
file_put_contents($root . '/outside', 'private');
symlink($root . '/outside', $uploads . '/escape');

try {
    $cases = [
        [$file, realpath($file)],
        ['/old/account/storage/uploads/device/session/part.am1', realpath($file)],
        ['/old/account/storage/uploads/device/session/missing.am1', null],
        ['/old/account/storage/uploads/../../outside', null],
        ['/old/account/storage/uploads/escape', null],
        [$root . '/outside', null],
        [$uploads, null],
        ['', null],
    ];
    foreach ($cases as [$path, $expected]) {
        if (resolveUploadStoredPath($path, $uploads) !== $expected) {
            throw new RuntimeException('Unexpected result for ' . $path);
        }
    }
    echo "Upload migration: 8 checks passed.\n";
} finally {
    unlink($uploads . '/escape');
    unlink($root . '/outside');
    unlink($file);
    rmdir($uploads . '/device/session');
    rmdir($uploads . '/device');
    rmdir($uploads);
    rmdir($root . '/storage');
    rmdir($root);
}
