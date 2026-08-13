<?php
declare(strict_types=1);

return [
    // Change to your deployment path. Use '' if app is served from domain root.
    'base_path' => '/remote',

    // Keep timestamps in UTC for consistency with firmware-issued timestamps.
    'timezone' => 'UTC',

    // cPanel/Apache needs write access to this file and upload directory.
    'db_path' => __DIR__ . '/storage/remote.sqlite',
    'upload_dir' => __DIR__ . '/storage/uploads',

    // Set a strong random token, then use it in X-ADMIN-TOKEN or Bearer auth.
    'admin_token' => '__CHANGE_ME__',

    // Customer dashboard settings (GET /dashboard).
    // If dashboard_device_id is empty, first configured device is used.
    'dashboard_device_id' => '',
    'offline_after_sec' => 180,
    'dashboard_poll_sec' => 5,
    'dashboard_title' => 'Device Remote Control',

    // Public coming-soon page contact address (GET /).
    'contact_email' => 'info@albasqueeze.hu',

    // Device credentials keyed by device_id from firmware config.
    // api_key must match cfg.apiKey on the device.
    // cmd_secret must match cfg.cmdSecret on the device.
    'devices' => [
        'replace-with-device-id' => [
            'api_key' => 'replace-with-device-api-key',
            'cmd_secret' => 'replace-with-device-command-secret',
        ],
    ],
];
