"""Exercise the real PHP command routes in an isolated server with test credentials."""
import hashlib
import hmac
import json
import pathlib
import shutil
import socket
import subprocess
import tempfile
import time
import urllib.error
import urllib.request

root = pathlib.Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='albasqueeze-backend-') as temporary:
    directory = pathlib.Path(temporary)
    shutil.copy(root / 'server/index.php', directory / 'index.php')
    (directory / 'config.php').write_text("<?php return ['admin_token'=>'test-admin','devices'=>['test-device'=>['api_key'=>'test-api','cmd_secret'=>'test-secret']]];")
    with socket.socket() as reserve:
        reserve.bind(('127.0.0.1', 0))
        port = reserve.getsockname()[1]
    with open(directory / 'php.log', 'w') as output:
        server = subprocess.Popen(['php', '-S', f'127.0.0.1:{port}', '-t', temporary, str(directory / 'index.php')], stdout=output, stderr=output)
    try:
        def request(route, payload=None, admin=True):
            headers = {'Content-Type': 'application/json'}
            if admin:
                headers['X-ADMIN-TOKEN'] = 'test-admin'
            req = urllib.request.Request(f'http://127.0.0.1:{port}'+route,
                data=None if payload is None else json.dumps(payload).encode(), headers=headers)
            try:
                with urllib.request.urlopen(req, timeout=3) as response:
                    return response.status, json.load(response)
            except urllib.error.HTTPError as error:
                return error.code, json.load(error)
        for attempt in range(40):
            try:
                status, _ = request('/health')
                assert status == 200
                break
            except (OSError, AssertionError):
                time.sleep(0.1)
        else:
            raise AssertionError((directory / 'php.log').read_text())
        def upload(image, admin=True):
            headers = {'Content-Type': 'application/octet-stream'}
            if admin:
                headers['X-ADMIN-TOKEN'] = 'test-admin'
            req = urllib.request.Request(f'http://127.0.0.1:{port}/admin/firmware', data=image, headers=headers)
            try:
                with urllib.request.urlopen(req, timeout=3) as response:
                    return response.status, json.load(response)
            except urllib.error.HTTPError as error:
                return error.code, json.load(error)
        image = bytes([0xE9]) + bytes(range(255)) * 100
        assert upload(image, admin=False)[0] in (401, 403)
        assert upload(b'not an application image')[0] == 400
        assert upload(image + bytes(4 * 1024 * 1024))[0] == 413
        status, firmware = upload(image)
        assert status == 201, firmware
        assert firmware['sha256'] == hashlib.sha256(image).hexdigest()
        assert firmware['bytes'] == len(image)
        with urllib.request.urlopen(f'http://127.0.0.1:{port}' + firmware['path'], timeout=3) as response:
            assert int(response.headers['Content-Length']) == len(image)
            assert response.read() == image
        status, _ = request('/firmware/' + '0' * 32 + '.bin', admin=False)
        assert status == 404
        stored = directory / 'storage/firmware' / pathlib.Path(firmware['path']).name
        import os
        os.utime(stored, (time.time() - 8 * 86400, time.time() - 8 * 86400))
        assert request(firmware['path'], admin=False)[0] == 404
        assert upload(image)[0] == 201
        assert not stored.exists(), 'expired firmware was not cleaned up'
        route = '/admin/devices/test-device/commands'
        status, _ = request(route, {'action':'upload_logs'}, admin=False)
        assert status in (401,403), status
        for payload in [
            {'action':'ota_update','params':'bad-json'},
            {'action':'ota_update','params':json.dumps({'path':'/firmware/../bad.bin','sha256':'0'*64})},
            {'action':'ota_update','params':json.dumps({'path':'https://other.test/fw.bin','sha256':'0'*64})},
            {'action':'ota_update','params':json.dumps({'path':'/firmware/a.bin','sha256':'invalid'})},
            {'action':'ota_update','params':json.dumps({'path':[], 'sha256':'0'*64})},
        ]:
            status, response = request(route,payload)
            assert status == 400, (status,response)
        for payload in [
            {'action':'upload_logs'},
            {'action':'ota_update','params':json.dumps({'path':'/firmware/v2.bin','sha256':'0'*64},separators=(',',':'))},
        ]:
            status, response = request(route,payload)
            assert status == 201, (status,response)
            cmd = response['command']
            canonical = {key:cmd[key] for key in ['id','action','params','issued_at','nonce']}
            digest = hmac.new(b'test-secret', json.dumps(canonical,separators=(',',':')).encode(),hashlib.sha256).hexdigest()
            assert hmac.compare_digest(digest,cmd['sig']), 'command signature mismatch'
        status, response = request(route)
        assert status == 200 and len(response['commands']) == 2, response
        print('Backend auth, firmware upload/download/expiry, OTA validation, command queue and HMAC integration checks passed')
    finally:
        server.terminate()
        server.wait(timeout=5)
