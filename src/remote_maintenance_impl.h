// Included after the existing HTTPS helpers. Maintenance runs only in standby.
static bool remoteMaintenanceAllowed() {
  return logic::maintenance::allowed(g_measActive, g_measTask != nullptr,
    g_measAutoRestartPending, g_measAutoRestartWaitingUpload, otaInProgress,
    g_pendingRemoteReboot);
}

static bool uploadRemoteLogs(String& result) {
  const String ingest = cloudIngestUrlFromServerUrl();
  if (!cfg.cloudEnabled || !ingest.length()) { result = "logs_cloud_disabled"; return false; }
  const String capture = "boot_" + String(g_bootCount) + "_" + isoNowFileSafe();
  String body = "{\"captured_at\":\"" + jsonEscape(isoNow())
    + "\",\"restart_history\":" + g_restartHistory
    + ",\"ram_log\":\"" + jsonEscape(ramLogText()) + "\"}";
  int code = -1; String response, error;
  const String separator = ingest.indexOf('?') >= 0 ? "&" : "?";
  bool summaryOk = httpsPostJson(ingest + separator + "upload=1&name="
    + urlEncode("diagnostics/" + capture + "/restart.json"), cfg.apiKey, body, code, response, error);
  if (!summaryOk) { result = "logs_summary_upload_failed"; return false; }
  // Even with an unavailable SD card, restart history and the RAM log are accessible.
  if (!storageCanUseSd()) { result = "logs_uploaded_ram_only"; return true; }
  bool copied[MAX_LOG_FILES] = {};
  bool copyFailed = false;
  {
    ScopedSpiBusLock lock;
    if (!lock) { result = "logs_snapshot_lock_failed"; return false; }
    prepareSdSpi();
    if (!SD.exists("/diagnostics") && !SD.mkdir("/diagnostics")) {
      result = "logs_snapshot_directory_failed"; return false;
    }
    uint8_t buffer[1024];
    for (int i = 0; i < MAX_LOG_FILES; ++i) {
      const String source = "/logs/log" + String(i) + ".log";
      const String snapshot = "/diagnostics/log" + String(i) + ".log";
      if (!SD.exists(source)) continue;
      File in = SD.open(source, FILE_READ);
      if (SD.exists(snapshot)) SD.remove(snapshot);
      File out = SD.open(snapshot, FILE_WRITE);
      bool ok = in && out;
      size_t remaining = in ? in.size() : 0;
      // Account for the final line that can push a log beyond its rotation threshold.
      if (remaining > MAX_LOG_SIZE + 16384) ok = false;
      while (ok && remaining) {
        size_t want = remaining < sizeof(buffer) ? remaining : sizeof(buffer);
        int n = in.read(buffer, want);
        if (n <= 0 || out.write(buffer, n) != static_cast<size_t>(n)) { ok = false; break; }
        remaining -= n;
        delay(1);
      }
      in.close(); out.close();
      copied[i] = ok;
      if (!ok) { copyFailed = true; SD.remove(snapshot); }
    }
  }
  unsigned uploaded = 0, failed = copyFailed ? 1 : 0;
  for (int i = MAX_LOG_FILES - 1; i >= 0; --i) {
    if (!copied[i]) continue;
    const String snapshot = "/diagnostics/log" + String(i) + ".log";
    // Existing uploader maps this dedicated directory into the diagnostic browser folder.
    if (uploadCloudFilePath(ingest, snapshot)) ++uploaded;
    else ++failed;
    ScopedSpiBusLock lock;
    if (lock) { prepareSdSpi(); SD.remove(snapshot); }
  }
  result = "logs_uploaded_" + String(uploaded) + "_failed_" + String(failed);
  return failed == 0;
}

static bool maintenanceReadLine(String& line, uint32_t began) {
  line = "";
  const uint32_t waiting = millis();
  while (millis() - waiting < 15000UL && millis() - began < 600000UL) {
    if (_tls.available()) {
      int c = _tls.read();
      if (c < 0) continue;
      if (c == '\n') { line.trim(); return true; }
      if (line.length() >= 512) return false;
      line += char(c);
    } else {
      if (!_tls.connected()) return false;
      delay(1);
    }
  }
  return false;
}

static bool installRemoteFirmware(const String& params, String& result) {
  // Params are JSON inside the signed command; no URL or credentials supplied by a third party.
  String path, expected;
  if (!jsonGetStringField(params, "path", path) || !jsonGetStringField(params, "sha256", expected)
      || !logic::maintenance::firmwarePath(path.c_str()) || !logic::maintenance::sha256Hex(expected.c_str())) {
    result = "ota_invalid_path_or_sha256"; return false;
  }
  expected.toLowerCase();
  const esp_partition_t* partition = esp_ota_get_next_update_partition(nullptr);
  if (!partition) { result = "ota_no_inactive_partition"; return false; }
  ScopedSpiBusLock lock;
  if (!lock) { result = "ota_spi_lock_failed"; return false; }
  otaInProgress = true;
  noteNetOp("remote_ota");
  noteRuntimeStage("remote_ota");
  mbedtls_sha256_context sha;
  mbedtls_sha256_init(&sha);
  bool begun = false;
  auto fail = [&](const String& reason) {
    if (begun) Update.abort();
    _tls.stop();
    mbedtls_sha256_free(&sha);
    otaInProgress = false;
    result = reason;
    noteNetResult(false, result);
    return false;
  };
  String scheme, host, basePath; uint16_t port;
  if (!parseUrl(cfg.serverUrl, scheme, host, port, basePath) || scheme != "https" || port != 443)
    return fail("ota_invalid_production_server");
  prepareEthernetSpi();
  String error;
  if (!tlsConnectHost(host, port, error)) return fail("ota_connect_failed_" + error);
  // No redirects and no API key sent to a firmware download. The signed SHA-256 pins the image.
  const String request = "GET " + path + " HTTP/1.1\r\nHost: " + host
    + "\r\nAccept-Encoding: identity\r\nConnection: close\r\n\r\n";
  if (_tls.print(request) != request.length()) return fail("ota_request_failed");
  const uint32_t beganAt = millis();
  String line;
  if (!maintenanceReadLine(line, beganAt)
      || !(line.startsWith("HTTP/1.1 200 ") || line.startsWith("HTTP/1.0 200 ")))
    return fail("ota_http_status_invalid");
  uint32_t length = 0;
  bool haveLength = false, headersDone = false;
  size_t headerBytes = 0;
  for (unsigned n = 0; n < 48; ++n) {
    if (!maintenanceReadLine(line, beganAt)) return fail("ota_headers_timeout");
    headerBytes += line.length();
    if (headerBytes > 8192) return fail("ota_headers_too_large");
    if (!line.length()) { headersDone = true; break; }
    int colon = line.indexOf(':');
    if (colon <= 0) return fail("ota_header_invalid");
    String key = line.substring(0, colon); key.toLowerCase();
    String value = line.substring(colon + 1); value.trim();
    if (key == "content-length") {
      if (haveLength || !logic::maintenance::contentLength(value.c_str(), partition->size, length))
        return fail("ota_length_invalid");
      haveLength = true;
    }
    if (key == "transfer-encoding" || (key == "content-encoding" && value != "identity"))
      return fail("ota_encoding_unsupported");
  }
  if (!headersDone || !haveLength || length < 32) return fail("ota_length_missing");
  if (!Update.begin(length, U_FLASH)) return fail("ota_begin_failed_" + String(Update.errorString()));
  begun = true;
  if (mbedtls_sha256_starts_ret(&sha, 0) != 0) return fail("ota_hash_init_failed");
  uint8_t buffer[1024];
  uint32_t received = 0, lastRx = millis();
  while (received < length) {
    if (millis() - beganAt >= 600000UL || millis() - lastRx >= 15000UL)
      return fail("ota_download_timeout");
    int available = _tls.available();
    if (available <= 0) {
      if (!_tls.connected()) return fail("ota_download_truncated");
      delay(1); continue;
    }
    size_t want = length - received;
    if (want > sizeof(buffer)) want = sizeof(buffer);
    if (want > static_cast<size_t>(available)) want = available;
    int n = _tls.read(buffer, want);
    if (n <= 0) { delay(1); continue; }
    if (received == 0 && buffer[0] != 0xE9) return fail("ota_not_esp32_image");
    if (mbedtls_sha256_update_ret(&sha, buffer, n) != 0) return fail("ota_hash_failed");
    if (Update.write(buffer, n) != static_cast<size_t>(n))
      return fail("ota_flash_write_failed_" + String(Update.errorString()));
    received += n;
    lastRx = millis();
    delay(1);
  }
  uint8_t digest[32];
  if (mbedtls_sha256_finish_ret(&sha, digest) != 0 || toHexLower(digest, sizeof(digest)) != expected)
    return fail("ota_sha256_mismatch");
  // The inactive slot becomes bootable only after the entire image and digest validate.
  if (!Update.end(false)) return fail("ota_finalize_failed_" + String(Update.errorString()));
  begun = false;
  _tls.stop();
  mbedtls_sha256_free(&sha);
  otaInProgress = false;
  noteNetResult(true);
  result = "ota_installed_reboot_scheduled";
  scheduleDeferredReboot("[OTA] remote verified firmware installed", 1500);
  return true;
}
