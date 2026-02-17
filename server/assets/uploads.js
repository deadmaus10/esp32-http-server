(() => {
  const boot = window.UPLOADS_BOOTSTRAP || {};
  const knownDeviceIds = normalizeDeviceIds(boot.deviceIds, boot.deviceId);

  const state = {
    deviceId: "",
    deviceIds: knownDeviceIds,
    limit: 20,
    beforeId: null,
    hasMore: false,
    nextBeforeId: null,
    pageIndex: 1,
    cursorStack: [],
    rowsCount: 0,
    inFlight: false,
    actionInFlight: false,
  };

  const apiUploadsTemplate = String(
    boot.apiUploadsUrlTemplate || "/admin/devices/{device_id}/uploads"
  );

  const els = {
    pageTitle: byId("uploadsPageTitle"),
    deviceIdLabel: byId("uploadsDeviceIdLabel"),
    deviceSelect: byId("uploadsDeviceSelect"),
    lastRefreshLabel: byId("uploadsLastRefreshLabel"),
    tokenInput: byId("uploadsTokenInput"),
    tokenState: byId("uploadsTokenState"),
    unlockBtn: byId("uploadsUnlockBtn"),
    clearTokenBtn: byId("uploadsClearTokenBtn"),
    alertBox: byId("uploadsAlertBox"),
    refreshBtn: byId("uploadsRefreshBtn"),
    rows: byId("uploadRows"),
    pageNewerBtn: byId("uploadsPageNewerBtn"),
    pageOlderBtn: byId("uploadsPageOlderBtn"),
    pageInfo: byId("uploadsPageInfo"),
    dashboardLink: byId("uploadsDashboardLink"),
  };

  init();

  function init() {
    if (els.pageTitle && boot.dashboardTitle) {
      els.pageTitle.textContent = `${String(boot.dashboardTitle)} - Upload Browser`;
    }

    initializeDeviceSelection();

    const savedToken = sessionStorage.getItem("remote_dashboard_token") || "";
    if (savedToken && els.tokenInput) {
      els.tokenInput.value = savedToken;
    }

    bindEvents();
    updateTokenState();
    setControlsEnabled(hasToken() && !!state.deviceId);
    updatePager();

    if (!state.deviceId) {
      setAlert("error", "Dashboard device is not configured on server.");
      return;
    }

    if (hasToken()) {
      refreshNow();
    }
  }

  function bindEvents() {
    if (els.unlockBtn) {
      els.unlockBtn.addEventListener("click", () => {
        const token = normalizeTokenInput(els.tokenInput?.value || "");
        if (!token) {
          setAlert("error", "Enter admin token first.");
          updateTokenState();
          return;
        }
        sessionStorage.setItem("remote_dashboard_token", token);
        resetPagination();
        updateTokenState();
        setControlsEnabled(true);
        refreshNow();
      });
    }

    if (els.clearTokenBtn) {
      els.clearTokenBtn.addEventListener("click", () => {
        sessionStorage.removeItem("remote_dashboard_token");
        if (els.tokenInput) {
          els.tokenInput.value = "";
        }
        resetPagination();
        updateTokenState();
        setControlsEnabled(false);
        setAlert("info", "Token cleared.");
      });
    }

    if (els.refreshBtn) {
      els.refreshBtn.addEventListener("click", () => {
        refreshNow();
      });
    }

    if (els.pageOlderBtn) {
      els.pageOlderBtn.addEventListener("click", () => {
        goOlderPage();
      });
    }

    if (els.pageNewerBtn) {
      els.pageNewerBtn.addEventListener("click", () => {
        goNewerPage();
      });
    }

    if (els.deviceSelect) {
      els.deviceSelect.addEventListener("change", () => {
        const nextDeviceId = String(els.deviceSelect.value || "").trim();
        if (!nextDeviceId || nextDeviceId === state.deviceId) {
          return;
        }
        state.deviceId = nextDeviceId;
        sessionStorage.setItem("remote_dashboard_device_id", state.deviceId);
        updateDeviceDependentUi();
        resetPagination();
        setControlsEnabled(hasToken() && !!state.deviceId);
        refreshNow();
      });
    }

    if (els.rows) {
      els.rows.addEventListener("click", (ev) => {
        const target = ev.target;
        if (!(target instanceof HTMLElement)) {
          return;
        }

        const downloadBtn = target.closest("button[data-download-url]");
        if (downloadBtn instanceof HTMLButtonElement) {
          const url = String(downloadBtn.getAttribute("data-download-url") || "");
          const filename = String(downloadBtn.getAttribute("data-filename") || "upload.bin");
          if (!url) {
            return;
          }
          downloadUpload(url, filename, downloadBtn);
          return;
        }

        const deleteBtn = target.closest("button[data-delete-url]");
        if (!(deleteBtn instanceof HTMLButtonElement)) {
          return;
        }
        const deleteUrl = String(deleteBtn.getAttribute("data-delete-url") || "");
        const filename = String(deleteBtn.getAttribute("data-filename") || "upload.bin");
        if (!deleteUrl) {
          return;
        }
        deleteUpload(deleteUrl, filename, deleteBtn);
      });
    }
  }

  function initializeDeviceSelection() {
    if (els.deviceSelect) {
      els.deviceSelect.innerHTML = "";
      state.deviceIds.forEach((deviceId) => {
        const opt = document.createElement("option");
        opt.value = deviceId;
        opt.textContent = deviceId;
        els.deviceSelect.appendChild(opt);
      });
    }

    const savedDeviceId = normalizeDeviceId(
      sessionStorage.getItem("remote_dashboard_device_id") || ""
    );
    state.deviceId = pickInitialDeviceId(
      savedDeviceId,
      normalizeDeviceId(boot.deviceId),
      state.deviceIds
    );

    if (els.deviceSelect) {
      if (state.deviceId) {
        els.deviceSelect.value = state.deviceId;
      }
      els.deviceSelect.disabled = state.deviceIds.length <= 1;
    }

    if (state.deviceId) {
      sessionStorage.setItem("remote_dashboard_device_id", state.deviceId);
    }

    updateDeviceDependentUi();
  }

  function updateDeviceDependentUi() {
    if (els.deviceIdLabel) {
      els.deviceIdLabel.textContent = state.deviceId || "(not configured)";
    }
    if (els.dashboardLink) {
      els.dashboardLink.href = withDeviceQuery(
        String(boot.dashboardUrl || "/dashboard"),
        state.deviceId
      );
    }
  }

  async function refreshNow() {
    if (!state.deviceId || state.inFlight || !hasToken()) {
      return;
    }

    state.inFlight = true;
    updatePager();
    setControlsBusy(true);

    try {
      const payload = await apiRequest(buildUploadsUrl(), { method: "GET" });
      const uploads = Array.isArray(payload?.uploads) ? payload.uploads : [];
      renderUploads(uploads);
      applyPage(payload?.page || null, uploads.length);
      setLastRefresh(new Date());
      setControlsEnabled(true);
    } catch (err) {
      handleError(err, "Failed to load uploads.");
    } finally {
      state.inFlight = false;
      setControlsBusy(false);
      updatePager();
    }
  }

  function renderUploads(rows) {
    if (!els.rows) {
      return;
    }

    if (!Array.isArray(rows) || rows.length === 0) {
      els.rows.innerHTML =
        '<tr><td colspan="6" class="muted no-commands-cell" data-label="Info">No uploads yet.</td></tr>';
      return;
    }

    els.rows.innerHTML = rows
      .map((row) => {
        const id = String(row.id || "");
        const filename = String(row.filename || "upload.bin");
        const originalName = String(row.filename_original || "");
        const bytes = Number.isFinite(row.bytes) ? row.bytes : 0;
        const receivedAt = formatDate(row.received_at || "");
        const downloadUrl = String(row.download_url || "");
        const deleteUrl = String(row.delete_url || "");
        const status = uploadStorageStatus(row);
        const originalLine = originalName && originalName !== filename
          ? `<br><span class="muted tiny">source: ${escapeHtml(originalName)}</span>`
          : "";

        return `
          <tr data-row-id="${escapeHtml(id)}">
            <td data-label="ID">#${escapeHtml(id)}</td>
            <td data-label="Filename">${escapeHtml(filename)}${originalLine}</td>
            <td data-label="Status"><span class="tag ${status.className}">${status.label}</span></td>
            <td data-label="Size">${escapeHtml(formatBytes(bytes))}</td>
            <td data-label="Received">${escapeHtml(receivedAt)}</td>
            <td data-label="Actions" class="action-cell">
              <div class="action-buttons">
              <button
                type="button"
                class="btn btn-primary btn-mini"
                data-download-url="${escapeHtml(downloadUrl)}"
                data-filename="${escapeHtml(filename)}"
              >Download</button>
              <button
                type="button"
                class="btn btn-danger btn-mini"
                data-delete-url="${escapeHtml(deleteUrl)}"
                data-filename="${escapeHtml(filename)}"
              >Delete</button>
              </div>
            </td>
          </tr>
        `;
      })
      .join("");
  }

  function applyPage(page, visibleCount) {
    state.rowsCount = Number.isFinite(visibleCount) ? visibleCount : 0;

    if (page && typeof page === "object") {
      state.beforeId = normalizePageCursor(page.before_id);
      state.hasMore = page.has_more === true;
      state.nextBeforeId = state.hasMore ? normalizePageCursor(page.next_before_id) : null;
    } else {
      state.hasMore = false;
      state.nextBeforeId = null;
    }

    updatePager();
  }

  function goOlderPage() {
    if (
      state.inFlight ||
      !state.hasMore ||
      !state.nextBeforeId ||
      !state.deviceId ||
      !hasToken()
    ) {
      return;
    }

    state.cursorStack.push(state.beforeId);
    state.beforeId = state.nextBeforeId;
    state.pageIndex += 1;
    updatePager();
    refreshNow();
  }

  function goNewerPage() {
    if (state.inFlight || state.pageIndex <= 1 || !state.deviceId || !hasToken()) {
      return;
    }

    const previousBeforeId = state.cursorStack.pop();
    state.beforeId =
      typeof previousBeforeId === "string" && previousBeforeId !== ""
        ? previousBeforeId
        : null;
    state.pageIndex = Math.max(1, state.pageIndex - 1);
    updatePager();
    refreshNow();
  }

  function resetPagination() {
    state.beforeId = null;
    state.hasMore = false;
    state.nextBeforeId = null;
    state.pageIndex = 1;
    state.cursorStack = [];
    state.rowsCount = 0;
    updatePager();
  }

  function updatePager() {
    if (els.pageInfo) {
      const suffix = state.rowsCount > 0 ? ` (${state.rowsCount} shown)` : "";
      els.pageInfo.textContent = `Page ${state.pageIndex}${suffix}`;
    }

    const hasNewer = state.pageIndex > 1;
    const canGoOlder = state.hasMore && !!state.nextBeforeId && state.rowsCount > 0;

    if (els.pageNewerBtn) {
      els.pageNewerBtn.disabled =
        !hasToken() || !state.deviceId || state.inFlight || state.actionInFlight || !hasNewer;
    }
    if (els.pageOlderBtn) {
      els.pageOlderBtn.disabled =
        !hasToken() || !state.deviceId || state.inFlight || state.actionInFlight || !canGoOlder;
    }
  }

  async function downloadUpload(url, filename, buttonEl) {
    if (!hasToken()) {
      updateTokenState();
      setAlert("error", "Token required before downloading files.");
      return;
    }

    if (buttonEl) {
      buttonEl.disabled = true;
    }
    state.actionInFlight = true;
    updatePager();

    try {
      const token = normalizeTokenInput(
        sessionStorage.getItem("remote_dashboard_token") || ""
      );
      const res = await fetch(url, {
        method: "GET",
        cache: "no-store",
        headers: {
          "X-ADMIN-TOKEN": token,
          "X-API-KEY": token,
          Authorization: `Bearer ${token}`,
        },
      });

      if (res.status === 401) {
        sessionStorage.removeItem("remote_dashboard_token");
        if (els.tokenInput) {
          els.tokenInput.value = "";
        }
        updateTokenState();
        setControlsEnabled(false);
        throw new Error("Unauthorized: invalid token");
      }

      if (!res.ok) {
        let message = `HTTP ${res.status}`;
        try {
          const errJson = await res.json();
          if (errJson && errJson.error) {
            message = String(errJson.error);
          }
        } catch (e) {
          // keep default message
        }
        throw new Error(message);
      }

      const blob = await res.blob();
      const objectUrl = window.URL.createObjectURL(blob);
      const anchor = document.createElement("a");
      anchor.href = objectUrl;
      anchor.download = safeDownloadName(filename);
      document.body.appendChild(anchor);
      anchor.click();
      anchor.remove();
      window.URL.revokeObjectURL(objectUrl);
      setAlert("success", `Downloaded: ${filename}`);
    } catch (err) {
      handleError(err, "Download failed.");
    } finally {
      if (buttonEl) {
        buttonEl.disabled = false;
      }
      state.actionInFlight = false;
      updatePager();
    }
  }

  async function deleteUpload(url, filename, buttonEl) {
    if (!hasToken()) {
      updateTokenState();
      setAlert("error", "Token required before deleting files.");
      return;
    }
    if (state.actionInFlight) {
      return;
    }

    const ok = window.confirm(
      `Delete ${filename} from server uploads and database? This cannot be undone.`
    );
    if (!ok) {
      return;
    }

    if (buttonEl) {
      buttonEl.disabled = true;
    }
    state.actionInFlight = true;
    updatePager();

    try {
      await apiRequest(url, { method: "POST", body: {} });
      setAlert("success", `Deleted: ${filename}`);
      refreshNow();
    } catch (err) {
      handleError(err, "Delete failed.");
    } finally {
      if (buttonEl) {
        buttonEl.disabled = false;
      }
      state.actionInFlight = false;
      updatePager();
    }
  }

  async function apiRequest(url, options) {
    const token = normalizeTokenInput(
      sessionStorage.getItem("remote_dashboard_token") || ""
    );
    if (!token) {
      const err = new Error("Missing admin token");
      err.code = "NO_TOKEN";
      throw err;
    }

    const headers = {
      Accept: "application/json",
      "X-ADMIN-TOKEN": token,
      "X-API-KEY": token,
      Authorization: `Bearer ${token}`,
    };

    const init = {
      method: options?.method || "GET",
      cache: "no-store",
      headers,
    };

    if (options?.body !== undefined) {
      headers["Content-Type"] = "application/json";
      init.body = JSON.stringify(options.body);
    }

    const res = await fetch(url, init);
    let data = null;
    try {
      data = await res.json();
    } catch (e) {
      data = null;
    }

    if (res.status === 401) {
      sessionStorage.removeItem("remote_dashboard_token");
      if (els.tokenInput) {
        els.tokenInput.value = "";
      }
      updateTokenState();
      setControlsEnabled(false);
      const err = new Error("Unauthorized: invalid token");
      err.code = 401;
      throw err;
    }

    if (!res.ok) {
      const message = data?.error ? String(data.error) : `HTTP ${res.status}`;
      const err = new Error(message);
      err.code = res.status;
      throw err;
    }

    return data;
  }

  function buildUploadsUrl() {
    const base = withDevice(apiUploadsTemplate, state.deviceId);
    const params = new URLSearchParams();
    params.set("limit", String(state.limit));
    if (state.beforeId) {
      params.set("before_id", state.beforeId);
    }
    return `${base}${base.includes("?") ? "&" : "?"}${params.toString()}`;
  }

  function withDevice(template, deviceId) {
    return template.replace("{device_id}", encodeURIComponent(deviceId));
  }

  function withDeviceQuery(baseUrl, deviceId) {
    const clean = String(baseUrl || "").trim() || "/";
    try {
      const url = new URL(clean, window.location.origin);
      if (deviceId) {
        url.searchParams.set("device_id", deviceId);
      } else {
        url.searchParams.delete("device_id");
      }
      return `${url.pathname}${url.search}${url.hash}`;
    } catch (e) {
      if (!deviceId) {
        return clean;
      }
      const joiner = clean.includes("?") ? "&" : "?";
      return `${clean}${joiner}device_id=${encodeURIComponent(deviceId)}`;
    }
  }

  function setControlsBusy(isBusy) {
    [els.refreshBtn].forEach((btn) => {
      if (btn) {
        btn.disabled = isBusy;
      }
    });
    updatePager();
  }

  function setControlsEnabled(enabled) {
    [els.refreshBtn].forEach((btn) => {
      if (btn) {
        btn.disabled = !enabled;
      }
    });
    updatePager();
  }

  function updateTokenState() {
    if (!els.tokenState) {
      return;
    }
    if (hasToken()) {
      els.tokenState.textContent = "Token loaded. File browser is enabled.";
    } else {
      els.tokenState.textContent = "Locked. Enter token to load files.";
    }
    updatePager();
  }

  function hasToken() {
    const token = normalizeTokenInput(
      sessionStorage.getItem("remote_dashboard_token") || ""
    );
    return token.length > 0;
  }

  function normalizeTokenInput(value) {
    let token = String(value || "").trim();
    if (
      token.length >= 2 &&
      ((token.startsWith('"') && token.endsWith('"')) ||
        (token.startsWith("'") && token.endsWith("'")))
    ) {
      token = token.slice(1, -1).trim();
    }
    return token;
  }

  function normalizePageCursor(value) {
    if (value === null || value === undefined) {
      return null;
    }
    const text = String(value).trim();
    if (!/^\d+$/.test(text) || text === "0") {
      return null;
    }
    return text;
  }

  function normalizeDeviceId(value) {
    return String(value || "").trim();
  }

  function normalizeDeviceIds(values, fallbackDeviceId) {
    const out = [];
    const seen = new Set();

    if (Array.isArray(values)) {
      values.forEach((value) => {
        const id = normalizeDeviceId(value);
        if (!id || id === "replace-with-device-id" || seen.has(id)) {
          return;
        }
        seen.add(id);
        out.push(id);
      });
    }

    const fallback = normalizeDeviceId(fallbackDeviceId);
    if (fallback && !seen.has(fallback)) {
      out.push(fallback);
    }

    return out;
  }

  function pickInitialDeviceId(savedDeviceId, bootDeviceId, knownIds) {
    if (savedDeviceId && knownIds.includes(savedDeviceId)) {
      return savedDeviceId;
    }
    if (bootDeviceId && knownIds.includes(bootDeviceId)) {
      return bootDeviceId;
    }
    if (knownIds.length > 0) {
      return knownIds[0];
    }
    return "";
  }

  function setLastRefresh(date) {
    if (!els.lastRefreshLabel) {
      return;
    }
    els.lastRefreshLabel.textContent = formatDate(date.toISOString());
  }

  function setAlert(type, message) {
    if (!els.alertBox) {
      return;
    }
    els.alertBox.classList.remove("hidden", "info", "success", "error");
    els.alertBox.classList.add(type || "info");
    els.alertBox.textContent = message;
  }

  function handleError(err, prefix) {
    const suffix = err?.message ? ` ${err.message}` : "";
    setAlert("error", `${prefix}${suffix}`.trim());
  }

  function formatDate(value) {
    if (!value) {
      return "--";
    }
    const dt = new Date(value);
    if (Number.isNaN(dt.getTime())) {
      return String(value);
    }
    return dt.toLocaleString();
  }

  function formatBytes(bytes) {
    const value = Math.max(0, Number(bytes) || 0);
    if (value < 1024) {
      return `${value} B`;
    }
    const kb = value / 1024;
    if (kb < 1024) {
      return `${kb.toFixed(1)} KB`;
    }
    const mb = kb / 1024;
    if (mb < 1024) {
      return `${mb.toFixed(2)} MB`;
    }
    const gb = mb / 1024;
    return `${gb.toFixed(2)} GB`;
  }

  function uploadStorageStatus(row) {
    const status = String(row?.storage_status || "").toLowerCase();
    if (status === "missing" || row?.file_exists === false) {
      return { label: "MISSING", className: "fail" };
    }
    return { label: "PRESENT", className: "ok" };
  }

  function safeDownloadName(value) {
    const raw = String(value || "upload.bin").trim();
    const sanitized = raw.replace(/[^A-Za-z0-9._-]/g, "_");
    if (!sanitized) {
      return "upload.bin";
    }
    return sanitized;
  }

  function byId(id) {
    return document.getElementById(id);
  }

  function escapeHtml(input) {
    return String(input)
      .replace(/&/g, "&amp;")
      .replace(/</g, "&lt;")
      .replace(/>/g, "&gt;")
      .replace(/\"/g, "&quot;")
      .replace(/'/g, "&#39;");
  }
})();
