(() => {
  const boot = window.UPLOADS_BOOTSTRAP || {};
  const knownDeviceIds = normalizeDeviceIds(boot.deviceIds, boot.deviceId);

  const state = {
    deviceId: "",
    deviceIds: knownDeviceIds,
    currentPath: "",
    parentPath: null,
    pageSize: 5,
    pageIndex: 1,
    pageItems: [],
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
    upBtn: byId("uploadsUpBtn"),
    pathLabel: byId("uploadsPathLabel"),
    rows: byId("uploadRows"),
    dashboardLink: byId("uploadsDashboardLink"),
    pageNewerBtn: byId("uploadsPageNewerBtn"),
    pageOlderBtn: byId("uploadsPageOlderBtn"),
    pageInfo: byId("uploadsPageInfo"),
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
    updatePathUi();
    setControlsEnabled(hasToken() && !!state.deviceId);

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
        updateTokenState();
        setControlsEnabled(false);
        resetPagination();
        setAlert("info", "Token cleared.");
      });
    }

    if (els.refreshBtn) {
      els.refreshBtn.addEventListener("click", () => {
        refreshNow();
      });
    }

    if (els.upBtn) {
      els.upBtn.addEventListener("click", () => {
        goUp();
      });
    }

    if (els.deviceSelect) {
      els.deviceSelect.addEventListener("change", () => {
        const nextDeviceId = String(els.deviceSelect.value || "").trim();
        if (!nextDeviceId || nextDeviceId === state.deviceId) {
          return;
        }
        state.deviceId = nextDeviceId;
        state.currentPath = "";
        state.parentPath = null;
        resetPagination();
        sessionStorage.setItem("remote_dashboard_device_id", state.deviceId);
        updateDeviceDependentUi();
        updatePathUi();
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

        const openBtn = target.closest("button[data-open-path]");
        if (openBtn instanceof HTMLButtonElement) {
          const nextPath = normalizeUploadPath(String(openBtn.getAttribute("data-open-path") || ""));
          navigateToPath(nextPath);
          return;
        }

        const folderDownloadBtn = target.closest("button[data-folder-download-url]");
        if (folderDownloadBtn instanceof HTMLButtonElement) {
          const url = String(folderDownloadBtn.getAttribute("data-folder-download-url") || "");
          const filename = String(folderDownloadBtn.getAttribute("data-filename") || "measurement.zip");
          if (!url) {
            return;
          }
          downloadBinary(url, filename, folderDownloadBtn, "Folder download failed.");
          return;
        }

        const downloadBtn = target.closest("button[data-download-url]");
        if (downloadBtn instanceof HTMLButtonElement) {
          const url = String(downloadBtn.getAttribute("data-download-url") || "");
          const filename = String(downloadBtn.getAttribute("data-filename") || "upload.bin");
          if (!url) {
            return;
          }
          downloadBinary(url, filename, downloadBtn, "Download failed.");
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

  function navigateToPath(pathValue) {
    if (state.inFlight || state.actionInFlight) {
      return;
    }
    state.currentPath = normalizeUploadPath(pathValue);
    resetPagination();
    updatePathUi();
    refreshNow();
  }

  function goUp() {
    if (!state.currentPath) {
      return;
    }
    const parent = parentPath(state.currentPath);
    state.currentPath = parent;
    state.parentPath = parentPath(parent);
    resetPagination();
    updatePathUi();
    refreshNow();
  }

  async function refreshNow() {
    if (!state.deviceId || state.inFlight || !hasToken()) {
      return;
    }

    state.inFlight = true;
    setControlsBusy(true);

    try {
      const payload = await apiRequest(buildUploadsUrl(), { method: "GET" });
      state.currentPath = normalizeUploadPath(String(payload?.path || state.currentPath));
      state.parentPath = normalizeParentPath(payload?.parent_path);
      renderUploads(payload);
      updatePathUi();
      setLastRefresh(new Date());
      setControlsEnabled(true);
    } catch (err) {
      handleError(err, "Failed to load uploads.");
    } finally {
      state.inFlight = false;
      setControlsBusy(false);
      updatePathUi();
    }
  }

  function renderUploads(payload) {
    if (!els.rows) {
      return;
    }

    const folders = Array.isArray(payload?.folders) ? payload.folders : [];
    const files = Array.isArray(payload?.files)
      ? payload.files
      : Array.isArray(payload?.uploads)
      ? payload.uploads
      : [];

    state.pageItems = buildUploadItems(folders, files);
    syncPagination();

    if (state.pageItems.length === 0) {
      els.rows.innerHTML =
        '<tr><td colspan="6" class="muted no-commands-cell" data-label="Info">No uploads in this folder.</td></tr>';
      updatePaginationUi();
      return;
    }

    renderCurrentPageItems();
  }

  function renderCurrentPageItems() {
    if (!els.rows) {
      return;
    }

    els.rows.innerHTML = pagedItems()
      .map((item) => (item.type === "folder" ? renderFolderRow(item) : renderFileRow(item)))
      .join("");
    updatePaginationUi();
  }

  function buildUploadItems(folders, files) {
    const items = [];

    folders.forEach((folder) => {
      items.push({
        ...folder,
        type: "folder",
        sortTime: sortableTime(folder?.received_at),
      });
    });

    files.forEach((file) => {
      items.push({
        ...file,
        type: "file",
        sortTime: sortableTime(file?.received_at),
      });
    });

    items.sort((a, b) => {
      if (a.sortTime !== b.sortTime) {
        return b.sortTime - a.sortTime;
      }
      return String(a.name || a.path || "").localeCompare(
        String(b.name || b.path || ""),
        undefined,
        { numeric: true, sensitivity: "base" }
      );
    });

    return items;
  }

  function pagedItems() {
    const start = (state.pageIndex - 1) * state.pageSize;
    return state.pageItems.slice(start, start + state.pageSize);
  }

  function totalPages() {
    return Math.max(1, Math.ceil(state.pageItems.length / state.pageSize));
  }

  function syncPagination() {
    const maxPage = totalPages();
    if (state.pageIndex > maxPage) {
      state.pageIndex = maxPage;
    }
    if (state.pageIndex < 1) {
      state.pageIndex = 1;
    }
  }

  function resetPagination() {
    state.pageIndex = 1;
    state.pageItems = [];
    updatePaginationUi();
  }

  function updatePaginationUi() {
    if (els.pageInfo) {
      const shown = state.pageItems.length === 0 ? 0 : pagedItems().length;
      els.pageInfo.textContent = `Page ${state.pageIndex} / ${totalPages()} (${shown}/${state.pageItems.length})`;
    }

    if (els.pageNewerBtn) {
      els.pageNewerBtn.disabled =
        !hasToken() ||
        state.inFlight ||
        state.actionInFlight ||
        state.pageIndex <= 1;
    }

    if (els.pageOlderBtn) {
      els.pageOlderBtn.disabled =
        !hasToken() ||
        state.inFlight ||
        state.actionInFlight ||
        state.pageItems.length === 0 ||
        state.pageIndex >= totalPages();
    }
  }

  function goOlderPage() {
    if (
      state.inFlight ||
      state.actionInFlight ||
      state.pageItems.length === 0 ||
      state.pageIndex >= totalPages()
    ) {
      return;
    }
    state.pageIndex += 1;
    renderCurrentPageItems();
  }

  function goNewerPage() {
    if (state.inFlight || state.actionInFlight || state.pageIndex <= 1) {
      return;
    }
    state.pageIndex -= 1;
    renderCurrentPageItems();
  }

  function renderFolderRow(folder) {
    const path = String(folder?.path || "");
    const name = String(folder?.name || path || "folder");
    const fileCount = Number(folder?.file_count) || 0;
    const bytes = Number(folder?.bytes) || 0;
    const receivedAt = formatDate(folder?.received_at || "");
    const status = uploadStorageStatus(folder);
    const downloadUrl = String(folder?.download_url || "");
    const downloadName = `${safeDownloadName(name)}.zip`;

    const actions = [
      `<button type="button" class="btn btn-primary btn-mini" data-open-path="${escapeHtml(path)}">Open</button>`,
    ];

    if (downloadUrl) {
      actions.push(
        `<button type="button" class="btn btn-soft btn-mini" data-folder-download-url="${escapeHtml(
          downloadUrl
        )}" data-filename="${escapeHtml(downloadName)}">Download Folder</button>`
      );
    }

    return `
      <tr data-row-type="folder" data-row-path="${escapeHtml(path)}">
        <td data-label="Type"><span class="tag pending">FOLDER</span></td>
        <td data-label="Name">${escapeHtml(name)}<br><span class="muted tiny">${escapeHtml(
      String(fileCount)
    )} file(s)</span></td>
        <td data-label="Status"><span class="tag ${status.className}">${status.label}</span></td>
        <td data-label="Size">${escapeHtml(formatBytes(bytes))}</td>
        <td data-label="Updated">${escapeHtml(receivedAt)}</td>
        <td data-label="Actions" class="action-cell"><div class="action-buttons">${actions.join(
          ""
        )}</div></td>
      </tr>
    `;
  }

  function renderFileRow(row) {
    const id = String(row?.id || "");
    const name = String(row?.name || row?.filename || "upload.bin");
    const relPath = String(row?.path || row?.filename || "");
    const bytes = Number(row?.bytes) || 0;
    const receivedAt = formatDate(row?.received_at || "");
    const downloadUrl = String(row?.download_url || "");
    const deleteUrl = String(row?.delete_url || "");
    const status = uploadStorageStatus(row);
    const pathLine = relPath && relPath !== name
      ? `<br><span class="muted tiny">${escapeHtml(relPath)}</span>`
      : "";

    const actions = [];
    if (downloadUrl) {
      actions.push(
        `<button type="button" class="btn btn-primary btn-mini" data-download-url="${escapeHtml(
          downloadUrl
        )}" data-filename="${escapeHtml(name)}">Download</button>`
      );
    }
    if (deleteUrl) {
      actions.push(
        `<button type="button" class="btn btn-danger btn-mini" data-delete-url="${escapeHtml(
          deleteUrl
        )}" data-filename="${escapeHtml(name)}">Delete</button>`
      );
    }

    return `
      <tr data-row-id="${escapeHtml(id)}" data-row-type="file">
        <td data-label="Type"><span class="tag ok">FILE</span></td>
        <td data-label="Name">${escapeHtml(name)}${pathLine}</td>
        <td data-label="Status"><span class="tag ${status.className}">${status.label}</span></td>
        <td data-label="Size">${escapeHtml(formatBytes(bytes))}</td>
        <td data-label="Updated">${escapeHtml(receivedAt)}</td>
        <td data-label="Actions" class="action-cell"><div class="action-buttons">${actions.join(
          ""
        )}</div></td>
      </tr>
    `;
  }

  async function downloadBinary(url, filename, buttonEl, failPrefix) {
    if (!hasToken()) {
      updateTokenState();
      setAlert("error", "Token required before downloading files.");
      return;
    }

    if (buttonEl) {
      buttonEl.disabled = true;
    }
    state.actionInFlight = true;

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
          // Keep fallback message.
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
      handleError(err, failPrefix || "Download failed.");
    } finally {
      if (buttonEl) {
        buttonEl.disabled = false;
      }
      state.actionInFlight = false;
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
    if (state.currentPath) {
      params.set("path", state.currentPath);
    }
    const qs = params.toString();
    if (!qs) {
      return base;
    }
    return `${base}${base.includes("?") ? "&" : "?"}${qs}`;
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
    [els.refreshBtn, els.upBtn].forEach((btn) => {
      if (btn) {
        btn.disabled = isBusy;
      }
    });
    updatePaginationUi();
  }

  function setControlsEnabled(enabled) {
    [els.refreshBtn].forEach((btn) => {
      if (btn) {
        btn.disabled = !enabled;
      }
    });
    if (els.upBtn) {
      els.upBtn.disabled = !enabled || !state.currentPath;
    }
    updatePaginationUi();
  }

  function updatePathUi() {
    if (els.pathLabel) {
      els.pathLabel.textContent = state.currentPath ? `/${state.currentPath}` : "/";
    }
    if (els.upBtn) {
      els.upBtn.disabled = !hasToken() || state.inFlight || !state.currentPath;
    }
    updatePaginationUi();
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
    updatePathUi();
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

  function normalizeUploadPath(value) {
    const raw = String(value || "")
      .replace(/\\+/g, "/")
      .replace(/\/+/g, "/")
      .replace(/^\/+|\/+$/g, "");
    if (!raw) {
      return "";
    }
    return raw
      .split("/")
      .map((segment) => segment.trim())
      .filter((segment) => segment && segment !== "." && segment !== "..")
      .join("/");
  }

  function normalizeParentPath(value) {
    if (value === null || value === undefined) {
      return null;
    }
    const normalized = normalizeUploadPath(String(value));
    if (normalized === "") {
      return "";
    }
    return normalized;
  }

  function parentPath(pathValue) {
    const value = normalizeUploadPath(pathValue);
    if (!value) {
      return "";
    }
    const idx = value.lastIndexOf("/");
    if (idx < 0) {
      return "";
    }
    return value.slice(0, idx);
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

  function sortableTime(value) {
    const dt = new Date(value || "");
    if (Number.isNaN(dt.getTime())) {
      return 0;
    }
    return dt.getTime();
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
