(() => {
  const boot = window.DASHBOARD_BOOTSTRAP || {};

  const state = {
    deviceId: String(boot.deviceId || "").trim(),
    pollMs: clampInt(boot.pollSec, 2, 300, 5) * 1000,
    commandLimit: 5,
    commandBeforeId: null,
    commandHasMore: false,
    commandNextBeforeId: null,
    commandPageIndex: 1,
    commandCursorStack: [],
    commandRowsCount: 0,
    inFlight: false,
    commandInFlight: false,
    pollTimer: null,
  };

  const apiDashboardTemplate = String(
    boot.apiDashboardUrlTemplate || "/admin/devices/{device_id}/dashboard"
  );
  const apiCommandsTemplate = String(
    boot.apiCommandsUrlTemplate || "/admin/devices/{device_id}/commands"
  );

  const els = {
    pageTitle: byId("pageTitle"),
    deviceIdLabel: byId("deviceIdLabel"),
    lastRefreshLabel: byId("lastRefreshLabel"),
    tokenInput: byId("tokenInput"),
    tokenState: byId("tokenState"),
    unlockBtn: byId("unlockBtn"),
    clearTokenBtn: byId("clearTokenBtn"),
    alertBox: byId("alertBox"),
    onlineState: byId("onlineState"),
    lastSeenAge: byId("lastSeenAge"),
    measurementState: byId("measurementState"),
    pendingCommands: byId("pendingCommands"),
    startBtn: byId("startBtn"),
    stopBtn: byId("stopBtn"),
    rebootBtn: byId("rebootBtn"),
    refreshBtn: byId("refreshBtn"),
    uploadsLink: byId("uploadsLink"),
    commandRows: byId("commandRows"),
    cmdPageNewerBtn: byId("cmdPageNewerBtn"),
    cmdPageOlderBtn: byId("cmdPageOlderBtn"),
    cmdPageInfo: byId("cmdPageInfo"),
    telemetryOut: byId("telemetryOut"),
  };

  init();

  function init() {
    if (els.pageTitle && boot.dashboardTitle) {
      els.pageTitle.textContent = String(boot.dashboardTitle);
    }

    if (els.deviceIdLabel) {
      els.deviceIdLabel.textContent = state.deviceId || "(not configured)";
    }

    if (els.uploadsLink) {
      els.uploadsLink.href = String(boot.uploadsUrl || "/uploads");
    }

    const savedToken = sessionStorage.getItem("remote_dashboard_token") || "";
    if (savedToken && els.tokenInput) {
      els.tokenInput.value = savedToken;
    }

    updateTokenState();
    bindEvents();
    setControlsEnabled(hasToken() && !!state.deviceId);
    updateCommandPager();

    if (!state.deviceId) {
      setAlert("error", "Dashboard device is not configured on server.");
      setControlsEnabled(false);
      return;
    }

    if (hasToken()) {
      refreshNow();
    }

    state.pollTimer = window.setInterval(() => {
      if (!hasToken() || state.inFlight || !state.deviceId) {
        return;
      }
      refreshNow();
    }, state.pollMs);
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
        setAlert("info", "Token stored for this browser session.");
        resetCommandPagination();
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
        resetCommandPagination();
        setAlert("info", "Token cleared.");
      });
    }

    if (els.refreshBtn) {
      els.refreshBtn.addEventListener("click", () => {
        refreshNow();
      });
    }

    if (els.startBtn) {
      els.startBtn.addEventListener("click", () => sendCommand("measure_start"));
    }

    if (els.stopBtn) {
      els.stopBtn.addEventListener("click", () => sendCommand("measure_stop"));
    }

    if (els.rebootBtn) {
      els.rebootBtn.addEventListener("click", () => {
        const ok = window.confirm(
          "Reboot the device now? This will interrupt current network connectivity briefly."
        );
        if (!ok) {
          return;
        }
        sendCommand("reboot");
      });
    }

    if (els.cmdPageOlderBtn) {
      els.cmdPageOlderBtn.addEventListener("click", () => {
        goOlderCommandsPage();
      });
    }

    if (els.cmdPageNewerBtn) {
      els.cmdPageNewerBtn.addEventListener("click", () => {
        goNewerCommandsPage();
      });
    }
  }

  async function refreshNow() {
    if (!state.deviceId) {
      return;
    }

    if (!hasToken()) {
      updateTokenState();
      return;
    }

    if (state.inFlight) {
      return;
    }

    state.inFlight = true;
    try {
      const url = buildDashboardUrl();
      const payload = await apiRequest(url, { method: "GET" });
      renderDashboard(payload);
      setControlsEnabled(true);
      setLastRefresh(new Date());
      updateTokenState();
    } catch (err) {
      handleError(err, "Failed to load dashboard data.");
    } finally {
      state.inFlight = false;
    }
  }

  async function sendCommand(action) {
    if (!hasToken()) {
      updateTokenState();
      setAlert("error", "Token required before sending commands.");
      return;
    }
    if (state.commandInFlight || !state.deviceId) {
      return;
    }

    state.commandInFlight = true;
    setButtonsBusy(true);

    const optimistic = prependCommandRow({
      id: "queued",
      action,
      created_at: new Date().toISOString(),
      acked_at: null,
      ack_ok: null,
      ack_result: "queued",
    });

    try {
      const url = withDevice(apiCommandsTemplate, state.deviceId);
      const payload = await apiRequest(url, {
        method: "POST",
        body: { action },
      });

      if (payload && payload.command) {
        updateCommandRow(optimistic, payload.command);
        setAlert(
          "success",
          `Command queued: ${payload.command.action} (#${payload.command.id})`
        );
      } else {
        setAlert("success", "Command queued.");
      }

      resetCommandPagination();
      refreshNow();
    } catch (err) {
      markRowFailed(optimistic, err?.message || "command_failed");
      handleError(err, "Command failed.");
    } finally {
      state.commandInFlight = false;
      setButtonsBusy(false);
    }
  }

  function renderDashboard(payload) {
    const health = payload?.health || {};
    const online = health.online === true;

    if (els.onlineState) {
      els.onlineState.textContent = online ? "ONLINE" : "OFFLINE";
      els.onlineState.classList.remove("online", "offline", "unknown");
      els.onlineState.classList.add(online ? "online" : "offline");
    }

    if (els.lastSeenAge) {
      if (typeof health.last_seen_age_sec === "number") {
        els.lastSeenAge.textContent = `${humanAge(health.last_seen_age_sec)} ago`;
      } else if (health.last_seen) {
        els.lastSeenAge.textContent = String(health.last_seen);
      } else {
        els.lastSeenAge.textContent = "No telemetry yet";
      }
    }

    if (els.measurementState) {
      const meas = health.measurement_active;
      els.measurementState.textContent =
        meas === true ? "ACTIVE" : meas === false ? "STOPPED" : "UNKNOWN";
    }

    if (els.pendingCommands) {
      els.pendingCommands.textContent = String(
        Number.isFinite(health.pending_commands) ? health.pending_commands : 0
      );
    }

    if (els.telemetryOut) {
      const latest = payload?.latest_telemetry;
      if (latest && latest.payload) {
        els.telemetryOut.classList.remove("muted");
        els.telemetryOut.textContent = JSON.stringify(latest, null, 2);
      } else {
        els.telemetryOut.classList.add("muted");
        els.telemetryOut.textContent = "No telemetry received yet.";
      }
    }

    const commands = Array.isArray(payload?.recent_commands)
      ? payload.recent_commands
      : [];
    renderCommands(commands);
    applyCommandPage(payload?.commands_page || null, commands.length);
  }

  function renderCommands(commands) {
    if (!els.commandRows) {
      return;
    }

    if (!Array.isArray(commands) || commands.length === 0) {
      els.commandRows.innerHTML =
        '<tr><td colspan="6" class="muted no-commands-cell" data-label="Info">No commands yet.</td></tr>';
      return;
    }

    els.commandRows.innerHTML = commands
      .map((cmd) => {
        const status = commandStatus(cmd);
        return `
          <tr data-row-id="${escapeHtml(String(cmd.id || ""))}">
            <td data-label="ID">#${escapeHtml(String(cmd.id || ""))}</td>
            <td data-label="Action">${escapeHtml(String(cmd.action || ""))}</td>
            <td data-label="Status"><span class="tag ${status.className}">${status.label}</span></td>
            <td data-label="Created">${escapeHtml(formatDate(cmd.created_at || cmd.issued_at || ""))}</td>
            <td data-label="ACK">${escapeHtml(formatDate(cmd.acked_at || ""))}</td>
            <td data-label="Result">${escapeHtml(String(cmd.ack_result || "--"))}</td>
          </tr>
        `;
      })
      .join("");
  }

  function applyCommandPage(page, visibleCount) {
    state.commandRowsCount = Number.isFinite(visibleCount) ? visibleCount : 0;

    if (page && typeof page === "object") {
      if (Object.prototype.hasOwnProperty.call(page, "before_id")) {
        state.commandBeforeId = normalizePageCursor(page.before_id);
      }
      state.commandHasMore = page.has_more === true;
      state.commandNextBeforeId = state.commandHasMore
        ? normalizePageCursor(page.next_before_id)
        : null;
    } else {
      state.commandHasMore = false;
      state.commandNextBeforeId = null;
    }

    updateCommandPager();
  }

  function updateCommandPager() {
    if (els.cmdPageInfo) {
      const suffix = state.commandRowsCount > 0
        ? ` (${state.commandRowsCount} shown)`
        : "";
      els.cmdPageInfo.textContent = `Page ${state.commandPageIndex}${suffix}`;
    }

    const hasNewer = state.commandPageIndex > 1;
    const canGoOlder =
      state.commandHasMore &&
      !!state.commandNextBeforeId &&
      state.commandRowsCount > 0;

    if (els.cmdPageNewerBtn) {
      els.cmdPageNewerBtn.disabled =
        !hasToken() || !state.deviceId || state.inFlight || state.commandInFlight || !hasNewer;
    }
    if (els.cmdPageOlderBtn) {
      els.cmdPageOlderBtn.disabled =
        !hasToken() || !state.deviceId || state.inFlight || state.commandInFlight || !canGoOlder;
    }
  }

  function resetCommandPagination() {
    state.commandBeforeId = null;
    state.commandHasMore = false;
    state.commandNextBeforeId = null;
    state.commandPageIndex = 1;
    state.commandCursorStack = [];
    state.commandRowsCount = 0;
    updateCommandPager();
  }

  function goOlderCommandsPage() {
    if (
      state.inFlight ||
      !state.commandHasMore ||
      !state.commandNextBeforeId ||
      !state.deviceId ||
      !hasToken()
    ) {
      return;
    }

    state.commandCursorStack.push(state.commandBeforeId);
    state.commandBeforeId = state.commandNextBeforeId;
    state.commandPageIndex += 1;
    updateCommandPager();
    refreshNow();
  }

  function goNewerCommandsPage() {
    if (state.inFlight || !state.deviceId || !hasToken() || state.commandPageIndex <= 1) {
      return;
    }

    const previousBeforeId = state.commandCursorStack.pop();
    state.commandBeforeId =
      typeof previousBeforeId === "string" && previousBeforeId !== ""
        ? previousBeforeId
        : null;
    state.commandPageIndex = Math.max(1, state.commandPageIndex - 1);
    updateCommandPager();
    refreshNow();
  }

  function prependCommandRow(cmd) {
    if (!els.commandRows) {
      return "";
    }

    const tempId = `tmp-${Date.now()}-${Math.floor(Math.random() * 9999)}`;
    const status = commandStatus(cmd);
    const row = document.createElement("tr");
    row.dataset.rowId = tempId;
    row.innerHTML = `
      <td data-label="ID">#${escapeHtml(String(cmd.id || "queued"))}</td>
      <td data-label="Action">${escapeHtml(String(cmd.action || ""))}</td>
      <td data-label="Status"><span class="tag ${status.className}">${status.label}</span></td>
      <td data-label="Created">${escapeHtml(formatDate(cmd.created_at || ""))}</td>
      <td data-label="ACK">${escapeHtml(formatDate(cmd.acked_at || ""))}</td>
      <td data-label="Result">${escapeHtml(String(cmd.ack_result || "queued"))}</td>
    `;

    const first = els.commandRows.firstElementChild;
    if (first && first.querySelector("td") && first.children.length === 1) {
      els.commandRows.innerHTML = "";
    }
    els.commandRows.prepend(row);
    return tempId;
  }

  function updateCommandRow(tempId, cmd) {
    if (!els.commandRows || !tempId) {
      return;
    }
    const row = els.commandRows.querySelector(`tr[data-row-id="${cssEscape(tempId)}"]`);
    if (!row) {
      return;
    }
    row.dataset.rowId = String(cmd.id || tempId);
    const status = commandStatus({ acked_at: null, ack_ok: null });
    row.innerHTML = `
      <td data-label="ID">#${escapeHtml(String(cmd.id || "queued"))}</td>
      <td data-label="Action">${escapeHtml(String(cmd.action || ""))}</td>
      <td data-label="Status"><span class="tag ${status.className}">${status.label}</span></td>
      <td data-label="Created">${escapeHtml(formatDate(cmd.created_at || cmd.issued_at || new Date().toISOString()))}</td>
      <td data-label="ACK">--</td>
      <td data-label="Result">queued</td>
    `;
  }

  function markRowFailed(tempId, reason) {
    if (!els.commandRows || !tempId) {
      return;
    }
    const row = els.commandRows.querySelector(`tr[data-row-id="${cssEscape(tempId)}"]`);
    if (!row) {
      return;
    }
    row.innerHTML = `
      <td data-label="ID">#--</td>
      <td data-label="Action">command</td>
      <td data-label="Status"><span class="tag fail">FAILED</span></td>
      <td data-label="Created">${escapeHtml(formatDate(new Date().toISOString()))}</td>
      <td data-label="ACK">--</td>
      <td data-label="Result">${escapeHtml(String(reason || "failed"))}</td>
    `;
  }

  function commandStatus(cmd) {
    if (!cmd || !cmd.acked_at) {
      return { label: "PENDING", className: "pending" };
    }
    if (cmd.ack_ok === true) {
      return { label: "ACK OK", className: "ok" };
    }
    return { label: "ACK FAIL", className: "fail" };
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

  function setButtonsBusy(isBusy) {
    [els.startBtn, els.stopBtn, els.rebootBtn, els.refreshBtn].forEach((btn) => {
      if (btn) {
        btn.disabled = isBusy;
      }
    });
    updateCommandPager();
  }

  function setControlsEnabled(enabled) {
    [els.startBtn, els.stopBtn, els.rebootBtn, els.refreshBtn].forEach((btn) => {
      if (btn) {
        btn.disabled = !enabled;
      }
    });
    updateCommandPager();
  }

  function updateTokenState() {
    if (!els.tokenState) {
      return;
    }
    if (hasToken()) {
      els.tokenState.textContent = "Token loaded. Commands are enabled.";
    } else {
      els.tokenState.textContent = "Locked. Enter token to enable commands.";
    }
    updateCommandPager();
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
    if (String(err?.code) === "401") {
      setControlsEnabled(false);
    }
  }

  function withDevice(template, deviceId) {
    return template.replace("{device_id}", encodeURIComponent(deviceId));
  }

  function buildDashboardUrl() {
    const base = withDevice(apiDashboardTemplate, state.deviceId);
    const params = new URLSearchParams();
    params.set("command_limit", String(state.commandLimit));
    if (state.commandBeforeId) {
      params.set("command_before_id", state.commandBeforeId);
    }
    return `${base}${base.includes("?") ? "&" : "?"}${params.toString()}`;
  }

  function normalizePageCursor(value) {
    if (value === null || value === undefined) {
      return null;
    }
    const text = String(value).trim();
    if (!/^\d+$/.test(text)) {
      return null;
    }
    if (text === "0") {
      return null;
    }
    return text;
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

  function humanAge(totalSec) {
    const sec = Math.max(0, Number(totalSec) || 0);
    if (sec < 60) {
      return `${sec}s`;
    }
    const min = Math.floor(sec / 60);
    if (min < 60) {
      return `${min}m ${sec % 60}s`;
    }
    const h = Math.floor(min / 60);
    return `${h}h ${min % 60}m`;
  }

  function byId(id) {
    return document.getElementById(id);
  }

  function clampInt(value, min, max, fallback) {
    const num = Number.parseInt(value, 10);
    if (!Number.isFinite(num)) {
      return fallback;
    }
    return Math.min(max, Math.max(min, num));
  }

  function escapeHtml(input) {
    return String(input)
      .replace(/&/g, "&amp;")
      .replace(/</g, "&lt;")
      .replace(/>/g, "&gt;")
      .replace(/\"/g, "&quot;")
      .replace(/'/g, "&#39;");
  }

  function cssEscape(value) {
    if (window.CSS && typeof window.CSS.escape === "function") {
      return window.CSS.escape(value);
    }
    return String(value).replace(/[^a-zA-Z0-9_-]/g, "\\$&");
  }
})();
