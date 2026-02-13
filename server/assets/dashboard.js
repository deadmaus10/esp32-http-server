(() => {
  const boot = window.DASHBOARD_BOOTSTRAP || {};

  const state = {
    deviceId: String(boot.deviceId || "").trim(),
    pollMs: clampInt(boot.pollSec, 2, 300, 5) * 1000,
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
    commandRows: byId("commandRows"),
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

    const savedToken = sessionStorage.getItem("remote_dashboard_token") || "";
    if (savedToken && els.tokenInput) {
      els.tokenInput.value = savedToken;
    }

    updateTokenState();
    bindEvents();
    setControlsEnabled(hasToken() && !!state.deviceId);

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
        const token = (els.tokenInput?.value || "").trim();
        if (!token) {
          setAlert("error", "Enter admin token first.");
          updateTokenState();
          return;
        }
        sessionStorage.setItem("remote_dashboard_token", token);
        updateTokenState();
        setAlert("info", "Token stored for this browser session.");
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
      const url = withDevice(apiDashboardTemplate, state.deviceId);
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

    renderCommands(payload?.recent_commands || []);
  }

  function renderCommands(commands) {
    if (!els.commandRows) {
      return;
    }

    if (!Array.isArray(commands) || commands.length === 0) {
      els.commandRows.innerHTML =
        '<tr><td colspan="6" class="muted">No commands yet.</td></tr>';
      return;
    }

    els.commandRows.innerHTML = commands
      .map((cmd) => {
        const status = commandStatus(cmd);
        return `
          <tr data-row-id="${escapeHtml(String(cmd.id || ""))}">
            <td>#${escapeHtml(String(cmd.id || ""))}</td>
            <td>${escapeHtml(String(cmd.action || ""))}</td>
            <td><span class="tag ${status.className}">${status.label}</span></td>
            <td>${escapeHtml(formatDate(cmd.created_at || cmd.issued_at || ""))}</td>
            <td>${escapeHtml(formatDate(cmd.acked_at || ""))}</td>
            <td>${escapeHtml(String(cmd.ack_result || "--"))}</td>
          </tr>
        `;
      })
      .join("");
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
      <td>#${escapeHtml(String(cmd.id || "queued"))}</td>
      <td>${escapeHtml(String(cmd.action || ""))}</td>
      <td><span class="tag ${status.className}">${status.label}</span></td>
      <td>${escapeHtml(formatDate(cmd.created_at || ""))}</td>
      <td>${escapeHtml(formatDate(cmd.acked_at || ""))}</td>
      <td>${escapeHtml(String(cmd.ack_result || "queued"))}</td>
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
      <td>#${escapeHtml(String(cmd.id || "queued"))}</td>
      <td>${escapeHtml(String(cmd.action || ""))}</td>
      <td><span class="tag ${status.className}">${status.label}</span></td>
      <td>${escapeHtml(formatDate(cmd.created_at || cmd.issued_at || new Date().toISOString()))}</td>
      <td>--</td>
      <td>queued</td>
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
      <td>#--</td>
      <td>command</td>
      <td><span class="tag fail">FAILED</span></td>
      <td>${escapeHtml(formatDate(new Date().toISOString()))}</td>
      <td>--</td>
      <td>${escapeHtml(String(reason || "failed"))}</td>
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
    const token = (sessionStorage.getItem("remote_dashboard_token") || "").trim();
    if (!token) {
      const err = new Error("Missing admin token");
      err.code = "NO_TOKEN";
      throw err;
    }

    const headers = {
      Accept: "application/json",
      "X-ADMIN-TOKEN": token,
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
  }

  function setControlsEnabled(enabled) {
    [els.startBtn, els.stopBtn, els.rebootBtn, els.refreshBtn].forEach((btn) => {
      if (btn) {
        btn.disabled = !enabled;
      }
    });
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
  }

  function hasToken() {
    const token = (sessionStorage.getItem("remote_dashboard_token") || "").trim();
    return token.length > 0;
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
