// ===== Config =====
let esp32IP = "192.168.4.1"; // SoftAP default from your firmware; change if needed.
let socket = null;
let currentAxis = "ROLL"; // UI state for tabs

// ===== Helpers =====
const $ = (id) => document.getElementById(id);

function appendLog(message) {
  const el = $("log");
  const ts = new Date().toLocaleTimeString();
  el.textContent += `[${ts}] ${message}\n`;
  el.scrollTop = el.scrollHeight;
}

function setStatus(text) {
  $("statusText").textContent = text;
}

let pidCache = {
  ROLL:  { kp: 0, ki: 0, kd: 0 },
  PITCH: { kp: 0, ki: 0, kd: 0 },
  YAW:   { kp: 0, ki: 0, kd: 0 },
};
function syncInputs(axis) {
  const v = pidCache[axis];
  $("kp_val").value = v.kp;
  $("ki_val").value = v.ki;
  $("kd_val").value = v.kd;
}


// ===== WebSocket =====
function connectWebSocket() {
  if (socket && socket.readyState === WebSocket.OPEN) {
    appendLog("WebSocket already connected.");
    return;
  }

  const url = `ws://${esp32IP}/ws`;
  appendLog(`Connecting to ${url} ...`);
  setStatus("Connecting...");

  socket = new WebSocket(url);

  socket.onopen = () => {
    setStatus("Connected");
    appendLog("🌐 Connected.");
    sendCommand("GET_PID=ALL");
  };

  socket.onmessage = (event) => {
    const msg = event.data;

    // Try to parse telemetry JSON like: {"roll":x,"pitch":y,"yaw":z}
    if (msg && msg.startsWith("{") && msg.endsWith("}")) {
      try {
        const data = JSON.parse(msg);

        // Update dashboard values if present
        if (typeof data.roll === "number") $("roll").textContent = data.roll.toFixed(1);
        if (typeof data.pitch === "number") $("pitch").textContent = data.pitch.toFixed(1);
        if (typeof data.yaw === "number") $("yaw").textContent = data.yaw.toFixed(1);
        if (data.pid) {
          if (data.pid.roll)  pidCache.ROLL  = data.pid.roll;
          if (data.pid.pitch) pidCache.PITCH = data.pid.pitch;
          if (data.pid.yaw)   pidCache.YAW   = data.pid.yaw;
          syncInputs(currentAxis);
          return;
        }
        return;
      } catch (e) {
        // fall through to log
      }
    }

    appendLog(msg);
  };

  socket.onclose = () => {
    setStatus("Disconnected");
    appendLog("❌ Disconnected. Reconnecting in 2s...");
    setTimeout(connectWebSocket, 2000);
  };

  socket.onerror = (err) => {
    appendLog(`⚠️ WebSocket error: ${err.message || err}`);
  };
}

function sendCommand(text) {
  if (!socket || socket.readyState !== WebSocket.OPEN) {
    appendLog("⚠️ Not connected; cannot send.");
    return;
  }
  socket.send(text);
  appendLog(`➡️ Sent: ${text}`);
}

// ===== PID UI =====
function setAxis(axis) {
  currentAxis = axis;
  const buttons = Array.from($("axisTabs").querySelectorAll("button"));
  buttons.forEach((b) => b.classList.toggle("active", b.dataset.axis === axis));
  $("pidHint").innerHTML = `Editing: <b>${axis}</b>`;
  syncInputs(axis);
}


function applyPID() {
  const kp = $("kp_val").value;
  const ki = $("ki_val").value;
  const kd = $("kd_val").value;
  const letter = currentAxis[0]; // R / P / Y

  if (kp !== "") sendCommand(`KP_${letter}=${kp}`);
  if (ki !== "") sendCommand(`KI_${letter}=${ki}`);
  if (kd !== "") sendCommand(`KD_${letter}=${kd}`);
}


function refreshPID() {
  sendCommand("GET_PID=ALL");
}


// ===== Wire up UI =====
window.addEventListener("DOMContentLoaded", () => {
  // Prefill IP field and connect
  $("ipInput").value = esp32IP;
  $("connectBtn").addEventListener("click", () => {
    esp32IP = $("ipInput").value.trim() || esp32IP;
    connectWebSocket();
  });

  // Axis tabs
  $("axisTabs").addEventListener("click", (e) => {
    if (e.target.tagName === "BUTTON") setAxis(e.target.dataset.axis);
  });

  // PID actions
  $("applyBtn").addEventListener("click", applyPID);
  $("refreshBtn").addEventListener("click", refreshPID);

  // Auto-connect on load
  connectWebSocket();
});
