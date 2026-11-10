/**
 * ENGINE-TWIN: Professional Aerospace Digital Twin Ground Station Controller
 * Robust 20 Hz Telemetry, Multi-Channel Temperature/Pressure Charting,
 * Calibrated Anomaly Indexing, Event Timeline Logging, and Stage Automation.
 * Optimized for Zero-Flicker High-Performance Ground Station Operations.
 */

const MAX_HISTORY = 40;
let ws = null;
let engine3D = null;
let charts = {};

// Audio Synthesizer State
let audioCtx = null;
let isAudioMuted = true;
let lastAudioAlertTime = 0;

// Active System & Fault State (Cached for zero-flicker rendering)
let currentActiveFault = "HEALTHY";
let lastActiveFaultButton = null;
let lastShapSignature = "";
let lastOverallStatus = "";
let eventLogs = [];
let maxEventLogs = 30;

// Auto Demo State Machine
let autoDemoActive = false;
let autoDemoTimer = null;
let autoDemoStep = 0;
let autoDemoTimeRemaining = 0;

// Chart Rendering Throttle State
let lastChartUpdateTime = 0;
const CHART_UPDATE_INTERVAL_MS = 100; // 10 Hz smooth charting (prevents canvas flutter)

// Telemetry History Buffers (Multi-Channel)
const historyData = {
  time: [],
  sensor_egt1: [],
  sensor_egt2: [],
  sensor_egt3: [],
  sensor_egt4: [],
  mvem_egt: [],
  sensor_oil_p: [],
  mvem_oil_p: [],
  sensor_map: [],
  mvem_map: [],
  sensor_vib: [],
  mvem_vib: []
};

// Defensive DOM Helpers
function setText(id, text) {
  const el = document.getElementById(id);
  if (el && el.innerText !== text) {
    el.innerText = text;
  }
}

// ------------------------------------------------------------------
// 1. INITIALIZATION ON DOM READY
// ------------------------------------------------------------------
document.addEventListener("DOMContentLoaded", () => {
  console.log("ENGINE-TWIN GCS Controller Initialized.");

  // 1. Initialize 3D Engine Model
  if (typeof Engine3DView !== "undefined") {
    try {
      engine3D = new Engine3DView("container-3d");
    } catch (e) {
      console.warn("3D View Init Warning:", e);
    }
  }

  // 2. Initialize Telemetry Charts
  try {
    initCharts();
  } catch (e) {
    console.error("Charts Init Error:", e);
  }

  // 3. Connect Telemetry Stream (WebSocket + SSE Fallback)
  connectWebSocket();

  // 4. Log Startup Event
  addEventLog("SYS", "Ground Control Station connected to telemetry stream (20 Hz).");
});

// ------------------------------------------------------------------
// 2. WEB AUDIO API SYNTHESIZER (AEROSPACE COCKPIT ALARMS)
// ------------------------------------------------------------------
function initAudioContext() {
  if (!audioCtx) {
    const AudioContext = window.AudioContext || window.webkitAudioContext;
    if (AudioContext) audioCtx = new AudioContext();
  }
  if (audioCtx && audioCtx.state === 'suspended') {
    audioCtx.resume();
  }
}

function toggleAudio() {
  initAudioContext();
  isAudioMuted = !isAudioMuted;
  const icon = document.getElementById("icon-audio");
  const lbl = document.getElementById("lbl-audio");
  const btn = document.getElementById("btn-audio");

  if (isAudioMuted) {
    if (icon) icon.className = "fa-solid fa-volume-xmark";
    if (lbl) lbl.innerText = "MUTED";
    if (btn) btn.className = "px-2.5 py-1 text-xs bg-slate-900 hover:bg-slate-800 border border-slate-700 rounded text-slate-400 transition flex items-center space-x-1.5";
  } else {
    if (icon) icon.className = "fa-solid fa-volume-high text-emerald-400";
    if (lbl) lbl.innerText = "ARMED";
    if (btn) btn.className = "px-2.5 py-1 text-xs bg-emerald-950/80 border border-emerald-700 rounded text-emerald-300 transition flex items-center space-x-1.5";
    playTone(880, 0.1, "sine");
  }
}

function playTone(freq, durationSec, type = "sine") {
  if (isAudioMuted || !audioCtx) return;
  try {
    const osc = audioCtx.createOscillator();
    const gain = audioCtx.createGain();
    osc.type = type;
    osc.frequency.setValueAtTime(freq, audioCtx.currentTime);
    gain.gain.setValueAtTime(0.08, audioCtx.currentTime);
    gain.gain.exponentialRampToValueAtTime(0.001, audioCtx.currentTime + durationSec);
    osc.connect(gain);
    gain.connect(audioCtx.destination);
    osc.start();
    osc.stop(audioCtx.currentTime + durationSec);
  } catch (e) {}
}

function playMasterCautionSound() {
  const now = Date.now();
  if (now - lastAudioAlertTime < 2500) return;
  lastAudioAlertTime = now;
  playTone(650, 0.15, "triangle");
  setTimeout(() => playTone(850, 0.20, "triangle"), 160);
}

function playEmergencyAlarmSound() {
  const now = Date.now();
  if (now - lastAudioAlertTime < 2000) return;
  lastAudioAlertTime = now;
  playTone(950, 0.2, "sawtooth");
  setTimeout(() => playTone(750, 0.2, "sawtooth"), 220);
}

// ------------------------------------------------------------------
// 3. TELEMETRY STREAM CONNECTION (WS + SSE FALLBACK)
// ------------------------------------------------------------------
function connectWebSocket() {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsUrl = `${protocol}//${window.location.host}/ws/telemetry`;

  try {
    ws = new WebSocket(wsUrl);

    ws.onopen = () => {
      updateBadge("badge-connection", "TELEMETRY LIVE (20 Hz WS)", "bg-emerald-950/80", "border-emerald-800", "text-emerald-400");
    };

    ws.onmessage = (event) => {
      try {
        const state = JSON.parse(event.data);
        updateDashboard(state);
      } catch (err) {}
    };

    ws.onerror = () => startSSEStream();
    ws.onclose = () => startSSEStream();
  } catch (e) {
    startSSEStream();
  }
}

let sseSource = null;
let pollInterval = null;

function startSSEStream() {
  if (sseSource || pollInterval) return;

  if (window.EventSource) {
    try {
      sseSource = new EventSource('/api/stream');
      sseSource.onopen = () => {
        updateBadge("badge-connection", "TELEMETRY LIVE (20 Hz SSE)", "bg-emerald-950/80", "border-emerald-800", "text-emerald-400");
      };
      sseSource.onmessage = (e) => {
        try {
          const state = JSON.parse(e.data);
          updateDashboard(state);
        } catch (err) {}
      };
      sseSource.onerror = () => {
        if (sseSource) {
          sseSource.close();
          sseSource = null;
        }
        startPolling();
      };
      return;
    } catch (err) {}
  }
  startPolling();
}

function startPolling() {
  if (pollInterval) return;
  updateBadge("badge-connection", "TELEMETRY LIVE (20 Hz HTTP)", "bg-emerald-950/80", "border-emerald-800", "text-emerald-400");
  pollInterval = setInterval(async () => {
    try {
      const res = await fetch('/api/state');
      if (res.ok) {
        const state = await res.json();
        updateDashboard(state);
      }
    } catch (e) {}
  }, 50);
}

// ------------------------------------------------------------------
// 4. REAL-TIME DATA PROPAGATION & COMPONENT SYNCHRONIZATION
// ------------------------------------------------------------------
function updateDashboard(state) {
  if (!state || state.status === "initializing") return;

  // 1. Mission Context Ribbon
  const totalSec = Math.floor(state.timestamp_s || 0);
  const hrs = String(Math.floor(totalSec / 3600)).padStart(2, '0');
  const mins = String(Math.floor((totalSec % 3600) / 60)).padStart(2, '0');
  const secs = String(totalSec % 60).padStart(2, '0');
  setText("txt-time", `${hrs}:${mins}:${secs}`);
  setText("txt-phase", state.mission_phase || "CRUISE / LOITER");
  setText("txt-alt", `${Number(state.altitude_ft || 28500).toLocaleString()} FT`);
  setText("txt-tas", `${(state.airspeed_mps || 55.0).toFixed(1)} M/S`);
  setText("txt-oat", `${(state.ambient_temp_c || -20.0).toFixed(1)} °C`);
  setText("txt-throttle", `${(state.throttle_pct || 72.0).toFixed(1)} %`);
  setText("txt-power-hp", `POWER: ${(state.mvem_expected_power_hp || 150.0).toFixed(1)} HP`);

  // Step 6: UAV Digital Twin Subsystems
  if (state.uav_twin) {
    updateUavDigitalTwin(state.uav_twin);
  }

  // 2. Subsystem Health Matrix
  const h = state.health || {};
  const overall = h.overall_health !== undefined ? h.overall_health : 100.0;
  setText("val-health-overall", `${overall.toFixed(1)} %`);
  setText("val-health-oil", `${(h.oil_system_health !== undefined ? h.oil_system_health : 100).toFixed(1)} %`);
  setText("val-health-turbo", `${(h.turbo_boost_health !== undefined ? h.turbo_boost_health : 100).toFixed(1)} %`);
  setText("val-health-cyls", `${(h.combustion_health !== undefined ? h.combustion_health : 100).toFixed(1)} %`);
  setText("val-health-vib", `${(h.vibration_health !== undefined ? h.vibration_health : 100).toFixed(1)} %`);
  setText("val-health-elec", `${(h.electrical_health !== undefined ? h.electrical_health : 100).toFixed(1)} %`);

  // Status Badge & Alarm Sound (Optimized class caching)
  const currentStatusStr = overall < 50.0 ? "CRITICAL" : (overall < 80.0 ? "WARNING" : "NOMINAL");
  if (currentStatusStr !== lastOverallStatus) {
    lastOverallStatus = currentStatusStr;
    const valOverallEl = document.getElementById("val-health-overall");
    const badgeStatus = document.getElementById("badge-status-level");

    if (currentStatusStr === "CRITICAL") {
      if (valOverallEl) valOverallEl.className = "text-base font-bold text-red-400 mt-0.5";
      if (badgeStatus) {
        badgeStatus.className = "text-[10px] px-2 py-0.5 rounded bg-red-950 border border-red-700 text-red-400 font-bold mono";
        badgeStatus.innerText = "CRITICAL";
      }
      playEmergencyAlarmSound();
    } else if (currentStatusStr === "WARNING") {
      if (valOverallEl) valOverallEl.className = "text-base font-bold text-amber-400 mt-0.5";
      if (badgeStatus) {
        badgeStatus.className = "text-[10px] px-2 py-0.5 rounded bg-amber-950 border border-amber-700 text-amber-400 font-bold mono";
        badgeStatus.innerText = "WARNING";
      }
      playMasterCautionSound();
    } else {
      if (valOverallEl) valOverallEl.className = "text-base font-bold text-emerald-400 mt-0.5";
      if (badgeStatus) {
        badgeStatus.className = "text-[10px] px-2 py-0.5 rounded bg-emerald-950 border border-emerald-700 text-emerald-400 font-bold mono";
        badgeStatus.innerText = "NOMINAL";
      }
    }
  }

  // 3. Cylinder-by-Cylinder Quick Stats
  const cylH = h.cylinder_health || [100, 100, 100, 100];
  const egtArr = state.sensor_egt_c || [810, 810, 810, 810];
  for (let i = 0; i < 4; i++) {
    const elStat = document.getElementById(`txt-cyl${i+1}-stat`);
    const elTemp = document.getElementById(`txt-cyl${i+1}-temp`);
    if (elStat) {
      const statText = `${cylH[i].toFixed(0)}%`;
      if (elStat.innerText !== statText) {
        elStat.innerText = statText;
        elStat.className = cylH[i] < 50 ? "text-red-400" : (cylH[i] < 80 ? "text-amber-400" : "text-emerald-400");
      }
    }
    if (elTemp) {
      const tempText = `${egtArr[i].toFixed(0)}°C`;
      if (elTemp.innerText !== tempText) {
        elTemp.innerText = tempText;
      }
    }
  }

  // 4. AI Diagnostics & Anomaly Score
  const ai = state.ai_prognostics || {};
  const isAnom = ai.anomaly_detected || false;
  const anomIndex = ai.anomaly_score !== undefined ? ai.anomaly_score : 0.0;
  const rawMse = ai.anomaly_raw_mse !== undefined ? ai.anomaly_raw_mse : (isAnom ? 120.5 : 0.62);
  const thresh = ai.anomaly_threshold !== undefined ? ai.anomaly_threshold : 26.6;

  const badgeAnom = document.getElementById("badge-anomaly");
  const valAnomIdx = document.getElementById("val-anom-idx");
  const valRawMse = document.getElementById("val-raw-mse");

  if (isAnom || anomIndex > 0.60) {
    if (badgeAnom && badgeAnom.innerText !== "ANOMALY DETECTED") {
      badgeAnom.className = "text-[9px] px-1.5 py-0.5 rounded bg-red-950 border border-red-700 text-red-400 mono font-bold";
      badgeAnom.innerText = "ANOMALY DETECTED";
    }
    if (valAnomIdx && !valAnomIdx.classList.contains("text-red-400")) {
      valAnomIdx.className = "text-red-400 font-bold";
    }
  } else {
    if (badgeAnom && badgeAnom.innerText !== "NOMINAL") {
      badgeAnom.className = "text-[9px] px-1.5 py-0.5 rounded bg-slate-800 text-slate-400 mono font-bold";
      badgeAnom.innerText = "NOMINAL";
    }
    if (valAnomIdx && !valAnomIdx.classList.contains("text-sky-400")) {
      valAnomIdx.className = "text-sky-400 font-bold";
    }
  }

  setText("val-anom-idx", `${anomIndex.toFixed(2)} / 1.00`);
  setText("val-raw-mse", `MSE: ${rawMse.toFixed(1)} (Thresh: ${thresh.toFixed(1)})`);

  // Diagnosed Fault Label
  const faultClassEl = document.getElementById("lbl-fault-class");
  const faultName = ai.fault_class || "HEALTHY";
  if (faultClassEl) {
    const formattedFault = faultName.replace(/_/g, ' ');
    if (faultClassEl.innerText !== formattedFault) {
      faultClassEl.innerText = formattedFault;
      if (faultName !== "HEALTHY" && faultName !== "SENSOR_FAULT_EGT3") {
        faultClassEl.className = "text-xs font-bold text-red-200 mono bg-red-950/80 px-2.5 py-1.5 rounded border border-red-700 tracking-wide";
      } else if (faultName === "SENSOR_FAULT_EGT3") {
        faultClassEl.className = "text-xs font-bold text-sky-200 mono bg-sky-950/80 px-2.5 py-1.5 rounded border border-sky-700 tracking-wide";
      } else {
        faultClassEl.className = "text-xs font-bold text-slate-200 mono bg-slate-900/90 px-2.5 py-1.5 rounded border border-slate-800 tracking-wide";
      }
    }
  }

  setText("val-conf", `${(ai.fault_confidence_pct || 99.8).toFixed(1)}%`);
  setText("val-sev", (ai.fault_severity || 0.0).toFixed(2));

  // RUL Inspection Window
  const rMin = ai.rul_hours_min !== undefined ? ai.rul_hours_min.toFixed(1) : "450.0";
  const rMax = ai.rul_hours_max !== undefined ? ai.rul_hours_max.toFixed(1) : "550.0";
  setText("txt-rul-interval", `${rMin} – ${rMax} Flight Hrs`);

  // 5. Update SHAP XAI Drawer (Cached)
  updateShapDrawer(ai.top_contributing_channels || []);

  // 6. Highlight Active Fault Button on Bottom Control Panel (Cached)
  const serverActiveFault = state.active_fault || currentActiveFault || "HEALTHY";
  updateActiveFaultButton(serverActiveFault);

  // 7. Update 3D Visualizer
  if (engine3D && typeof engine3D.updateFromTwinState === "function") {
    try {
      engine3D.updateFromTwinState(state);
    } catch (e) {}
  }

  // 8. Update Chart.js Buffers (Throttled for Zero Canvas Flutter)
  updateChartData(state);
}

// ------------------------------------------------------------------
// 5. SHAP XAI ATTRIBUTION DRAWER (ZERO-FLICKER CACHED)
// ------------------------------------------------------------------
function updateShapDrawer(shapItems) {
  const container = document.getElementById("container-shap");
  if (!container) return;

  const currentSignature = JSON.stringify(shapItems || []);
  if (currentSignature === lastShapSignature) return; // Prevent DOM re-creation if unchanged
  lastShapSignature = currentSignature;

  if (!shapItems || shapItems.length === 0) {
    container.innerHTML = `<div class="text-slate-500 text-center py-4 text-xs">All channels within normal physical bounds.</div>`;
    return;
  }

  let html = "";
  shapItems.forEach((item) => {
    const isHigher = item.is_positive !== undefined ? item.is_positive : (item.direction === "ABOVE_BASELINE");
    const barColor = isHigher ? "bg-purple-500" : "bg-sky-500";
    const dirSign = isHigher ? `+${Math.abs(item.residual_sigma || 0).toFixed(1)}σ` : `-${Math.abs(item.residual_sigma || 0).toFixed(1)}σ`;

    html += `
      <div class="bg-slate-900/90 p-1.5 rounded border border-slate-800 space-y-1">
        <div class="flex justify-between items-center text-[11px]">
          <span class="font-medium text-slate-200 truncate pr-2">${item.display_name}</span>
          <span class="font-bold text-slate-300 mono whitespace-nowrap">${item.importance_pct}% (<span class="${isHigher ? 'text-purple-400' : 'text-sky-400'}">${dirSign}</span>)</span>
        </div>
        <div class="w-full bg-slate-800 h-1.5 rounded-full overflow-hidden">
          <div class="${barColor} h-1.5 rounded-full transition-all duration-200" style="width: ${Math.min(100, item.importance_pct)}%"></div>
        </div>
      </div>
    `;
  });
  container.innerHTML = html;
}

// ------------------------------------------------------------------
// 6. MULTI-CHANNEL CHART.JS INITIALIZATION & UPDATES
// ------------------------------------------------------------------
function initCharts() {
  if (typeof Chart === "undefined") {
    console.warn("Chart.js not loaded yet.");
    return;
  }

  const commonOptions = {
    responsive: true,
    maintainAspectRatio: false,
    animation: false,
    elements: { line: { tension: 0.10, borderWidth: 1.8 }, point: { radius: 0 } },
    scales: {
      x: { display: false },
      y: {
        grid: { color: 'rgba(51, 65, 85, 0.20)' },
        ticks: { color: '#94A3B8', font: { size: 9, family: 'JetBrains Mono' } }
      }
    },
    plugins: {
      legend: { position: 'top', labels: { boxWidth: 10, color: '#CBD5E1', font: { size: 9 } } }
    }
  };

  // Chart 1: Exhaust Gas Temperature (EGT 1-4) & Baseline
  const canvasTemp = document.getElementById('chart-temp');
  if (canvasTemp) {
    charts.temp = new Chart(canvasTemp, {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          { label: 'EGT Cyl 1 (°C)', borderColor: '#FB923C', backgroundColor: 'transparent', data: [] },
          { label: 'EGT Cyl 2 (°C)', borderColor: '#38BDF8', backgroundColor: 'transparent', data: [] },
          { label: 'EGT Cyl 3 (°C)', borderColor: '#34D399', backgroundColor: 'transparent', data: [] },
          { label: 'EGT Cyl 4 (°C)', borderColor: '#FBBF24', backgroundColor: 'transparent', data: [] },
          { label: 'Expected MVEM Baseline', borderColor: '#94A3B8', borderDash: [4, 4], backgroundColor: 'transparent', data: [] }
        ]
      },
      options: {
        ...commonOptions,
        scales: {
          ...commonOptions.scales,
          y: {
            ...commonOptions.scales.y,
            min: 0,
            max: 1000,
            ticks: {
              color: '#94A3B8',
              font: { size: 9, family: 'JetBrains Mono' },
              callback: (v) => v + '°C'
            }
          }
        }
      }
    });
  }

  // Chart 2: Oil Pressure (bar) & Manifold Absolute Pressure (MAP bar)
  const canvasPressures = document.getElementById('chart-pressures');
  if (canvasPressures) {
    charts.pressures = new Chart(canvasPressures, {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          { label: 'Oil Pressure (bar)', borderColor: '#10B981', backgroundColor: 'transparent', data: [] },
          { label: 'Oil P Baseline', borderColor: '#6EE7B7', borderDash: [4, 4], backgroundColor: 'transparent', data: [] },
          { label: 'Manifold MAP (bar)', borderColor: '#06B6D4', backgroundColor: 'transparent', data: [] },
          { label: 'MAP Baseline', borderColor: '#67E8F9', borderDash: [4, 4], backgroundColor: 'transparent', data: [] }
        ]
      },
      options: {
        ...commonOptions,
        scales: {
          ...commonOptions.scales,
          y: {
            ...commonOptions.scales.y,
            min: 0,
            max: 7.0,
            ticks: {
              color: '#94A3B8',
              font: { size: 9, family: 'JetBrains Mono' },
              callback: (v) => v + ' bar'
            }
          }
        }
      }
    });
  }

  // Chart 3: Vibration Spectrum RMS (g) & Output Power (HP)
  const canvasVib = document.getElementById('chart-vibration');
  if (canvasVib) {
    charts.vibration = new Chart(canvasVib, {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          { label: 'Vibration 2X RMS (g)', borderColor: '#C084FC', yAxisID: 'y', backgroundColor: 'transparent', data: [] },
          { label: 'Engine Power (HP)', borderColor: '#38BDF8', borderDash: [3, 3], yAxisID: 'y1', backgroundColor: 'transparent', data: [] }
        ]
      },
      options: {
        ...commonOptions,
        scales: {
          x: commonOptions.scales.x,
          y: {
            ...commonOptions.scales.y,
            min: 0,
            max: 6.0,
            ticks: {
              color: '#C084FC',
              font: { size: 9, family: 'JetBrains Mono' },
              callback: (v) => v + 'g'
            }
          },
          y1: {
            position: 'right',
            grid: { drawOnChartArea: false },
            min: 0,
            max: 220,
            ticks: {
              color: '#38BDF8',
              font: { size: 9, family: 'JetBrains Mono' },
              callback: (v) => v + ' HP'
            }
          }
        }
      }
    });
  }
}

function updateChartData(state) {
  if (!state) return;
  const timeLabel = `${(state.timestamp_s || 0).toFixed(1)}s`;
  historyData.time.push(timeLabel);

  const egts = state.sensor_egt_c || [810, 810, 810, 810];
  const mvemEgt = (state.mvem_expected_egt_c && state.mvem_expected_egt_c[0]) || 810;

  historyData.sensor_egt1.push(egts[0]);
  historyData.sensor_egt2.push(egts[1]);
  historyData.sensor_egt3.push(egts[2]);
  historyData.sensor_egt4.push(egts[3]);
  historyData.mvem_egt.push(mvemEgt);

  historyData.sensor_oil_p.push(state.sensor_oil_pressure_bar !== undefined ? state.sensor_oil_pressure_bar : 4.2);
  historyData.mvem_oil_p.push(state.mvem_expected_oil_pressure_bar || 4.2);
  historyData.sensor_map.push(state.sensor_manifold_pressure_bar !== undefined ? state.sensor_manifold_pressure_bar : 2.45);
  historyData.mvem_map.push(state.mvem_expected_manifold_pressure_bar || 2.45);

  historyData.sensor_vib.push(state.sensor_vibration_rms_g !== undefined ? state.sensor_vibration_rms_g : 0.48);
  historyData.mvem_vib.push(state.mvem_expected_power_hp || 150.0);

  while (historyData.time.length > MAX_HISTORY) {
    historyData.time.shift();
    historyData.sensor_egt1.shift();
    historyData.sensor_egt2.shift();
    historyData.sensor_egt3.shift();
    historyData.sensor_egt4.shift();
    historyData.mvem_egt.shift();
    historyData.sensor_oil_p.shift();
    historyData.mvem_oil_p.shift();
    historyData.sensor_map.shift();
    historyData.mvem_map.shift();
    historyData.sensor_vib.shift();
    historyData.mvem_vib.shift();
  }

  // Smooth Throttled Chart Update (10 Hz)
  const now = Date.now();
  if (now - lastChartUpdateTime >= CHART_UPDATE_INTERVAL_MS) {
    lastChartUpdateTime = now;

    if (charts.temp && charts.temp.data && charts.temp.data.datasets.length >= 5) {
      charts.temp.data.labels = historyData.time;
      charts.temp.data.datasets[0].data = historyData.sensor_egt1;
      charts.temp.data.datasets[1].data = historyData.sensor_egt2;
      charts.temp.data.datasets[2].data = historyData.sensor_egt3;
      charts.temp.data.datasets[3].data = historyData.sensor_egt4;
      charts.temp.data.datasets[4].data = historyData.mvem_egt;
      charts.temp.update('none');
    }

    if (charts.pressures && charts.pressures.data && charts.pressures.data.datasets.length >= 4) {
      charts.pressures.data.labels = historyData.time;
      charts.pressures.data.datasets[0].data = historyData.sensor_oil_p;
      charts.pressures.data.datasets[1].data = historyData.mvem_oil_p;
      charts.pressures.data.datasets[2].data = historyData.sensor_map;
      charts.pressures.data.datasets[3].data = historyData.mvem_map;
      charts.pressures.update('none');
    }

    if (charts.vibration && charts.vibration.data && charts.vibration.data.datasets.length >= 2) {
      charts.vibration.data.labels = historyData.time;
      charts.vibration.data.datasets[0].data = historyData.sensor_vib;
      charts.vibration.data.datasets[1].data = historyData.mvem_vib;
      charts.vibration.update('none');
    }
  }
}

// -------------------------------------------------------------
// FLIGHT SANDBOX & MANUAL OVERRIDE CONTROLS
// -------------------------------------------------------------
let isManualSandbox = false;

function toggleManualSandbox() {
  isManualSandbox = !isManualSandbox;
  const btn = document.getElementById("btn-sandbox-toggle");
  if (btn) {
    if (isManualSandbox) {
      btn.className = "px-2 py-0.5 rounded text-[10px] bg-amber-950 hover:bg-amber-900 text-amber-300 border border-amber-700 font-bold";
      btn.innerText = "[Mode: MANUAL SANDBOX]";
      addEventLog("SYS", "Switched to Manual Flight Sandbox Mode.");
    } else {
      btn.className = "px-2 py-0.5 rounded text-[10px] bg-slate-800 hover:bg-slate-700 text-slate-300 border border-slate-700";
      btn.innerText = "[Mode: AUTO MISSION]";
      addEventLog("SYS", "Restored Auto Standard Mission Flight Profile.");
    }
  }
  onFlightControlChange();
}

function onFlightControlChange() {
  const elThr = document.getElementById("slider-throttle");
  const elAlt = document.getElementById("slider-altitude");
  const elOat = document.getElementById("slider-oat");

  const thr = elThr ? parseFloat(elThr.value) : 72.0;
  const alt = elAlt ? parseFloat(elAlt.value) : 28500.0;
  const oat = elOat ? parseFloat(elOat.value) : -20.0;

  setText("txt-slider-throttle", `${thr.toFixed(0)}%`);
  setText("txt-slider-altitude", `${(alt/1000).toFixed(1)}k FT`);
  setText("txt-slider-oat", `${oat.toFixed(0)}°C`);

  if (isManualSandbox) {
    fetch('/api/flight/override', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        enabled: true,
        throttle_pct: thr,
        altitude_ft: alt,
        ambient_temp_c: oat
      })
    }).catch(err => console.warn("Flight override error:", err));
  } else {
    fetch('/api/flight/override', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ enabled: false })
    }).catch(err => console.warn("Flight override disable error:", err));
  }
}

// ------------------------------------------------------------------
// 7. FAULT INJECTION & REAL-TIME EVENT HANDLERS
// ------------------------------------------------------------------
async function injectFault(faultType) {
  playTone(700, 0.08, "sine");
  // Immediate optimistic UI highlight
  updateActiveFaultButton(faultType);

  try {
    if (faultType === 'HEALTHY') {
      await fetch('/api/fault/clear', { method: 'POST' });
      addEventLog("FLT", "Engine reset to HEALTHY baseline. All trims normalized.");
    } else {
      const res = await fetch('/api/fault/inject', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ fault_type: faultType, severity: 1.0, ramp_duration_s: 1.0 })
      });
      const data = await res.json();
      addEventLog("FLT", `Injected: ${faultType.replace(/_/g, ' ')} (Ramp: 1.0s)`);
    }
  } catch (e) {
    console.error("Fault injection error:", e);
    addEventLog("ERR", `Fault injection network error: ${e.message}`);
  }
}

function updateActiveFaultButton(activeFaultName) {
  if (activeFaultName === lastActiveFaultButton) return; // Prevent continuous class manipulation
  lastActiveFaultButton = activeFaultName;
  currentActiveFault = activeFaultName;

  const statusTxt = document.getElementById("txt-active-status");

  // Reset all buttons to neutral state
  const allBtns = document.querySelectorAll(".btn-fault");
  allBtns.forEach(btn => {
    btn.classList.remove("active-fault", "active-sensor", "bg-emerald-950", "border-emerald-700", "text-emerald-300");
    btn.classList.add("bg-slate-900/90", "border-slate-800", "text-slate-300");
  });

  const activeBtn = document.getElementById(`btn-flt-${activeFaultName}`);
  if (activeBtn) {
    activeBtn.classList.remove("bg-slate-900/90", "border-slate-800", "text-slate-300");
    if (activeFaultName === "HEALTHY") {
      activeBtn.classList.add("bg-emerald-950", "border-emerald-700", "text-emerald-300");
      if (statusTxt) {
        statusTxt.className = "text-[10px] mono text-emerald-400 bg-emerald-950/80 px-2 py-0.5 rounded border border-emerald-800";
        statusTxt.innerText = "STATUS: HEALTHY BASELINE";
      }
    } else if (activeFaultName === "SENSOR_FAULT_EGT3") {
      activeBtn.classList.add("active-sensor");
      if (statusTxt) {
        statusTxt.className = "text-[10px] mono text-sky-400 bg-sky-950/80 px-2 py-0.5 rounded border border-sky-800";
        statusTxt.innerText = "STATUS: SENSOR PROBE DEFECT (EGT3) ACTIVE";
      }
    } else {
      activeBtn.classList.add("active-fault");
      if (statusTxt) {
        statusTxt.className = "text-[10px] mono text-red-400 bg-red-950/80 px-2 py-0.5 rounded border border-red-800";
        statusTxt.innerText = `STATUS: ${activeFaultName.replace(/_/g, ' ')} ACTIVE`;
      }
    }
  }
}

// ------------------------------------------------------------------
// 8. EVENT TIMELINE LOGGER
// ------------------------------------------------------------------
function addEventLog(tag, message) {
  const timestamp = new Date().toTimeString().split(' ')[0];
  eventLogs.unshift({ timestamp, tag, message });
  if (eventLogs.length > maxEventLogs) eventLogs.pop();

  const container = document.getElementById("container-logs");
  const countEl = document.getElementById("txt-log-count");
  if (countEl) countEl.innerText = `${eventLogs.length} events`;

  if (!container) return;
  container.innerHTML = eventLogs.map(log => {
    let tagColor = "text-emerald-400";
    if (log.tag === "FLT") tagColor = "text-amber-400";
    if (log.tag === "ERR") tagColor = "text-red-400";
    if (log.tag === "AI") tagColor = "text-sky-400";
    return `<div><span class="text-slate-500">[${log.timestamp}]</span> <span class="${tagColor} font-bold">[${log.tag}]</span> ${log.message}</div>`;
  }).join('');
}

// ------------------------------------------------------------------
// 9. AUTOMATED 4-PHASE STAGE DEMONSTRATION CONTROLLER
// ------------------------------------------------------------------
const autoDemoPhases = [
  { fault: "HEALTHY", duration: 6, title: "Phase 1/4: Baseline Nominal Cruise Tracking (100% Health)" },
  { fault: "SENSOR_FAULT_EGT3", duration: 8, title: "Phase 2/4: Thermocouple Probe Defect Decoupling (No Abort)" },
  { fault: "OIL_PRESSURE_LOSS", duration: 10, title: "Phase 3/4: Critical Oil Pressure Loss & SHAP Root-Cause" },
  { fault: "HEALTHY", duration: 6, title: "Phase 4/4: Recovery to Nominal Healthy Cruise Baseline" }
];

function toggleAutoDemo() {
  if (autoDemoActive) {
    stopAutoDemo();
  } else {
    startAutoDemo();
  }
}

function startAutoDemo() {
  autoDemoActive = true;
  autoDemoStep = 0;
  const bar = document.getElementById("box-autodemo-status");
  const btn = document.getElementById("btn-autodemo");
  if (bar) { bar.classList.remove("hidden"); bar.classList.add("flex"); }
  if (btn) { btn.classList.add("bg-amber-950", "border-amber-700", "text-amber-300"); }
  addEventLog("SYS", "Automated 4-Phase Stage Showcase started.");
  runNextDemoPhase();
}

function runNextDemoPhase() {
  if (!autoDemoActive) return;
  if (autoDemoStep >= autoDemoPhases.length) {
    stopAutoDemo();
    addEventLog("SYS", "Automated Showcase completed successfully.");
    return;
  }

  const phase = autoDemoPhases[autoDemoStep];
  injectFault(phase.fault);
  autoDemoTimeRemaining = phase.duration;

  const txtPhase = document.getElementById("txt-autodemo-phase");
  if (txtPhase) txtPhase.innerText = `${phase.title} (${autoDemoTimeRemaining}s)`;

  if (autoDemoTimer) clearInterval(autoDemoTimer);
  autoDemoTimer = setInterval(() => {
    autoDemoTimeRemaining--;
    if (txtPhase) txtPhase.innerText = `${phase.title} (${autoDemoTimeRemaining}s)`;
    if (autoDemoTimeRemaining <= 0) {
      clearInterval(autoDemoTimer);
      autoDemoStep++;
      runNextDemoPhase();
    }
  }, 1000);
}

function stopAutoDemo() {
  autoDemoActive = false;
  if (autoDemoTimer) clearInterval(autoDemoTimer);
  const bar = document.getElementById("box-autodemo-status");
  const btn = document.getElementById("btn-autodemo");
  if (bar) { bar.classList.remove("flex"); bar.classList.add("hidden"); }
  if (btn) { btn.classList.remove("bg-amber-950", "border-amber-700", "text-amber-300"); }
  injectFault("HEALTHY");
  addEventLog("SYS", "Automated Showcase stopped.");
}

// ------------------------------------------------------------------
// 10. SIMULATION CONTROLS (SPEED & CAMERA)
// ------------------------------------------------------------------
function setSimSpeed(speed) {
  fetch(`/api/sim/speed?speed=${speed}`, { method: 'POST' }).catch(e => {});
  ['1', '2', '5'].forEach(s => {
    const btn = document.getElementById(`btn-spd-${s}`);
    if (btn) {
      if (parseFloat(s) === speed) {
        btn.className = "px-2 py-0.5 bg-sky-950 border border-sky-700 text-sky-300 rounded font-bold";
      } else {
        btn.className = "px-2 py-0.5 bg-slate-800 border border-slate-700 text-slate-300 rounded";
      }
    }
  });
  addEventLog("SYS", `Simulation speed set to ${speed}x.`);
}

function setCameraPreset(preset) {
  if (engine3D && typeof engine3D.setCameraPreset === "function") {
    engine3D.setCameraPreset(preset);
  }
}

function resetSimulation() {
  fetch('/api/sim/reset', { method: 'POST' }).catch(e => {});
  injectFault('HEALTHY');
  addEventLog("SYS", "Simulation reset to initial flight condition.");
}

function updateBadge(id, text, bgClass, borderClass, textClass) {
  const el = document.getElementById(id);
  if (el) {
    el.className = `flex items-center space-x-1.5 px-2.5 py-1 rounded text-xs mono ${bgClass} border ${borderClass} ${textClass}`;
    const lbl = document.getElementById("lbl-connection");
    if (lbl) lbl.innerText = text;
  }
}

// ------------------------------------------------------------------
// 11. STEP 6: UNIFIED UAV DIGITAL TWIN DASHBOARD CONTROLLER
// ------------------------------------------------------------------
let currentActiveTab = "hybrid";
let lastUavTwinData = null;

function switchTab(tabId) {
  currentActiveTab = tabId;
  const panels = ["hybrid", "mission", "engine", "quality", "reliability", "decision", "predictive", "rerouting"];
  panels.forEach(id => {
    const p = document.getElementById(`tab-${id}`);
    const b = document.getElementById(`btn-tab-${id}`);
    if (p) {
      if (id === tabId) {
        p.classList.remove("hidden");
        if (id === "engine") p.classList.add("grid");
      } else {
        p.classList.add("hidden");
        if (id === "engine") p.classList.remove("grid");
      }
    }
    if (b) {
      if (id === tabId) {
        b.classList.add("active-tab");
      } else {
        b.classList.remove("active-tab");
      }
    }
  });

  if (tabId === "mission" && lastUavTwinData && lastUavTwinData.mission) {
    renderMissionCanvas(lastUavTwinData.mission);
  }
  if (tabId === "rerouting") {
    refreshAuditTrail();
  }
}

let isSimPaused = false;
function toggleSimPause() {
  isSimPaused = !isSimPaused;
  const endpoint = isSimPaused ? '/api/sim/pause' : '/api/sim/start';
  fetch(endpoint, { method: 'POST' }).catch(e => {});
  
  const icon = document.getElementById("icon-sim-play-pause");
  const lbl = document.getElementById("lbl-sim-play-pause");

  if (isSimPaused) {
    if (icon) icon.className = "fa-solid fa-play text-emerald-400 text-[10px]";
    if (lbl) lbl.innerText = "RESUME";
    addEventLog("SYS", "Simulation execution paused.");
  } else {
    if (icon) icon.className = "fa-solid fa-pause text-amber-400 text-[10px]";
    if (lbl) lbl.innerText = "PAUSE";
    addEventLog("SYS", "Simulation execution resumed.");
  }
}

function setHybridMode(mode) {
  fetch(`/api/hybrid/mode?mode=${mode}`, { method: 'POST' }).catch(e => {});
  ['AUTO', 'ENGINE_ONLY', 'ELECTRIC_ONLY', 'MOTOR_ASSIST', 'BATTERY_CHARGING'].forEach(m => {
    const btn = document.getElementById(`btn-mode-${m}`);
    if (btn) {
      if (m === mode) {
        btn.className = "px-2 py-0.5 rounded text-[10px] bg-sky-950 border border-sky-700 text-sky-300 font-bold";
      } else {
        btn.className = "px-2 py-0.5 rounded text-[10px] bg-slate-800 border border-slate-700 text-slate-300";
      }
    }
  });
  addEventLog("SYS", `Hybrid powertrain mode set to ${mode}.`);
}

function updateUavDigitalTwin(uav) {
  if (!uav) return;
  lastUavTwinData = uav;

  // 1. Digital Twin Quality Status Badge
  const qStatus = uav.quality_status || "HEALTHY";
  const badgeQ = document.getElementById("badge-twin-quality");
  const badgeQ2 = document.getElementById("badge-quality-master-2");
  if (badgeQ) {
    badgeQ.innerText = `TWIN: ${qStatus}`;
    if (qStatus === "HEALTHY") badgeQ.className = "text-[11px] px-2.5 py-0.5 rounded bg-emerald-950 text-emerald-400 font-mono font-bold border border-emerald-800";
    else if (qStatus === "WARNING") badgeQ.className = "text-[11px] px-2.5 py-0.5 rounded bg-amber-950 text-amber-400 font-mono font-bold border border-amber-800";
    else if (qStatus === "DEGRADED") badgeQ.className = "text-[11px] px-2.5 py-0.5 rounded bg-orange-950 text-orange-400 font-mono font-bold border border-orange-800";
    else badgeQ.className = "text-[11px] px-2.5 py-0.5 rounded bg-red-950 text-red-400 font-mono font-bold border border-red-800";
  }
  if (badgeQ2) {
    badgeQ2.innerText = `STATUS: ${qStatus}`;
    if (qStatus === "HEALTHY") badgeQ2.className = "text-[10px] px-2.5 py-0.5 rounded bg-emerald-950 border border-emerald-700 text-emerald-400 font-bold mono";
    else badgeQ2.className = "text-[10px] px-2.5 py-0.5 rounded bg-amber-950 border border-amber-700 text-amber-400 font-bold mono";
  }

  // 2. BESS Subsystem
  const b = uav.battery || {};
  const soc = b.soc_pct !== undefined ? b.soc_pct : 100.0;
  setText("val-bess-soc", `${soc.toFixed(1)}%`);
  
  const socCircle = document.getElementById("svg-bess-soc-circle");
  if (socCircle) {
    socCircle.setAttribute("stroke-dasharray", `${soc.toFixed(1)}, 100`);
    if (soc < 20.0) socCircle.setAttribute("class", "text-red-400 transition-all duration-300");
    else if (soc < 50.0) socCircle.setAttribute("class", "text-amber-400 transition-all duration-300");
    else socCircle.setAttribute("class", "text-emerald-400 transition-all duration-300");
  }

  setText("val-bess-energy", `${(b.usable_energy_kwh || 18.0).toFixed(2)} kWh`);
  const barEnergy = document.getElementById("bar-bess-energy");
  if (barEnergy) barEnergy.style.width = `${Math.min(100, Math.max(0, (b.usable_energy_kwh / 18.0) * 100))}%`;

  setText("val-bess-soh", `${(b.soh_pct || 100.0).toFixed(1)} %`);
  setText("val-bess-charge-state", b.charge_state || "IDLE");
  setText("val-bess-vterm", `${(b.terminal_voltage_v || 350.0).toFixed(1)} V`);
  setText("val-bess-current", `${(b.current_a || 0.0).toFixed(1)} A`);
  setText("val-bess-power", `${((b.power_actual_w || 0.0) / 1000.0).toFixed(1)} kW`);
  setText("val-bess-r-int", `${(b.internal_resistance_ohm || 0.025).toFixed(3)} Ω`);
  setText("val-bess-temp", `${(b.temperature_c || 25.0).toFixed(1)} °C`);
  setText("val-bess-ocv", `${(b.open_circuit_voltage_v || 350.0).toFixed(1)} V`);

  const badgeBessCons = document.getElementById("badge-bess-constraint");
  if (badgeBessCons) {
    if (b.is_power_constrained) {
      badgeBessCons.className = "text-[10px] px-2 py-0.5 rounded bg-red-950 border border-red-700 text-red-400 font-bold mono";
      badgeBessCons.innerText = "POWER: CONSTRAINED";
    } else {
      badgeBessCons.className = "text-[10px] px-2 py-0.5 rounded bg-emerald-950 border border-emerald-700 text-emerald-400 font-bold mono";
      badgeBessCons.innerText = "POWER: NOMINAL";
    }
  }

  // 3. Electric Machine Subsystem
  const em = uav.electric_machine || {};
  setText("val-em-mode", em.operating_mode || "IDLE");
  setText("val-em-p-mech", `${((em.mechanical_power_w || 0.0) / 1000.0).toFixed(1)} kW`);
  setText("val-em-p-elec", `${((em.electrical_power_w || 0.0) / 1000.0).toFixed(1)} kW`);
  setText("val-em-eff", `${((em.efficiency || 0.94) * 100).toFixed(1)} %`);
  setText("val-em-torque", `${(em.torque_nm || 0.0).toFixed(1)} Nm`);
  setText("val-em-rpm", `${(em.rpm || 0).toFixed(0)} RPM`);
  setText("val-em-temp", `${(em.temperature_c || 25.0).toFixed(1)} °C`);
  setText("val-em-loss", `${(em.power_loss_w || 0.0).toFixed(0)} W`);

  const badgeEmTherm = document.getElementById("badge-em-thermal");
  if (badgeEmTherm) {
    if (em.is_thermal_warning) {
      badgeEmTherm.className = "text-[10px] px-2 py-0.5 rounded bg-red-950 border border-red-700 text-red-400 font-bold mono";
      badgeEmTherm.innerText = "EM THERMAL: OVERHEAT";
    } else {
      badgeEmTherm.className = "text-[10px] px-2 py-0.5 rounded bg-emerald-950 border border-emerald-700 text-emerald-400 font-bold mono";
      badgeEmTherm.innerText = "EM THERMAL: NOMINAL";
    }
  }

  // 4. Hybrid Powertrain Subsystem
  const pt = uav.powertrain || {};
  const ptMode = pt.operating_mode || "ENGINE_ONLY";
  const badgePtMode = document.getElementById("badge-hybrid-mode");
  if (badgePtMode) badgePtMode.innerText = `MODE: ${ptMode}`;

  setText("val-powertrain-control", `[CONTROL: ${pt.control_mode || "AUTO"}]`);
  setText("val-powertrain-preq", `${((pt.propulsion_power_req_w || 0.0) / 1000.0).toFixed(1)} kW`);
  setText("val-powertrain-peng", `${((pt.engine_power_contribution_w || 0.0) / 1000.0).toFixed(1)} kW`);
  setText("val-powertrain-pem", `${((pt.electric_power_contribution_w || 0.0) / 1000.0).toFixed(1)} kW`);
  setText("val-powertrain-pdef", `${((pt.propulsion_power_deficit_w || 0.0) / 1000.0).toFixed(1)} kW`);

  const pAct = Math.max(1.0, pt.propulsion_power_actual_w || 1.0);
  const engPct = Math.min(100, Math.max(0, ((pt.engine_power_contribution_w || 0) / pAct) * 100));
  const emPct = Math.min(100, Math.max(0, 100 - engPct));
  setText("txt-eng-pct", `${engPct.toFixed(0)}%`);
  setText("txt-em-pct", `${emPct.toFixed(0)}%`);

  const barEng = document.getElementById("bar-eng-split");
  const barEm = document.getElementById("bar-em-split");
  if (barEng) barEng.style.width = `${engPct}%`;
  if (barEm) barEm.style.width = `${emPct}%`;

  // 5. Mission Navigation Subsystem
  const nav = uav.mission || {};
  const progPct = nav.mission_progress_pct !== undefined ? nav.mission_progress_pct : 0.0;
  setText("val-mission-prog-pct", `${progPct.toFixed(1)} %`);
  const barMission = document.getElementById("bar-mission-prog");
  if (barMission) barMission.style.width = `${progPct}%`;

  const currWpName = nav.next_waypoint ? nav.next_waypoint.name : `WP-${nav.current_waypoint_index || 0}`;
  setText("val-mission-active-wp-badge", `ACTIVE: ${currWpName}`);
  setText("val-mission-d-next", `${((nav.distance_to_next_waypoint_m || 0.0) / 1000.0).toFixed(1)} km`);
  setText("val-mission-d-home", `${((nav.distance_to_home_m || 0.0) / 1000.0).toFixed(1)} km`);
  setText("val-mission-d-rem", `${((nav.remaining_route_distance_m || 0.0) / 1000.0).toFixed(1)} km`);
  setText("val-mission-d-done", `${((nav.completed_route_distance_m || 0.0) / 1000.0).toFixed(1)} km`);

  function formatEta(sec) {
    if (sec === null || sec === undefined || sec <= 0) return "--:--";
    const m = Math.floor(sec / 60);
    const s = Math.floor(sec % 60);
    return `${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
  }
  setText("val-mission-eta-next", formatEta(nav.eta_to_next_waypoint_s));
  setText("val-mission-eta-home", formatEta(nav.eta_to_home_s));

  // Render 2D Mission Route Canvas
  renderMissionCanvas(nav);

  // 6. Mission Reliability & Dynamic Contingency
  const rel = uav.reliability || null;
  if (rel) {
    const rIndex = rel.mission_reliability_index !== undefined ? rel.mission_reliability_index : 100.0;
    const rStatus = rel.reliability_status || "NOMINAL";
    const rAction = rel.contingency_recommendation || "CONTINUE_MISSION";
    const rReason = rel.contingency_reason || "All systems nominal.";

    // Header Badges
    const badgeRelScore = document.getElementById("badge-reliability-score");
    if (badgeRelScore) {
      badgeRelScore.innerText = `RELIABILITY: ${rIndex.toFixed(1)}%`;
      if (rStatus === "NOMINAL") badgeRelScore.className = "text-[11px] px-2.5 py-0.5 rounded bg-emerald-950 text-emerald-400 font-mono font-bold border border-emerald-800";
      else if (rStatus === "ADVISORY") badgeRelScore.className = "text-[11px] px-2.5 py-0.5 rounded bg-sky-950 text-sky-400 font-mono font-bold border border-sky-800";
      else if (rStatus === "CAUTION") badgeRelScore.className = "text-[11px] px-2.5 py-0.5 rounded bg-amber-950 text-amber-400 font-mono font-bold border border-amber-800";
      else badgeRelScore.className = "text-[11px] px-2.5 py-0.5 rounded bg-red-950 text-red-400 font-mono font-bold border border-red-800 animate-pulse";
    }

    const badgeAction = document.getElementById("badge-contingency-action");
    if (badgeAction) {
      badgeAction.innerText = `ACTION: ${rAction}`;
      if (rAction === "CONTINUE_MISSION") badgeAction.className = "text-[11px] px-2.5 py-0.5 rounded bg-emerald-950 text-emerald-400 font-mono font-bold border border-emerald-800";
      else if (rAction === "ABORT_AND_RTB") badgeAction.className = "text-[11px] px-2.5 py-0.5 rounded bg-amber-950 text-amber-400 font-mono font-bold border border-amber-800";
      else if (rAction === "DIVERT_TO_ALTERNATE") badgeAction.className = "text-[11px] px-2.5 py-0.5 rounded bg-orange-950 text-orange-400 font-mono font-bold border border-orange-800";
      else badgeAction.className = "text-[11px] px-2.5 py-0.5 rounded bg-red-950 text-red-400 font-mono font-bold border border-red-800 animate-pulse";
    }

    // Card 1: Score & Factors
    setText("val-reliability-score", rIndex.toFixed(1));
    const badgeRelStat = document.getElementById("badge-reliability-status");
    if (badgeRelStat) {
      badgeRelStat.innerText = rStatus;
      if (rStatus === "NOMINAL") badgeRelStat.className = "text-[10px] px-2 py-0.5 rounded bg-emerald-950 border border-emerald-700 text-emerald-400 font-bold mono";
      else if (rStatus === "ADVISORY") badgeRelStat.className = "text-[10px] px-2 py-0.5 rounded bg-sky-950 border border-sky-700 text-sky-400 font-bold mono";
      else if (rStatus === "CAUTION") badgeRelStat.className = "text-[10px] px-2 py-0.5 rounded bg-amber-950 border border-amber-700 text-amber-400 font-bold mono";
      else badgeRelStat.className = "text-[10px] px-2 py-0.5 rounded bg-red-950 border border-red-700 text-red-400 font-bold mono animate-pulse";
    }

    const relCircle = document.getElementById("svg-reliability-circle");
    if (relCircle) {
      relCircle.setAttribute("stroke-dasharray", `${Math.max(0, Math.min(100, rIndex)).toFixed(1)}, 100`);
      if (rIndex >= 85.0) relCircle.setAttribute("class", "text-emerald-400 transition-all duration-300");
      else if (rIndex >= 65.0) relCircle.setAttribute("class", "text-sky-400 transition-all duration-300");
      else if (rIndex >= 40.0) relCircle.setAttribute("class", "text-amber-400 transition-all duration-300");
      else relCircle.setAttribute("class", "text-red-400 transition-all duration-300");
    }

    const sb = rel.score_breakdown || {};
    const fEnergy = sb.energy_reserve_factor !== undefined ? sb.energy_reserve_factor : 100;
    const fProp = sb.propulsion_health_factor !== undefined ? sb.propulsion_health_factor : 100;
    const fPower = sb.power_headroom_factor !== undefined ? sb.power_headroom_factor : 100;
    const fQual = sb.telemetry_quality_factor !== undefined ? sb.telemetry_quality_factor : 100;
    const fProg = sb.mission_progress_factor !== undefined ? sb.mission_progress_factor : 100;

    setText("val-factor-energy", `${fEnergy.toFixed(1)}%`);
    setText("val-factor-propulsion", `${fProp.toFixed(1)}%`);
    setText("val-factor-power", `${fPower.toFixed(1)}%`);
    setText("val-factor-quality", `${fQual.toFixed(1)}%`);
    setText("val-factor-progress", `${fProg.toFixed(1)}%`);

    const setBar = (id, val) => {
      const el = document.getElementById(id);
      if (el) el.style.width = `${Math.max(0, Math.min(100, val))}%`;
    };
    setBar("bar-factor-energy", fEnergy);
    setBar("bar-factor-propulsion", fProp);
    setBar("bar-factor-power", fPower);
    setBar("bar-factor-quality", fQual);
    setBar("bar-factor-progress", fProg);

    // Card 2: RTB Feasibility & Reserves
    const rtb = rel.rtb_prediction || {};
    const badgeRtb = document.getElementById("badge-rtb-feasible");
    if (badgeRtb) {
      if (rel.rtb_feasible) {
        badgeRtb.className = "text-[10px] px-2 py-0.5 rounded bg-emerald-950 border border-emerald-700 text-emerald-400 font-bold mono";
        badgeRtb.innerText = "RTB FEASIBLE";
      } else {
        badgeRtb.className = "text-[10px] px-2 py-0.5 rounded bg-red-950 border border-red-700 text-red-400 font-bold mono";
        badgeRtb.innerText = "RTB INFEASIBLE";
      }
    }

    setText("val-rtb-dist", `${((rtb.distance_to_home_m || 0) / 1000.0).toFixed(1)} km`);
    setText("val-rtb-time", formatEta(rel.estimated_rtb_time_s || rtb.estimated_duration_s));
    setText("val-rtb-energy", `${((rel.estimated_rtb_energy_wh || 0) / 1000.0).toFixed(2)} kWh`);
    setText("val-rtb-fuel", `${(rel.estimated_rtb_fuel_liters || 0).toFixed(2)} L`);
    setText("val-rtb-total-energy", `${((rtb.available_total_energy_wh || 0) / 1000.0).toFixed(1)} kWh`);
    
    const margin = rel.available_energy_margin_pct !== undefined ? rel.available_energy_margin_pct : (rtb.energy_margin_pct || 0);
    setText("val-rtb-margin", `${margin >= 0 ? '+' : ''}${margin.toFixed(1)}%`);

    const fuelRem = rel.available_fuel_liters !== undefined ? rel.available_fuel_liters : 150.0;
    setText("val-fuel-remaining", `${fuelRem.toFixed(1)} L`);
    setBar("bar-fuel-reserve", (fuelRem / 150.0) * 100.0);

    const battSoc = b.soc_pct !== undefined ? b.soc_pct : 100.0;
    setText("val-batt-reserve-soc", `${battSoc.toFixed(1)}%`);
    setBar("bar-batt-reserve", battSoc);

    setText("val-nearest-alt-name", rtb.nearest_alternate_id ? `${rtb.nearest_alternate_id}` : "ALT-01: Baramati");
    setText("val-nearest-alt-dist", `${((rtb.nearest_alternate_distance_m || 42500.0) / 1000.0).toFixed(1)} km`);
    const badgeNearestAlt = document.getElementById("badge-nearest-alt-feas");
    if (badgeNearestAlt) {
      if (rtb.nearest_alternate_feasible) {
        badgeNearestAlt.className = "text-[9px] px-1.5 py-0.5 rounded bg-emerald-950 text-emerald-400 border border-emerald-800 font-bold";
        badgeNearestAlt.innerText = "REACHABLE";
      } else {
        badgeNearestAlt.className = "text-[9px] px-1.5 py-0.5 rounded bg-red-950 text-red-400 border border-red-800 font-bold";
        badgeNearestAlt.innerText = "UNREACHABLE";
      }
    }

    // Card 3: Contingency Decision & Reason
    setText("val-contingency-action", rAction);
    setText("val-contingency-reason", rReason);

    const cardBox = document.getElementById("card-contingency-box");
    const badgePriority = document.getElementById("badge-contingency-priority");
    if (cardBox) {
      if (rAction === "CONTINUE_MISSION") {
        cardBox.className = "bg-emerald-950/40 border border-emerald-700/60 rounded-lg p-3.5 space-y-2";
        if (badgePriority) {
          badgePriority.innerText = "NORMAL";
          badgePriority.className = "text-[10px] px-2 py-0.5 rounded bg-emerald-950 border border-emerald-700 text-emerald-400 font-bold mono";
        }
      } else if (rAction === "ABORT_AND_RTB") {
        cardBox.className = "bg-amber-950/40 border border-amber-700/60 rounded-lg p-3.5 space-y-2";
        if (badgePriority) {
          badgePriority.innerText = "ADVISORY";
          badgePriority.className = "text-[10px] px-2 py-0.5 rounded bg-amber-950 border border-amber-700 text-amber-400 font-bold mono";
        }
      } else if (rAction === "DIVERT_TO_ALTERNATE") {
        cardBox.className = "bg-orange-950/40 border border-orange-700/60 rounded-lg p-3.5 space-y-2";
        if (badgePriority) {
          badgePriority.innerText = "CAUTION";
          badgePriority.className = "text-[10px] px-2 py-0.5 rounded bg-orange-950 border border-orange-700 text-orange-400 font-bold mono";
        }
      } else {
        cardBox.className = "bg-red-950/50 border border-red-700/80 rounded-lg p-3.5 space-y-2 animate-pulse";
        if (badgePriority) {
          badgePriority.innerText = "EMERGENCY";
          badgePriority.className = "text-[10px] px-2 py-0.5 rounded bg-red-950 border border-red-700 text-red-400 font-bold mono animate-pulse";
        }
      }
    }
  }

  // 8. AI Mission Decision Engine (Step 8)
  const dec = uav.decision || null;
  if (dec) {
    const dAction = dec.recommended_action || "CONTINUE_MISSION";
    const dReason = dec.decision_reason || "All parameters nominal.";
    const dConf = dec.decision_confidence !== undefined ? (dec.decision_confidence * 100.0) : 98.5;
    const dQuality = dec.data_quality_status || "HEALTHY";

    // Header Badges
    const badgeDec = document.getElementById("badge-decision-action");
    if (badgeDec) {
      badgeDec.innerText = `DECISION: ${dAction}`;
      if (dAction === "CONTINUE_MISSION") badgeDec.className = "text-[11px] px-2.5 py-0.5 rounded bg-emerald-950 text-emerald-400 font-mono font-bold border border-emerald-800";
      else if (dAction === "ABORT_AND_RTB") badgeDec.className = "text-[11px] px-2.5 py-0.5 rounded bg-amber-950 text-amber-400 font-mono font-bold border border-amber-800";
      else if (dAction === "DIVERT_TO_ALTERNATE") badgeDec.className = "text-[11px] px-2.5 py-0.5 rounded bg-orange-950 text-orange-400 font-mono font-bold border border-orange-800";
      else badgeDec.className = "text-[11px] px-2.5 py-0.5 rounded bg-red-950 text-red-400 font-mono font-bold border border-red-800 animate-pulse";
    }

    setText("badge-decision-confidence", `CONFIDENCE: ${dConf.toFixed(1)}%`);
    setText("val-decision-action", dAction);
    setText("val-decision-reason", dReason);
    setText("val-decision-quality-tag", `DATA: ${dQuality}`);
    setText("val-decision-target", dec.selected_destination || "Pune Lohegaon");
    setText("val-decision-mission-feas", dec.mission_feasible ? "YES" : "NO");
    setText("val-decision-rtb-feas", dec.rtb_feasible ? "YES" : "NO");
    setText("val-decision-alt-feas", dec.alternate_feasible ? "YES" : "NO");

    const badgeConf = document.getElementById("badge-decision-confidence");
    if (badgeConf) {
      if (dConf >= 90.0) badgeConf.className = "text-[10px] px-2 py-0.5 rounded bg-emerald-950 border border-emerald-700 text-emerald-400 font-bold mono";
      else if (dConf >= 75.0) badgeConf.className = "text-[10px] px-2 py-0.5 rounded bg-sky-950 border border-sky-700 text-sky-400 font-bold mono";
      else if (dConf >= 50.0) badgeConf.className = "text-[10px] px-2 py-0.5 rounded bg-amber-950 border border-amber-700 text-amber-400 font-bold mono";
      else badgeConf.className = "text-[10px] px-2 py-0.5 rounded bg-red-950 border border-red-700 text-red-400 font-bold mono";
    }

    const cardDecBox = document.getElementById("card-decision-box");
    if (cardDecBox) {
      if (dAction === "CONTINUE_MISSION") cardDecBox.className = "bg-emerald-950/40 border border-emerald-700/60 rounded-lg p-3.5 space-y-2";
      else if (dAction === "ABORT_AND_RTB") cardDecBox.className = "bg-amber-950/40 border border-amber-700/60 rounded-lg p-3.5 space-y-2";
      else if (dAction === "DIVERT_TO_ALTERNATE") cardDecBox.className = "bg-orange-950/40 border border-orange-700/60 rounded-lg p-3.5 space-y-2";
      else cardDecBox.className = "bg-red-950/50 border border-red-700/80 rounded-lg p-3.5 space-y-2 animate-pulse";
    }

    // Triggered Conditions list
    const condList = document.getElementById("list-triggered-conditions");
    if (condList) {
      const conds = dec.triggered_conditions || [];
      setText("val-conditions-count", `${conds.length} Active Condition${conds.length === 1 ? '' : 's'}`);
      if (conds.length === 0) {
        condList.innerHTML = `<div class="p-1 rounded bg-slate-950/60 border border-slate-800/60 text-slate-400">✓ No warning or abort conditions triggered.</div>`;
      } else {
        condList.innerHTML = conds.map(c => `
          <div class="p-1.5 rounded ${c.includes('CRITICAL') || c.includes('FAIL') || c.includes('EMERGENCY') ? 'bg-red-950/50 border border-red-800/70 text-red-300' : (c.includes('NOMINAL') ? 'bg-emerald-950/40 border border-emerald-800/60 text-emerald-300' : 'bg-amber-950/40 border border-amber-800/60 text-amber-300')}">
            • ${c}
          </div>
        `).join('');
      }
    }
  }

  // 9. Multi-Criteria Trajectory Optimizer (Step 8)
  const traj = uav.trajectory || null;
  if (traj) {
    const routeName = traj.recommended_route_name || "ROUTE_CONTINUE";
    const destName = traj.recommended_destination || "Pune Lohegaon (Home Base)";
    const tStatus = traj.status || "OPTIMAL";
    const tScore = traj.route_optimization_score !== undefined ? traj.route_optimization_score : 95.0;

    const badgeRoute = document.getElementById("badge-route-opt");
    if (badgeRoute) {
      badgeRoute.innerText = `ROUTE: ${routeName}`;
      if (tStatus === "OPTIMAL") badgeRoute.className = "text-[11px] px-2.5 py-0.5 rounded bg-sky-950 text-sky-300 font-mono font-bold border border-sky-800";
      else if (tStatus === "SUBOPTIMAL") badgeRoute.className = "text-[11px] px-2.5 py-0.5 rounded bg-amber-950 text-amber-300 font-mono font-bold border border-amber-800";
      else badgeRoute.className = "text-[11px] px-2.5 py-0.5 rounded bg-red-950 text-red-300 font-mono font-bold border border-red-800";
    }

    setText("badge-opt-status", `STATUS: ${tStatus}`);
    setText("badge-opt-score", `SCORE: ${tScore.toFixed(1)} / 100`);
    setText("val-opt-route-name", routeName);
    setText("val-opt-dest-name", destName);
    setText("val-opt-dist", `${(traj.route_distance_km || 0.0).toFixed(1)} km`);
    setText("val-opt-time", `${(traj.estimated_route_time_min || 0.0).toFixed(1)} min`);
    setText("val-opt-fuel", `${(traj.estimated_route_fuel_l || 0.0).toFixed(1)} L`);
    setText("val-opt-battery", `${(traj.estimated_route_battery_kwh || 0.0).toFixed(2)} kWh`);
    setText("val-opt-explanation", traj.explanation || "Optimal trajectory satisfying fuel and battery safety policies.");

    if (traj.selected_route && traj.selected_route.reserve_margin_pct !== undefined) {
      setText("val-opt-margin", `${traj.selected_route.reserve_margin_pct >= 0 ? '+' : ''}${traj.selected_route.reserve_margin_pct.toFixed(1)}%`);
    }

    // Candidate Trajectories Table
    const tbody = document.getElementById("table-candidates-body");
    if (tbody && traj.candidate_summaries && traj.candidate_summaries.length > 0) {
      tbody.innerHTML = traj.candidate_summaries.map(c => {
        const isSel = c.route_id === routeName;
        return `
          <tr class="${isSel ? 'bg-sky-950/40 text-sky-200 font-bold border-l-2 border-sky-400' : (c.is_feasible ? 'text-slate-300 hover:bg-slate-900/50' : 'text-slate-500 bg-red-950/20')}">
            <td class="py-1 px-1.5">${c.rank || 1}</td>
            <td class="py-1 px-1.5">${c.route_id}${isSel ? ' <span class="text-[9px] text-emerald-400 font-bold">[SELECTED]</span>' : ''}</td>
            <td class="py-1 px-1.5 text-slate-400">${c.route_type}</td>
            <td class="py-1 px-1.5">${c.destination_name}</td>
            <td class="py-1 px-1.5">${(c.total_distance_km || 0).toFixed(1)} km</td>
            <td class="py-1 px-1.5">${(c.total_duration_min || 0).toFixed(1)} m</td>
            <td class="py-1 px-1.5">${(c.total_fuel_l || 0).toFixed(1)} L</td>
            <td class="py-1 px-1.5">
              ${c.is_feasible ? '<span class="text-emerald-400">✓ YES</span>' : '<span class="text-red-400 font-bold">✗ NO</span>'}
            </td>
            <td class="py-1 px-1.5 text-right font-mono">${(c.multi_criteria_cost || 0).toFixed(3)}</td>
          </tr>
        `;
      }).join('');
    }
  }

  // 9b. Predictive Simulation & Risk Analysis (Step 9)
  if (uav.prediction) {
    const pred = uav.prediction;
    const pStatus = pred.prediction_status || "FEASIBLE";
    setText("val-pred-status", pStatus);
    const stEl = document.getElementById("val-pred-status");
    if (stEl) {
      stEl.className = pStatus === "FEASIBLE" ? "text-emerald-400 font-bold" : (pStatus === "CAUTION" ? "text-amber-400 font-bold" : "text-rose-400 font-bold");
    }

    setText("val-pred-count", `${pred.prediction_scenario_count || 0} RUNS`);
    setText("val-pred-time", `${(pred.prediction_computation_time_ms || 0).toFixed(0)} ms`);
    setText("badge-pred-rec-route", `REC: ${pred.recommended_predictive_route || "DIRECT_RTB"}`);

    const compRate = (pred.predicted_mission_completion_rate !== null && pred.predicted_mission_completion_rate !== undefined) ? pred.predicted_mission_completion_rate : 100.0;
    setText("val-pred-completion", `${compRate.toFixed(1)}%`);
    const compBar = document.getElementById("bar-pred-completion");
    if (compBar) compBar.style.width = `${Math.min(100, compRate)}%`;

    const rtbRate = (pred.predicted_rtb_feasibility_rate !== null && pred.predicted_rtb_feasibility_rate !== undefined) ? pred.predicted_rtb_feasibility_rate : 100.0;
    setText("val-pred-rtb", `${rtbRate.toFixed(1)}%`);
    const rtbBar = document.getElementById("bar-pred-rtb");
    if (rtbBar) rtbBar.style.width = `${Math.min(100, rtbRate)}%`;

    const fuelRisk = (pred.predicted_fuel_reserve_violation_rate !== null && pred.predicted_fuel_reserve_violation_rate !== undefined) ? pred.predicted_fuel_reserve_violation_rate : 0.0;
    setText("val-pred-fuel-risk", `${fuelRisk.toFixed(1)}%`);
    const fuelBar = document.getElementById("bar-pred-fuel-risk");
    if (fuelBar) fuelBar.style.width = `${Math.min(100, fuelRisk)}%`;

    const battRisk = (pred.predicted_battery_depletion_rate !== null && pred.predicted_battery_depletion_rate !== undefined) ? pred.predicted_battery_depletion_rate : 0.0;
    setText("val-pred-batt-risk", `${battRisk.toFixed(1)}%`);
    const battBar = document.getElementById("bar-pred-batt-risk");
    if (battBar) battBar.style.width = `${Math.min(100, battRisk)}%`;

    const thermRisk = ((pred.predicted_engine_thermal_risk || 0.0) + (pred.predicted_battery_thermal_risk || 0.0));
    setText("val-pred-thermal-risk", `${thermRisk.toFixed(1)}% (Engine &le; 220°C, Batt &le; 65°C)`);

    // Predictive Route Summaries Table
    if (pred.route_summaries && pred.route_summaries.length > 0) {
      const topSummary = pred.route_summaries[0];
      if (topSummary.completion_ci_95_pct) {
        setText("val-pred-ci", `95% CI [${topSummary.completion_ci_95_pct[0]}%, ${topSummary.completion_ci_95_pct[1]}%]`);
      }
      setText("val-pred-fuel-p50", `${(topSummary.fuel_remaining_p50_l || 0).toFixed(1)} L`);
      setText("val-pred-fuel-range", `[${(topSummary.fuel_remaining_p05_l || 0).toFixed(1)}L, ${(topSummary.fuel_remaining_p95_l || 0).toFixed(1)}L]`);
      setText("val-pred-soc-p50", `${(topSummary.battery_soc_p50_pct || 0).toFixed(1)}%`);
      setText("val-pred-soc-range", `[${(topSummary.battery_soc_p05_pct || 0).toFixed(1)}%, ${(topSummary.battery_soc_p95_pct || 0).toFixed(1)}%]`);
      setText("val-pred-explanation", `Recommended trajectory '${pred.recommended_predictive_route}' maintains ${compRate.toFixed(1)}% simulated ensemble completion rate under atmospheric turbulence and powertrain degradation.`);

      const pTable = document.getElementById("table-predictive-body");
      if (pTable) {
        pTable.innerHTML = pred.route_summaries.map(r => {
          const isRec = r.route_id === pred.recommended_predictive_route;
          return `
            <tr class="${isRec ? 'bg-cyan-950/40 text-cyan-200 font-bold border-l-2 border-cyan-400' : (r.is_statistically_feasible ? 'text-slate-300 hover:bg-slate-900/50' : 'text-slate-500 bg-red-950/20')}">
              <td class="py-1 px-1.5">${r.selection_rank || 1}</td>
              <td class="py-1 px-1.5">${r.route_id}${isRec ? ' <span class="text-[9px] text-emerald-400 font-bold">[OPTIMAL]</span>' : ''}</td>
              <td class="py-1 px-1.5">${r.destination_name}</td>
              <td class="py-1 px-1.5">${(r.distance_km || 0).toFixed(1)} km</td>
              <td class="py-1 px-1.5">${(r.duration_p50_min || 0).toFixed(1)} m</td>
              <td class="py-1 px-1.5">
                <span class="${r.completion_rate_pct >= 90 ? 'text-emerald-400' : 'text-amber-400'}">${r.completion_rate_pct.toFixed(1)}%</span>
              </td>
              <td class="py-1 px-1.5">
                <span class="${r.fuel_reserve_violation_rate_pct === 0 ? 'text-slate-400' : 'text-red-400'}">${r.fuel_reserve_violation_rate_pct.toFixed(1)}%</span>
              </td>
              <td class="py-1 px-1.5">
                ${r.is_statistically_feasible ? '<span class="text-emerald-400">✓ YES</span>' : '<span class="text-red-400 font-bold">✗ NO</span>'}
              </td>
              <td class="py-1 px-1.5 text-right font-mono text-cyan-300">${(r.predictive_score || 0).toFixed(1)}</td>
            </tr>
          `;
        }).join('');
      }
    }
  }

  // 10. Quality Audit Checklist
  const stale = uav.stale_data_flags || {};
  ['engine', 'battery', 'em', 'powertrain', 'env', 'mission', 'reliability'].forEach(key => {
    const apiKey = key === 'em' ? 'electric_machine' : (key === 'env' ? 'environment' : key);
    const isStale = stale[apiKey] || false;
    const el = document.getElementById(`stale-${key}`);
    if (el) {
      if (isStale) {
        el.className = "text-amber-400 font-bold";
        el.innerText = "STALE";
      } else {
        el.className = "text-emerald-400 font-bold";
        el.innerText = "FRESH";
      }
    }
  });

  // Validation Warnings/Errors Console
  const valLogsContainer = document.getElementById("container-validation-logs");
  if (valLogsContainer) {
    const warnings = uav.validation_warnings || [];
    const errors = uav.validation_errors || [];
    if (warnings.length === 0 && errors.length === 0) {
      valLogsContainer.innerHTML = `<div class="text-emerald-400">✓ All subsystems synchronized & physically consistent. 0 active warnings.</div>`;
    } else {
      let html = '';
      errors.forEach(err => { html += `<div class="text-red-400">✗ [ERROR] ${err}</div>`; });
      warnings.forEach(warn => { html += `<div class="text-amber-400">⚠ [WARN] ${warn}</div>`; });
      valLogsContainer.innerHTML = html;
    }
  }

  // 11. Step 10 Closed-Loop Mission Re-Routing & Operator Confirmation
  updateReroutingSubsystem(uav);
}

function renderMissionCanvas(mission) {
  const canvas = document.getElementById("canvas-mission-map");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  const width = canvas.width;
  const height = canvas.height;

  // Clear background
  ctx.fillStyle = "#090d16";
  ctx.fillRect(0, 0, width, height);

  // Draw grid background lines
  ctx.strokeStyle = "#1e293b";
  ctx.lineWidth = 1;
  for (let x = 0; x < width; x += 40) {
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, height); ctx.stroke();
  }
  for (let y = 0; y < height; y += 40) {
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke();
  }

  if (!mission || !mission.waypoints || mission.waypoints.length === 0) return;

  const wps = mission.waypoints;
  let minLat = 90, maxLat = -90, minLon = 180, maxLon = -180;
  wps.forEach(wp => {
    if (wp.latitude_deg < minLat) minLat = wp.latitude_deg;
    if (wp.latitude_deg > maxLat) maxLat = wp.latitude_deg;
    if (wp.longitude_deg < minLon) minLon = wp.longitude_deg;
    if (wp.longitude_deg > maxLon) maxLon = wp.longitude_deg;
  });

  const currPos = mission.current_position || wps[0];
  if (currPos.latitude_deg < minLat) minLat = currPos.latitude_deg;
  if (currPos.latitude_deg > maxLat) maxLat = currPos.latitude_deg;
  if (currPos.longitude_deg < minLon) minLon = currPos.longitude_deg;
  if (currPos.longitude_deg > maxLon) maxLon = currPos.longitude_deg;

  const latSpan = Math.max(0.01, (maxLat - minLat) * 1.3);
  const lonSpan = Math.max(0.01, (maxLon - minLon) * 1.3);
  const midLat = (minLat + maxLat) / 2;
  const midLon = (minLon + maxLon) / 2;

  function toPx(lat, lon) {
    const px = width / 2 + ((lon - midLon) / lonSpan) * (width * 0.85);
    const py = height / 2 - ((lat - midLat) / latSpan) * (height * 0.85);
    return { x: px, y: py };
  }

  // Draw Planned Route Line Segments
  ctx.strokeStyle = "#38bdf8";
  ctx.lineWidth = 2;
  ctx.setLineDash([5, 5]);
  ctx.beginPath();
  wps.forEach((wp, i) => {
    const p = toPx(wp.latitude_deg, wp.longitude_deg);
    if (i === 0) ctx.moveTo(p.x, p.y);
    else ctx.lineTo(p.x, p.y);
  });
  ctx.stroke();
  ctx.setLineDash([]);

  // Draw Completed Flight Path
  const homeP = toPx(wps[0].latitude_deg, wps[0].longitude_deg);
  const uavP = toPx(currPos.latitude_deg, currPos.longitude_deg);

  ctx.strokeStyle = "#10b981";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(homeP.x, homeP.y);
  ctx.lineTo(uavP.x, uavP.y);
  ctx.stroke();

  // Draw Waypoint Nodes
  wps.forEach((wp, i) => {
    const p = toPx(wp.latitude_deg, wp.longitude_deg);
    const isActive = (i === (mission.current_waypoint_index || 0));

    if (i === 0) {
      // Home Base Marker
      ctx.fillStyle = "#38bdf8";
      ctx.fillRect(p.x - 7, p.y - 7, 14, 14);
      ctx.fillStyle = "#090d16";
      ctx.font = "bold 9px JetBrains Mono";
      ctx.fillText("H", p.x - 3, p.y + 3);
    } else {
      // Waypoint Circle
      ctx.fillStyle = isActive ? "#f59e0b" : "#475569";
      ctx.beginPath();
      ctx.arc(p.x, p.y, isActive ? 8 : 5, 0, 2 * Math.PI);
      ctx.fill();

      if (isActive) {
        ctx.strokeStyle = "#f59e0b";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(p.x, p.y, 12, 0, 2 * Math.PI);
        ctx.stroke();
      }
    }

    ctx.fillStyle = isActive ? "#fbbf24" : "#94a3b8";
    ctx.font = "10px JetBrains Mono";
    ctx.fillText(`WP-${i}`, p.x + 10, p.y + 3);
  });

  // Draw UAV Marker
  ctx.save();
  ctx.translate(uavP.x, uavP.y);
  let angle = 0;
  if (mission.next_waypoint) {
    const nextP = toPx(mission.next_waypoint.latitude_deg, mission.next_waypoint.longitude_deg);
    angle = Math.atan2(nextP.y - uavP.y, nextP.x - uavP.x);
  }
  ctx.rotate(angle);

  ctx.fillStyle = "#10b981";
  ctx.strokeStyle = "#ffffff";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(12, 0);
  ctx.lineTo(-8, -7);
  ctx.lineTo(-4, 0);
  ctx.lineTo(-8, 7);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();

  ctx.fillStyle = "#10b981";
  ctx.font = "bold 10px JetBrains Mono";
  ctx.fillText(`UAV [${currPos.latitude_deg.toFixed(4)}°, ${currPos.longitude_deg.toFixed(4)}°]`, uavP.x + 12, uavP.y - 12);

  // If there is an active pending proposal or contingency divert route, draw proposed divert trajectory
  const reroute = lastUavTwinData ? lastUavTwinData.rerouting : null;
  const traj = lastUavTwinData ? lastUavTwinData.trajectory : null;
  if (reroute && reroute.active_proposal_id && traj && traj.candidate_routes) {
    const propCand = traj.candidate_routes.find(c => c.route_id === reroute.proposed_route_id);
    if (propCand && propCand.waypoints && propCand.waypoints.length > 0) {
      ctx.save();
      ctx.strokeStyle = "#f59e0b";
      ctx.lineWidth = 2.5;
      ctx.setLineDash([6, 4]);
      ctx.beginPath();
      ctx.moveTo(uavP.x, uavP.y);
      propCand.waypoints.forEach(wp => {
        const pt = toPx(wp.latitude_deg, wp.longitude_deg);
        ctx.lineTo(pt.x, pt.y);
      });
      ctx.stroke();
      ctx.setLineDash([]);

      // Draw Divert Destination Target Marker
      const lastWp = propCand.waypoints[propCand.waypoints.length - 1];
      const dPt = toPx(lastWp.latitude_deg, lastWp.longitude_deg);
      ctx.fillStyle = "#f59e0b";
      ctx.fillRect(dPt.x - 8, dPt.y - 8, 16, 16);
      ctx.fillStyle = "#090d16";
      ctx.font = "bold 9px JetBrains Mono";
      ctx.fillText("D", dPt.x - 3, dPt.y + 3);
      ctx.fillStyle = "#fbbf24";
      ctx.font = "bold 10px JetBrains Mono";
      ctx.fillText(`PROPOSED DIVERT [${propCand.destination_name}]`, dPt.x + 12, dPt.y + 3);
      ctx.restore();
    }
  }
}

// ------------------------------------------------------------------
// STEP 10: Closed-Loop Mission Re-Routing & Operator Confirmation
// ------------------------------------------------------------------
let activePendingProposalId = null;

function updateReroutingSubsystem(uav) {
  if (!uav) return;
  const rr = uav.rerouting || {};
  const mission = uav.mission || {};
  const status = rr.rerouting_subsystem_status || "NOMINAL";
  const pendingCount = rr.pending_proposal_count || 0;
  const hasContingency = rr.contingency_detected || false;
  const activeProposalId = rr.active_proposal_id || null;

  // 1. Header & Tab Status Badges
  const badgeStatus = document.getElementById("badge-reroute-status");
  if (badgeStatus) {
    badgeStatus.innerText = `REROUTE: ${status}`;
    if (status === "NOMINAL") {
      badgeStatus.className = "text-[11px] px-2.5 py-0.5 rounded bg-slate-900 text-slate-300 font-mono font-bold border border-slate-700";
    } else if (status === "CONTINGENCY_DETECTED" || status === "PENDING_APPROVAL") {
      badgeStatus.className = "text-[11px] px-2.5 py-0.5 rounded bg-amber-950 text-amber-300 font-mono font-bold border border-amber-600 animate-pulse";
    } else if (status === "ROUTE_UPDATED") {
      badgeStatus.className = "text-[11px] px-2.5 py-0.5 rounded bg-cyan-950 text-cyan-300 font-mono font-bold border border-cyan-600";
    }
  }

  const badgeTabPending = document.getElementById("badge-tab-reroute-pending");
  if (badgeTabPending) {
    if (pendingCount > 0) {
      badgeTabPending.classList.remove("hidden");
      badgeTabPending.innerText = `${pendingCount} !`;
    } else {
      badgeTabPending.classList.add("hidden");
    }
  }

  // 2. Section A: Current Navigation & Waypoint State
  const activeRouteId = rr.active_route_id || mission.active_route_id || "ORIGINAL_MISSION_ROUTE";
  setText("txt-reroute-active-id", activeRouteId);
  setText("txt-reroute-dest", rr.proposed_destination || (mission.home_position ? mission.home_position.name : "Pune Lohegaon (Home Base)"));

  if (mission.current_position) {
    const lat = mission.current_position.latitude_deg.toFixed(4);
    const lon = mission.current_position.longitude_deg.toFixed(4);
    const alt = mission.current_position.altitude_m.toFixed(0);
    setText("txt-reroute-pos", `${lat}°N, ${lon}°E (${alt}m)`);
  }

  if (mission.current_waypoint) {
    setText("txt-reroute-active-wp", `WP-${mission.current_waypoint_index} (${mission.current_waypoint.name})`);
  }

  const remDistKm = (mission.remaining_route_distance_m || 0.0) / 1000.0;
  setText("txt-reroute-rem-dist", `${remDistKm.toFixed(1)} km`);
  setText("txt-reroute-progress", `${(mission.mission_progress_pct || 0.0).toFixed(1)}%`);

  const routeVerBadge = document.getElementById("badge-reroute-route-version");
  if (routeVerBadge) {
    const isDiverted = activeRouteId !== "ORIGINAL_MISSION_ROUTE";
    routeVerBadge.innerText = isDiverted ? `DIVERTED (${activeRouteId})` : "ROUTE v1 (ORIGINAL)";
    routeVerBadge.className = isDiverted 
      ? "text-[10px] px-2 py-0.5 rounded bg-amber-950 border border-amber-700 text-amber-300 mono font-bold"
      : "text-[10px] px-2 py-0.5 rounded bg-sky-950 border border-sky-700 text-sky-300 mono font-bold";
  }

  // 3. Section B: Contingency Alert Monitor
  const badgeCont = document.getElementById("badge-reroute-contingency-flag");
  const boxContAlert = document.getElementById("box-reroute-contingency-alert");
  if (hasContingency) {
    setText("txt-reroute-contingency-status", `CONTINGENCY: ${rr.contingency_action || "ALERT"}`);
    if (badgeCont) {
      badgeCont.innerText = "CONTINGENCY DETECTED";
      badgeCont.className = "text-[10px] px-2 py-0.5 rounded bg-rose-950 border border-rose-600 text-rose-300 mono font-bold animate-pulse";
    }
    if (boxContAlert) {
      boxContAlert.className = "bg-rose-950/40 p-2.5 rounded border border-rose-700/60 text-xs font-mono space-y-1";
    }
    setText("val-reroute-trigger", rr.contingency_action || "SAFETY_ALERT");
    setText("val-reroute-rec-action", rr.contingency_action || "DIVERT");
    setText("val-reroute-target-dest", rr.proposed_destination || "Alternate Airfield");
    setText("val-reroute-advisory", "Telemetry safety limits breached. Recommended flight trajectory generated and pending pilot confirmation.");
  } else {
    setText("txt-reroute-contingency-status", "NOMINAL (NO DIVERSION)");
    if (badgeCont) {
      badgeCont.innerText = "ALL SYSTEMS NOMINAL";
      badgeCont.className = "text-[10px] px-2 py-0.5 rounded bg-emerald-950 border border-emerald-700 text-emerald-400 mono font-bold";
    }
    if (boxContAlert) {
      boxContAlert.className = "bg-slate-900/90 p-2.5 rounded border border-slate-800 text-xs font-mono space-y-1";
    }
    setText("val-reroute-trigger", "NONE");
    setText("val-reroute-rec-action", "CONTINUE_MISSION");
    setText("val-reroute-target-dest", "Pune Lohegaon (Home Base)");
    setText("val-reroute-advisory", "Normal flight conditions. Automatic contingency detection active across engine health, battery SoC, thermal limits, and fuel reserves.");
  }

  // Operator Attention
  const attentionRequired = rr.operator_attention_required || false;
  const txtAtt = document.getElementById("txt-reroute-attention-status");
  if (txtAtt) {
    if (attentionRequired) {
      txtAtt.innerText = "PILOT CONFIRMATION REQUIRED";
      txtAtt.className = "font-bold text-amber-400 animate-pulse";
    } else {
      txtAtt.innerText = "NONE REQUIRED";
      txtAtt.className = "font-bold text-slate-400";
    }
  }

  // 4. Section C: Proposed Route Comparison Matrix
  const cmpActiveDest = document.getElementById("cmp-active-dest");
  if (cmpActiveDest) {
    cmpActiveDest.innerText = mission.home_position ? mission.home_position.name : "Pune Lohegaon";
    setText("cmp-active-dist", `${remDistKm.toFixed(1)} km`);
    const actTimeMin = (remDistKm / Math.max(1.0, (uav.environment ? uav.environment.airspeed_mps : 55.0) * 0.06)).toFixed(1);
    setText("cmp-active-time", `${actTimeMin} min`);
    setText("cmp-active-fuel", `${(uav.engine ? 45.0 : 40.0).toFixed(1)} L`);
    setText("cmp-active-soc", `${(uav.battery ? uav.battery.soc_pct : 100.0).toFixed(1)}%`);
    setText("cmp-active-therm", "0.0%");
    setText("cmp-active-feas", hasContingency ? "COMPROMISED" : "FEASIBLE");
  }

  // If there's a predictive report or candidate route, update comparison values
  if (uav.trajectory && uav.trajectory.candidate_routes && rr.proposed_route_id) {
    const cand = uav.trajectory.candidate_routes.find(c => c.route_id === rr.proposed_route_id);
    if (cand) {
      setText("cmp-prop-dest", cand.destination_name);
      setText("cmp-delta-dest", "DIVERT AIRFIELD");
      setText("cmp-prop-dist", `${(cand.total_distance_m / 1000.0).toFixed(1)} km`);
      const deltaDist = (cand.total_distance_m / 1000.0) - remDistKm;
      const dDistEl = document.getElementById("cmp-delta-dist");
      if (dDistEl) {
        dDistEl.innerText = `${deltaDist >= 0 ? '+' : ''}${deltaDist.toFixed(1)} km`;
        dDistEl.className = deltaDist < 0 ? "py-1 px-2 text-right text-emerald-400 font-bold" : "py-1 px-2 text-right text-amber-400";
      }
      setText("cmp-prop-time", `${(cand.estimated_flight_time_s / 60.0).toFixed(1)} min`);
      setText("cmp-prop-fuel", `${cand.estimated_fuel_consumed_l ? (50.0 - cand.estimated_fuel_consumed_l).toFixed(1) : "38.5"} L`);
      setText("cmp-prop-soc", "42.0%");
      setText("cmp-prop-therm", "0.0%");
      setText("cmp-prop-feas", cand.is_feasible ? "FEASIBLE" : "INFEASIBLE");
    }
  } else {
    setText("cmp-prop-dest", "—");
    setText("cmp-delta-dest", "—");
    setText("cmp-prop-dist", "—");
    setText("cmp-delta-dist", "—");
    setText("cmp-prop-time", "—");
    setText("cmp-delta-time", "—");
    setText("cmp-prop-fuel", "—");
    setText("cmp-delta-fuel", "—");
    setText("cmp-prop-soc", "—");
    setText("cmp-delta-soc", "—");
    setText("cmp-prop-therm", "—");
    setText("cmp-delta-therm", "—");
    setText("cmp-prop-feas", "—");
  }

  // 5. Section D: Operator Decision Panel
  const badgeOpPending = document.getElementById("badge-operator-pending");
  if (badgeOpPending) {
    badgeOpPending.innerText = `${pendingCount} PENDING`;
    badgeOpPending.className = pendingCount > 0 
      ? "text-[10px] px-2 py-0.5 rounded bg-amber-950 border border-amber-600 text-amber-300 mono font-bold"
      : "text-[10px] px-2 py-0.5 rounded bg-slate-800 text-slate-400 mono";
  }

  const vNoPending = document.getElementById("view-no-pending-proposal");
  const vActivePending = document.getElementById("view-active-pending-proposal");

  if (activeProposalId && pendingCount > 0) {
    activePendingProposalId = activeProposalId;
    if (vNoPending) vNoPending.classList.add("hidden");
    if (vActivePending) vActivePending.classList.remove("hidden");

    setText("txt-prop-id", activeProposalId);
    setText("txt-prop-dest", rr.proposed_destination || "Pune Lohegaon (Direct RTB)");
    setText("txt-prop-route-id", rr.proposed_route_id || "ROUTE_DIRECT_RTB");
    setText("txt-prop-dist", `${remDistKm.toFixed(1)} km`);
    setText("txt-prop-justification", `Contingency trigger: ${rr.contingency_action || "ANOMALY"}. Pilot authorization required before route plan modification.`);
  } else {
    activePendingProposalId = null;
    if (vNoPending) vNoPending.classList.remove("hidden");
    if (vActivePending) vActivePending.classList.add("hidden");
  }
}

// Modal Approval Functions
function confirmApproveModal() {
  if (!activePendingProposalId) return;
  const m = document.getElementById("modal-confirm-reroute");
  if (m) {
    setText("modal-prop-id", activePendingProposalId);
    const destEl = document.getElementById("txt-prop-dest");
    setText("modal-prop-dest", destEl ? destEl.innerText : "Proposed Destination");
    const posEl = document.getElementById("txt-reroute-pos");
    setText("modal-prop-pos", posEl ? posEl.innerText : "Current Position");
    m.classList.remove("hidden");
  }
}

function closeApproveModal() {
  const m = document.getElementById("modal-confirm-reroute");
  if (m) m.classList.add("hidden");
}

async function executeApproveProposal() {
  if (!activePendingProposalId) return;
  closeApproveModal();
  const feedbackEl = document.getElementById("txt-operator-feedback");
  if (feedbackEl) feedbackEl.innerText = `Submitting approval for ${activePendingProposalId}...`;

  try {
    const res = await fetch(`/api/mission/proposals/${activePendingProposalId}/approve`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ operator_id: "PILOT-GCS-01" })
    });
    const data = await res.json();
    if (res.ok) {
      if (feedbackEl) {
        feedbackEl.innerText = `✓ ROUTE APPROVED: ${data.message}`;
        feedbackEl.className = "text-[10px] mono text-emerald-400 bg-emerald-950/60 p-1.5 rounded border border-emerald-800 text-center";
      }
      addEventLog("PILOT", `Route proposal ${activePendingProposalId} APPROVED. Active route updated to ${data.active_route_id}.`);
      refreshAuditTrail();
    } else {
      if (feedbackEl) {
        feedbackEl.innerText = `✗ APPROVAL FAILED: ${data.detail || data.message}`;
        feedbackEl.className = "text-[10px] mono text-red-400 bg-red-950/60 p-1.5 rounded border border-red-800 text-center";
      }
    }
  } catch (err) {
    if (feedbackEl) {
      feedbackEl.innerText = `✗ Network error during route approval: ${err}`;
      feedbackEl.className = "text-[10px] mono text-red-400 bg-red-950/60 p-1.5 rounded border border-red-800 text-center";
    }
  }
}

async function executeRejectProposal() {
  if (!activePendingProposalId) return;
  const reasonInput = document.getElementById("input-reject-reason");
  const reason = (reasonInput && reasonInput.value.trim()) ? reasonInput.value.trim() : "Rejected by pilot in command";
  const feedbackEl = document.getElementById("txt-operator-feedback");
  if (feedbackEl) feedbackEl.innerText = `Submitting rejection for ${activePendingProposalId}...`;

  try {
    const res = await fetch(`/api/mission/proposals/${activePendingProposalId}/reject`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ operator_id: "PILOT-GCS-01", reason: reason })
    });
    const data = await res.json();
    if (res.ok) {
      if (feedbackEl) {
        feedbackEl.innerText = `✓ PROPOSAL REJECTED: ${data.message}`;
        feedbackEl.className = "text-[10px] mono text-slate-300 bg-slate-900/60 p-1.5 rounded border border-slate-700 text-center";
      }
      addEventLog("PILOT", `Route proposal ${activePendingProposalId} REJECTED. Reason: ${reason}`);
      if (reasonInput) reasonInput.value = "";
      refreshAuditTrail();
    } else {
      if (feedbackEl) {
        feedbackEl.innerText = `✗ REJECTION FAILED: ${data.detail || data.message}`;
        feedbackEl.className = "text-[10px] mono text-red-400 bg-red-950/60 p-1.5 rounded border border-red-800 text-center";
      }
    }
  } catch (err) {
    if (feedbackEl) {
      feedbackEl.innerText = `✗ Network error during proposal rejection: ${err}`;
      feedbackEl.className = "text-[10px] mono text-red-400 bg-red-950/60 p-1.5 rounded border border-red-800 text-center";
    }
  }
}

async function refreshAuditTrail() {
  try {
    const res = await fetch("/api/mission/audit");
    if (!res.ok) return;
    const data = await res.json();
    const tableBody = document.getElementById("table-audit-body");
    if (!tableBody) return;

    const events = data.audit_events || [];
    if (events.length === 0) {
      tableBody.innerHTML = `<tr><td colspan="6" class="py-2 text-center text-slate-500">Audit trail initialized. Ready for events.</td></tr>`;
      return;
    }

    const sorted = [...events].reverse();
    tableBody.innerHTML = sorted.map(ev => {
      let typeColor = "text-slate-300";
      if (ev.event_type.includes("APPROVED")) typeColor = "text-emerald-400 font-bold";
      else if (ev.event_type.includes("REJECTED")) typeColor = "text-rose-400";
      else if (ev.event_type.includes("CREATED")) typeColor = "text-amber-400";

      const t = ev.timestamp_s ? `${ev.timestamp_s.toFixed(1)}s` : "0.0s";
      return `
        <tr class="hover:bg-slate-900/40">
          <td class="py-1 px-1.5 text-slate-400">${t}</td>
          <td class="py-1 px-1.5 ${typeColor}">${ev.event_type}</td>
          <td class="py-1 px-1.5 text-slate-300">${ev.proposal_id || "—"}</td>
          <td class="py-1 px-1.5 text-slate-400">${ev.operator_id || "SYSTEM"}</td>
          <td class="py-1 px-1.5 text-slate-300">${ev.reason || "—"}</td>
          <td class="py-1 px-1.5 text-right font-bold ${ev.result === 'SUCCESS' ? 'text-emerald-400' : 'text-amber-400'}">${ev.result}</td>
        </tr>
      `;
    }).join("");
  } catch (e) {
    console.warn("Audit trail fetch error:", e);
  }
}
