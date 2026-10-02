<script>
  import { onMount } from 'svelte';

  let ws;
  let isConnected = false;
  let isCalibrating = false;

  // 실시간 텔레메트리
  let pressure = 0;
  let angle = 0;
  let controlMode = 0; // 0: Continuous, 1: Hysteresis

  // 튜닝 파라미터
  let emaAlpha = 0.25;
  let thresholdHigh = 2200;
  let thresholdLow = 800;

  function connectWs() {
    const host = window.location.hostname || '192.168.4.1';
    ws = new WebSocket(`ws://${host}/ws`);

    ws.onopen = () => {
      isConnected = true;
    };

    ws.onclose = () => {
      isConnected = false;
      setTimeout(connectWs, 1500);
    };

    ws.onmessage = (evt) => {
      try {
        const data = JSON.parse(evt.data);
        pressure = data.p;
        angle = data.a;
        controlMode = data.m;
      } catch (e) {}
    };
  }

  function sendCmd(cmd, payload = {}) {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({ cmd, ...payload }));
    }
  }

  function setMode(mode) {
    controlMode = mode;
    sendCmd('set_mode', { val: mode });
  }

  function handleAlphaChange(e) {
    emaAlpha = parseFloat(e.target.value);
    sendCmd('set_alpha', { val: emaAlpha });
  }

  function handleThresholdChange() {
    sendCmd('set_threshold', { high: thresholdHigh, low: thresholdLow });
  }

  function toggleCalib() {
    isCalibrating = !isCalibrating;
    if (isCalibrating) {
      sendCmd('start_calib');
    } else {
      sendCmd('finish_calib');
    }
  }

  onMount(() => {
    connectWs();
  });
</script>

<main class="container">
  <header>
    <h1>여섯 번째 손가락 (Svelte 대시보드)</h1>
    <div class="status-badge">
      <div class="status-dot" class:online={isConnected}></div>
      <span>{isConnected ? '실시간 연결됨' : '연결 끊김'}</span>
    </div>
  </header>

  <!-- 실시간 모니터링 카드 -->
  <section class="card">
    <h2>실시간 센서 & 서보 모니터링</h2>
    <div class="gauge-grid">
      <div class="gauge-box">
        <div class="gauge-label">발 족압 센서 (FSR)</div>
        <div class="gauge-val">{pressure}</div>
        <div class="gauge-label">ADC 12-bit (0 ~ 4095)</div>
        <div class="bar-bg">
          <div class="bar-fill" style="width: {(pressure / 4095) * 100}%"></div>
        </div>
      </div>

      <div class="gauge-box">
        <div class="gauge-label">서보 모터 각도</div>
        <div class="gauge-val">{angle}°</div>
        <div class="gauge-label">가동 범위 (0° ~ 180°)</div>
        <div class="bar-bg">
          <div class="bar-fill servo" style="width: {(angle / 180) * 100}%"></div>
        </div>
      </div>
    </div>
  </section>

  <!-- 제어 모드 및 민감도 카드 -->
  <section class="card">
    <h2>제어 모드 & 민감도 설정</h2>
    <div class="toggle-row">
      <button 
        class="toggle-btn" 
        class:active={controlMode === 0}
        on:click={() => setMode(0)}>
        연속 비례 추종 (Continuous)
      </button>
      <button 
        class="toggle-btn" 
        class:active={controlMode === 1}
        on:click={() => setMode(1)}>
        파지 래치 (Hysteresis)
      </button>
    </div>

    <div class="control-group">
      <div class="control-header">
        <span>EMA 필터 계수 (α)</span>
        <span class="highlight">{emaAlpha}</span>
      </div>
      <input 
        type="range" 
        min="0.05" 
        max="0.95" 
        step="0.05" 
        bind:value={emaAlpha} 
        on:change={handleAlphaChange} 
      />
    </div>

    <div class="control-group">
      <div class="control-header">
        <span>파지 임계값 (Threshold HIGH)</span>
        <span class="highlight-warn">{thresholdHigh}</span>
      </div>
      <input 
        type="range" 
        min="1000" 
        max="3800" 
        step="50" 
        bind:value={thresholdHigh} 
        on:change={handleThresholdChange} 
      />

      <div class="control-header" style="margin-top:10px;">
        <span>해제 임계값 (Threshold LOW)</span>
        <span class="highlight-accent">{thresholdLow}</span>
      </div>
      <input 
        type="range" 
        min="200" 
        max="2000" 
        step="50" 
        bind:value={thresholdLow} 
        on:change={handleThresholdChange} 
      />
    </div>
  </section>

  <!-- 캘리브레이션 및 NVS 저장 카드 -->
  <section class="card">
    <h2>캘리브레이션 & LUT 메모리 관리</h2>
    <div class="btn-grid">
      <button class="btn btn-warn" on:click={() => sendCmd('reset_lut')}>
        arccos 기본 LUT 복원
      </button>
      <button 
        class="btn" 
        class:btn-accent={!isCalibrating}
        class:btn-danger={isCalibrating}
        on:click={toggleCalib}>
        {isCalibrating ? '플렉스 센서 보정 종료' : '플렉스 센서 보정 시작'}
      </button>
      <button class="btn btn-primary btn-full" on:click={() => sendCmd('save_nvs')}>
        설정 및 LUT 영구 저장 (Flash NVS)
      </button>
    </div>
  </section>
</main>

<style>
  :global(:root) {
    --bg: #0f172a;
    --card-bg: #1e293b;
    --text: #f8fafc;
    --text-muted: #94a3b8;
    --primary: #38bdf8;
    --accent: #10b981;
    --warn: #f59e0b;
    --danger: #ef4444;
    --border: #334155;
  }
  :global(body) {
    background: var(--bg);
    color: var(--text);
    margin: 0;
    padding: 16px;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
  }
  .container {
    max-width: 600px;
    margin: 0 auto;
    display: flex;
    flex-direction: column;
    gap: 16px;
  }
  header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding-bottom: 8px;
    border-bottom: 1px solid var(--border);
  }
  h1 { font-size: 1.2rem; font-weight: 700; color: var(--primary); margin: 0; }
  .status-badge { display: flex; align-items: center; gap: 6px; font-size: 0.85rem; padding: 4px 10px; border-radius: 9999px; background: #334155; }
  .status-dot { width: 8px; height: 8px; border-radius: 50%; background: var(--danger); }
  .status-dot.online { background: var(--accent); box-shadow: 0 0 8px var(--accent); }
  
  .card { background: var(--card-bg); border-radius: 12px; padding: 16px; border: 1px solid var(--border); }
  .card h2 { font-size: 0.95rem; margin-top: 0; margin-bottom: 12px; color: var(--text); }

  .gauge-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
  .gauge-box { background: #0f172a; border-radius: 8px; padding: 12px; text-align: center; }
  .gauge-val { font-size: 1.8rem; font-weight: 800; color: var(--primary); margin: 4px 0; }
  .gauge-label { font-size: 0.75rem; color: var(--text-muted); }
  .bar-bg { width: 100%; height: 8px; background: #334155; border-radius: 4px; overflow: hidden; margin-top: 8px; }
  .bar-fill { height: 100%; background: var(--primary); transition: width 0.08s ease; }
  .bar-fill.servo { background: var(--accent); }

  .toggle-row { display: flex; background: #0f172a; border-radius: 8px; padding: 4px; gap: 4px; margin-bottom: 16px; }
  .toggle-btn { flex: 1; padding: 10px; border: none; background: transparent; color: var(--text-muted); font-size: 0.85rem; font-weight: 600; border-radius: 6px; cursor: pointer; transition: 0.2s; }
  .toggle-btn.active { background: var(--primary); color: #0f172a; }

  .control-group { margin-bottom: 14px; }
  .control-header { display: flex; justify-content: space-between; font-size: 0.85rem; margin-bottom: 6px; }
  input[type=range] { width: 100%; accent-color: var(--primary); cursor: pointer; }

  .highlight { color: var(--primary); font-weight: bold; }
  .highlight-warn { color: var(--warn); font-weight: bold; }
  .highlight-accent { color: var(--accent); font-weight: bold; }

  .btn-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
  .btn { padding: 10px 14px; border: none; border-radius: 8px; font-size: 0.85rem; font-weight: 600; cursor: pointer; transition: 0.2s; background: #334155; color: var(--text); }
  .btn-primary { background: var(--primary); color: #0f172a; }
  .btn-accent { background: var(--accent); color: #0f172a; }
  .btn-warn { background: var(--warn); color: #0f172a; }
  .btn-danger { background: var(--danger); color: white; }
  .btn-full { grid-column: span 2; }
</style>
