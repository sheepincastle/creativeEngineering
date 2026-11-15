<script>
  import { onMount, tick } from 'svelte';

  let ws;
  let isConnected = false;
  let activeTab = 'monitor'; // 'monitor' | 'params' | 'terminal'

  // 1~5. 튜닝 파라미터 (NVS 저장 대상)
  let emaAlpha = 0.25;
  let maxStepAlpha = 35.0;
  let expCurveK = 2.5;
  let couplingGamma = 0.25;
  let stallSafeDemand = 140;
  let stallCutoffDemand = 150;
  let stallProtectSec = 5.0;

  // 7. 실시간 모니터링 상태 (1번 모터: 1·2번 마디, 2번 모터: 3번 마디)
  let p1 = 0, des1 = 0, a1 = 0, dem1 = 0, step1 = 0, target1 = 0, ext1 = 0, flx1 = 180, cal1 = false, stall1 = false;
  let p2 = 0, des2 = 0, a2 = 0, dem2 = 0, step2 = 0, pure2 = 0, target2 = 0, ext2 = 0, flx2 = 180, cal2 = false, stall2 = false;
  let isPaused = false;
  let isCalibrating = false;
  let isOnline = false;

  // 8. 시스템 터미널 로그
  let logs = [];
  let autoScroll = true;
  let terminalEl;
  let manualCmd = '';

  function addLog(level, msg) {
    const now = new Date();
    const time = now.toTimeString().split(' ')[0] + '.' + String(now.getMilliseconds()).padStart(3, '0');
    logs = [...logs, { time, level: (level || 'info').toLowerCase(), msg }];
    if (autoScroll && terminalEl) {
      tick().then(() => {
        if (terminalEl) terminalEl.scrollTop = terminalEl.scrollHeight;
      });
    }
  }

  function clearLogs() {
    logs = [];
    addLog('info', '터미널 로그가 초기화되었습니다.');
  }

  function computeExpAlpha(demand, maxStep, k) {
    if (demand === 0) return 0;
    const sign = demand > 0 ? 1 : -1;
    const absD = Math.min(Math.abs(demand), 180.0);
    const norm = absD / 180.0;
    const safeK = Math.max(k, 0.05);
    const factor = (Math.exp(safeK * norm) - 1.0) / (Math.exp(safeK) - 1.0);
    const step = Math.max(1, Math.round(maxStep * factor));
    return sign * step;
  }

  $: safeAlphaPreview = computeExpAlpha(stallSafeDemand, maxStepAlpha, expCurveK);

  function connectWs() {
    const host = window.location.hostname || '192.168.4.1';
    ws = new WebSocket(`ws://${host}/ws`);

    ws.onopen = () => {
      isConnected = true;
      addLog('info', `ESP32 웹소켓 서버(ws://${host}/ws) 연결됨`);
      sendCmd('get_config');
    };

    ws.onclose = () => {
      isConnected = false;
      addLog('err', '웹소켓 연결 끊김. 1.5초 후 재연결 시도...');
      setTimeout(connectWs, 1500);
    };

    ws.onmessage = (evt) => {
      try {
        const data = JSON.parse(evt.data);
        if (data.type === 'log') {
          addLog(data.level, data.msg);
          return;
        }
        if (data.type === 'config') {
          if (data.ema_alpha !== undefined) emaAlpha = Number(data.ema_alpha.toFixed(2));
          if (data.max_step !== undefined)  maxStepAlpha = Number(data.max_step.toFixed(1));
          if (data.exp_k !== undefined)     expCurveK = Number(data.exp_k.toFixed(2));
          if (data.gamma !== undefined)     couplingGamma = Number(data.gamma.toFixed(2));
          if (data.stall_safe !== undefined) stallSafeDemand = data.stall_safe;
          if (data.stall_cut !== undefined)  stallCutoffDemand = data.stall_cut;
          if (data.stall_ms !== undefined)   stallProtectSec = Number((data.stall_ms / 1000).toFixed(1));
          if (data.ext1 !== undefined) ext1 = data.ext1;
          if (data.flx1 !== undefined) flx1 = data.flx1;
          if (data.cal1 !== undefined) cal1 = data.cal1;
          if (data.ext2 !== undefined) ext2 = data.ext2;
          if (data.flx2 !== undefined) flx2 = data.flx2;
          if (data.cal2 !== undefined) cal2 = data.cal2;
          return;
        }

        // 실시간 텔레메트리
        p1 = data.p ?? 0;
        des1 = data.des ?? 0;
        a1 = data.a ?? 0;
        dem1 = data.dem ?? 0;
        step1 = data.step ?? 0;
        target1 = data.target ?? 0;
        if (data.ext !== undefined) ext1 = data.ext;
        if (data.flx !== undefined) flx1 = data.flx;
        if (data.cal !== undefined) cal1 = data.cal;
        stall1 = !!data.stall;

        p2 = data.p2 ?? 0;
        des2 = data.des2 ?? 0;
        a2 = data.a2 ?? 0;
        dem2 = data.dem2 ?? 0;
        step2 = data.step2 ?? 0;
        pure2 = data.pure2 ?? 0;
        target2 = data.target2 ?? 0;
        if (data.ext2 !== undefined) ext2 = data.ext2;
        if (data.flx2 !== undefined) flx2 = data.flx2;
        if (data.cal2 !== undefined) cal2 = data.cal2;
        stall2 = !!data.stall2;

        isPaused = !!data.paused;
        isCalibrating = !!data.calibrating;
        isOnline = !!data.online;
      } catch (e) {}
    };
  }

  function sendCmd(cmd, payload = {}) {
    const packet = JSON.stringify({ cmd, ...payload });
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(packet);
    } else {
      addLog('warn', `[미연결 상태 명령] ${packet}`);
    }
  }

  function handleStallChange() {
    if (stallCutoffDemand < stallSafeDemand) {
      stallCutoffDemand = stallSafeDemand;
    }
    sendCmd('set_stall_params', {
      safe: Number(stallSafeDemand),
      cutoff: Number(stallCutoffDemand),
      ms: Math.round(Number(stallProtectSec) * 1000)
    });
  }

  function sendManualCommand() {
    const raw = manualCmd.trim();
    if (!raw) return;
    manualCmd = '';
    if (raw.startsWith('{')) {
      try {
        const parsed = JSON.parse(raw);
        sendCmd(parsed.cmd, parsed);
      } catch (e) {
        addLog('err', 'JSON 파싱 오류: ' + e.message);
      }
    } else {
      sendCmd(raw);
    }
  }

  onMount(() => {
    addLog('info', 'Svelte 통합 제어 대시보드 초기화 완료');
    connectWs();
  });
</script>

<main class="container">
  <header>
    <div>
      <h1>🦾 여섯 번째 손가락 통합 제어 대시보드</h1>
      <p class="subtitle">듀얼 모터(1·2번 마디 / 3번 마디) 지수함수 토크 제어 & 전향 간섭 보상</p>
    </div>
    <div class="header-right">
      <div class="status-badge">
        <div class="status-dot" class:online={isConnected}></div>
        <span>{isConnected ? 'ESP32 연결됨' : '연결 대기중'}</span>
      </div>
      <button class="btn" class:btn-accent={isPaused} class:btn-warn={!isPaused} on:click={() => sendCmd('toggle_pause')}>
        {isPaused ? '🔓 잠금 해제' : '⏸️ 일시정지/잠금'}
      </button>
    </div>
  </header>

  <!-- 상단 상태 요약 바 -->
  <section class="quick-grid">
    <div class="pill">
      <span>무선 수신</span>
      <strong class:ok={isOnline} class:danger={!isOnline}>{isOnline ? '수신 양호(50Hz)' : '타임아웃'}</strong>
    </div>
    <div class="pill">
      <span>일시정지 상태</span>
      <strong class:danger={isPaused} class:ok={!isPaused}>{isPaused ? '🔒 잠금됨' : '🟢 정상 구동'}</strong>
    </div>
    <div class="pill">
      <span>캘리브레이션</span>
      <strong class:ok={cal1 && cal2} class:warn={!cal1 || !cal2}>
        {isCalibrating ? '🔄 보정 진행중' : (cal1 && cal2 ? '✅ 보정 완료' : '📐 기본 LUT')}
      </strong>
    </div>
    <div class="pill">
      <span>과부하(Stall)</span>
      <strong class:danger={stall1 || stall2} class:ok={!stall1 && !stall2}>
        {stall1 || stall2 ? `⚠️ 제한 작동(${stallSafeDemand}°)` : '정상'}
      </strong>
    </div>
  </section>

  <!-- 탭 네비게이션 -->
  <nav class="tab-nav">
    <button class="tab-btn" class:active={activeTab === 'monitor'} on:click={() => activeTab = 'monitor'}>
      📊 실시간 모니터링
    </button>
    <button class="tab-btn" class:active={activeTab === 'params'} on:click={() => activeTab = 'params'}>
      ⚙️ 파라미터 & 캘리브레이션
    </button>
    <button class="tab-btn" class:active={activeTab === 'terminal'} on:click={() => activeTab = 'terminal'}>
      💻 시스템 터미널 ({logs.length})
    </button>
  </nav>

  <!-- [탭 1] 실시간 모니터링 -->
  {#if activeTab === 'monitor'}
    <div class="motor-grid">
      <!-- 1번 모터 -->
      <section class="card">
        <div class="card-head">
          <h2>1번 모터 (바깥쪽 1·2번 마디)</h2>
          <span class="tag">{cal1 ? '✅ 플렉스 보정됨' : '📐 arccos 기본값'}</span>
        </div>
        <div class="metric-grid">
          <div class="metric-box">
            <div class="label">압력 목표 각도 (Desired)</div>
            <div class="val">{des1}°</div>
            <div class="sub">FSR 1 압력: {p1} / 4095</div>
          </div>
          <div class="metric-box">
            <div class="label">플렉스 현재 각도 (Current)</div>
            <div class="val green">{a1}°</div>
            <div class="sub">요구치(Demand): {dem1 > 0 ? `+${dem1}` : dem1}°</div>
          </div>
          <div class="metric-box highlight-span">
            <div>
              <div class="label">서보 각도 변화 목표 (α₁)</div>
              <div class="val warn">{step1 > 0 ? `+${step1}` : step1}°</div>
              <div class="sub">지수함수 실제 변화량</div>
            </div>
            <div>
              <div class="label">최종 서보 지령 각도</div>
              <div class="val">{target1}°</div>
              <div class="sub">현재각 + α₁</div>
            </div>
          </div>
        </div>
        <div class="range-info">
          <span>현재 가동범위(한계각): <strong>{ext1}° ~ {flx1}°</strong> ({Math.max(2, flx1 - ext1 + 1)}단계)</span>
          <span class:danger={stall1}>{stall1 ? `⚠️ 과부하 제한(${stallSafeDemand}°)` : '정상'}</span>
        </div>
        <div class="bar-bg">
          <div class="bar-fill green" style="width: {(a1 / 180) * 100}%"></div>
        </div>
      </section>

      <!-- 2번 모터 -->
      <section class="card">
        <div class="card-head">
          <h2>2번 모터 (손날 쪽 3번 마디)</h2>
          <span class="tag">{cal2 ? '✅ 플렉스 보정됨' : '📐 arccos 기본값'}</span>
        </div>
        <div class="metric-grid">
          <div class="metric-box">
            <div class="label">압력 목표 각도 (Desired)</div>
            <div class="val">{des2}°</div>
            <div class="sub">FSR 2 압력: {p2} / 4095</div>
          </div>
          <div class="metric-box">
            <div class="label">플렉스 현재 각도 (Current)</div>
            <div class="val green">{a2}°</div>
            <div class="sub">요구치(Demand): {dem2 > 0 ? `+${dem2}` : dem2}°</div>
          </div>
          <div class="metric-box highlight-span">
            <div>
              <div class="label">서보 각도 변화 목표 (α₂)</div>
              <div class="val warn">{step2 > 0 ? `+${step2}` : step2}°</div>
              <div class="sub">순수({pure2 > 0 ? `+${pure2}` : pure2}°) - 간섭보상(γ·α₁)</div>
            </div>
            <div>
              <div class="label">최종 서보 지령 각도</div>
              <div class="val">{target2}°</div>
              <div class="sub">Anchor 기준 + α₂</div>
            </div>
          </div>
        </div>
        <div class="range-info">
          <span>현재 가동범위(한계각): <strong>{ext2}° ~ {flx2}°</strong> ({Math.max(2, flx2 - ext2 + 1)}단계)</span>
          <span class:danger={stall2}>{stall2 ? `⚠️ 과부하 제한(${stallSafeDemand}°)` : '정상'}</span>
        </div>
        <div class="bar-bg">
          <div class="bar-fill green" style="width: {(a2 / 180) * 100}%"></div>
        </div>
      </section>
    </div>
  {/if}

  <!-- [탭 2] 파라미터 & 캘리브레이션 -->
  {#if activeTab === 'params'}
    <section class="card">
      <div class="card-head">
        <h2>제어 파라미터 튜닝 (요구치 {stallSafeDemand}° 시 변화량 +{safeAlphaPreview}° 출력)</h2>
        <span class="tag">조절 즉시 반영</span>
      </div>

      <div class="control-group">
        <div class="control-header">
          <span>1. 발 압력 센서 EMA 필터 계수 (ema_alpha)</span>
          <strong class="primary">{emaAlpha}</strong>
        </div>
        <input type="range" min="0.05" max="0.95" step="0.05" bind:value={emaAlpha} on:change={() => sendCmd('set_alpha', { val: emaAlpha })} />
      </div>

      <div class="control-group">
        <div class="control-header">
          <span>2. 지수함수 최대 변화량 제한 (max_step_alpha)</span>
          <strong class="primary">{maxStepAlpha}°</strong>
        </div>
        <input type="range" min="5" max="70" step="1" bind:value={maxStepAlpha} on:change={() => sendCmd('set_max_step', { val: maxStepAlpha })} />
      </div>

      <div class="control-group">
        <div class="control-header">
          <span>3. 지수함수 곡률 상수 (exp_curve_k)</span>
          <strong class="primary">{expCurveK}</strong>
        </div>
        <input type="range" min="0.1" max="6.0" step="0.1" bind:value={expCurveK} on:change={() => sendCmd('set_exp_k', { val: expCurveK })} />
      </div>

      <div class="control-group">
        <div class="control-header">
          <span>4. 3번 마디 전향 교차 결합 보상 계수 (coupling_gamma, γ)</span>
          <strong class="primary">{couplingGamma}</strong>
        </div>
        <input type="range" min="0.00" max="0.60" step="0.02" bind:value={couplingGamma} on:change={() => sendCmd('set_gamma', { val: couplingGamma })} />
      </div>

      <div class="control-group">
        <div class="control-header">
          <span>5-A. 과부하 제한 안전 요구치 각도 (기본 140°)</span>
          <strong class="warn">{stallSafeDemand}°</strong>
        </div>
        <input type="range" min="50" max="170" step="5" bind:value={stallSafeDemand} on:change={handleStallChange} />

        <div class="control-header" style="margin-top:8px;">
          <span>5-B. 과부하 감지 커트라인 요구치 각도 (기본 150°)</span>
          <strong class="warn">{stallCutoffDemand}°</strong>
        </div>
        <input type="range" min="60" max="180" step="5" bind:value={stallCutoffDemand} on:change={handleStallChange} />

        <div class="control-header" style="margin-top:8px;">
          <span>5-C. 과부하 제한 발동 지속 시간 (기본 5.0초)</span>
          <strong class="warn">{stallProtectSec}초</strong>
        </div>
        <input type="range" min="1.0" max="15.0" step="0.5" bind:value={stallProtectSec} on:change={handleStallChange} />
      </div>

      <div class="btn-grid">
        <button class="btn btn-accent" on:click={() => sendCmd('start_calib')}>
          🔄 플렉스 센서 순차 보정 시작 (2번→1번)
        </button>
        <button class="btn btn-warn" on:click={() => sendCmd('reset_lut')}>
          📐 arccos 기본 LUT 복원
        </button>
        <button class="btn btn-primary btn-full" on:click={() => sendCmd('save_nvs')}>
          💾 모든 파라미터 및 LUT 비휘발성 메모리(NVS) 영구 저장
        </button>
      </div>
    </section>
  {/if}

  <!-- [탭 3] 시스템 터미널 -->
  {#if activeTab === 'terminal'}
    <section class="card">
      <div class="card-head">
        <h2>실시간 시스템 메시지 & 오류 출력 터미널</h2>
        <button class="btn" on:click={clearLogs}>로그 지우기</button>
      </div>
      <div class="terminal-box" bind:this={terminalEl}>
        {#each logs as item}
          <div class="log-line">
            <span class="time">[{item.time}]</span>
            <span class="lvl {item.level}">[{item.level.toUpperCase()}]</span>
            <span>{item.msg}</span>
          </div>
        {/each}
      </div>
      <div class="cmd-row">
        <input class="cmd-input" bind:value={manualCmd} placeholder="명령어 입력 (start_calib, save_nvs, toggle_pause 등)" on:keydown={(e) => e.key === 'Enter' && sendManualCommand()} />
        <button class="btn btn-primary" on:click={sendManualCommand}>전송</button>
      </div>
    </section>
  {/if}
</main>

<style>
  :global(body) { background: #0b1120; color: #f8fafc; margin: 0; padding: 14px; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
  .container { max-width: 820px; margin: 0 auto; display: flex; flex-direction: column; gap: 14px; }
  header { background: #1e293b; border: 1px solid #334155; border-radius: 12px; padding: 14px 16px; display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 10px; }
  h1 { font-size: 1.15rem; color: #38bdf8; margin: 0; }
  .subtitle { font-size: 0.76rem; color: #94a3b8; margin: 2px 0 0 0; }
  .header-right { display: flex; align-items: center; gap: 8px; }
  .status-badge { display: flex; align-items: center; gap: 6px; font-size: 0.8rem; padding: 5px 10px; border-radius: 999px; background: #0f172a; border: 1px solid #334155; }
  .status-dot { width: 8px; height: 8px; border-radius: 50%; background: #ef4444; }
  .status-dot.online { background: #10b981; box-shadow: 0 0 8px #10b981; }

  .quick-grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 8px; }
  @media (max-width: 600px) { .quick-grid { grid-template-columns: repeat(2, 1fr); } }
  .pill { background: #1e293b; border: 1px solid #334155; border-radius: 10px; padding: 10px 12px; display: flex; justify-content: space-between; font-size: 0.8rem; }
  .ok { color: #10b981; }
  .warn { color: #f59e0b; }
  .danger { color: #ef4444; }
  .primary { color: #38bdf8; }

  .tab-nav { display: flex; background: #0f172a; border: 1px solid #334155; border-radius: 10px; padding: 4px; gap: 4px; }
  .tab-btn { flex: 1; padding: 10px; border: none; background: transparent; color: #94a3b8; font-weight: 700; border-radius: 8px; cursor: pointer; }
  .tab-btn.active { background: #38bdf8; color: #0b1120; }

  .motor-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; }
  @media (max-width: 680px) { .motor-grid { grid-template-columns: 1fr; } }
  .card { background: #1e293b; border: 1px solid #334155; border-radius: 12px; padding: 16px; }
  .card-head { display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px; padding-bottom: 8px; border-bottom: 1px solid #334155; }
  .card-head h2 { font-size: 0.95rem; margin: 0; }
  .tag { font-size: 0.72rem; background: #0f172a; padding: 3px 8px; border-radius: 6px; border: 1px solid #334155; }

  .metric-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-bottom: 10px; }
  .metric-box { background: #0f172a; border-radius: 8px; padding: 10px; text-align: center; }
  .highlight-span { grid-column: span 2; display: flex; justify-content: space-around; background: rgba(56, 189, 248, 0.08); border: 1px solid rgba(56, 189, 248, 0.3); }
  .label { font-size: 0.72rem; color: #94a3b8; }
  .val { font-size: 1.4rem; font-weight: 800; color: #38bdf8; margin: 3px 0; }
  .val.green { color: #10b981; }
  .val.warn { color: #f59e0b; }
  .sub { font-size: 0.7rem; color: #94a3b8; }

  .range-info { display: flex; justify-content: space-between; font-size: 0.74rem; color: #94a3b8; margin-bottom: 4px; }
  .bar-bg { width: 100%; height: 9px; background: #0f172a; border-radius: 5px; overflow: hidden; }
  .bar-fill.green { height: 100%; background: #10b981; transition: width 0.08s ease; }

  .control-group { background: #0f172a; border: 1px solid #334155; border-radius: 8px; padding: 12px; margin-bottom: 10px; }
  .control-header { display: flex; justify-content: space-between; font-size: 0.84rem; margin-bottom: 6px; }
  input[type=range] { width: 100%; accent-color: #38bdf8; cursor: pointer; }

  .btn-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-top: 12px; }
  .btn { padding: 10px 14px; border: none; border-radius: 8px; font-size: 0.84rem; font-weight: 700; cursor: pointer; background: #334155; color: #f8fafc; }
  .btn-primary { background: #38bdf8; color: #0b1120; }
  .btn-accent { background: #10b981; color: #0b1120; }
  .btn-warn { background: #f59e0b; color: #0b1120; }
  .btn-full { grid-column: span 2; }

  .terminal-box { background: #050811; border: 1px solid #334155; border-radius: 8px; height: 320px; overflow-y: auto; padding: 10px; font-family: monospace; font-size: 0.8rem; }
  .log-line { margin-bottom: 4px; }
  .time { color: #64748b; margin-right: 6px; }
  .lvl.info { color: #38bdf8; font-weight: bold; margin-right: 6px; }
  .lvl.warn { color: #f59e0b; font-weight: bold; margin-right: 6px; }
  .lvl.err { color: #ef4444; font-weight: bold; margin-right: 6px; }
  .lvl.calib { color: #10b981; font-weight: bold; margin-right: 6px; }
  .cmd-row { display: flex; gap: 8px; margin-top: 10px; }
  .cmd-input { flex: 1; background: #0f172a; border: 1px solid #334155; color: #f8fafc; padding: 8px 12px; border-radius: 8px; font-family: monospace; }
</style>
