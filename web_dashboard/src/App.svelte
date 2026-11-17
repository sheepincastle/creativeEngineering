<script>
  import { onMount, tick } from 'svelte';

  let ws = null;
  let isConnected = false;
  let isSimMode = false;
  let activeTab = 'monitor'; // 'monitor' | 'params' | 'terminal'

  // 활성 모터 채널 수 (펌웨어 NUM_MOTORS와 동기화: 1 또는 2)
  let numMotors = 2;

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

  // PC 가상 압력 & 외란 테스트 시뮬레이터 상태
  let simP1 = 0;
  let simP2 = 0;
  let simBlock1 = false;
  let simBlock2 = false;
  let lastSimSendAt = 0;
  let simTrailingTimer = null;

  // 8. 시스템 터미널 로그 & 캔버스 참조
  let logs = [];
  let lastLogText = '시스템 초기화 완료. 대기 중...';
  let autoScroll = true;
  let terminalEl;
  let manualCmd = '';
  let expCurveCanvas;

  function addLog(level, msg) {
    const now = new Date();
    const time = now.toTimeString().split(' ')[0] + '.' + String(now.getMilliseconds()).padStart(3, '0');
    const lvl = (level || 'info').toLowerCase();
    logs = [...logs, { time, level: lvl, msg }];
    lastLogText = `[${time}] ${msg}`;
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

  // 지수함수 변화량 계산 (펌웨어 computeExponentialAlpha와 동일 수식)
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
  $: isSingleMotor = numMotors === 1;
  $: hasStall = isSingleMotor ? stall1 : (stall1 || stall2);

  // 지수함수 곡선 캔버스 렌더링
  function drawExpCurve(canvas, maxStep, k, safeDemand) {
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const w = canvas.width;
    const h = canvas.height;
    ctx.clearRect(0, 0, w, h);

    const padL = 40, padR = 18, padT = 16, padB = 26;
    const pw = w - padL - padR;
    const ph = h - padT - padB;

    // 격자선 및 X축 눈금
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i++) {
      const y = padT + (ph * i) / 4;
      ctx.beginPath();
      ctx.moveTo(padL, y);
      ctx.lineTo(padL + pw, y);
      ctx.stroke();
    }
    for (let deg = 0; deg <= 180; deg += 30) {
      const x = padL + (deg / 180) * pw;
      ctx.beginPath();
      ctx.moveTo(x, padT);
      ctx.lineTo(x, padT + ph);
      ctx.stroke();
      ctx.fillStyle = '#64748b';
      ctx.font = '10px sans-serif';
      ctx.fillText(deg + '°', x - 8, h - 8);
    }

    const maxY = Math.max(40, maxStep);
    ctx.fillStyle = '#64748b';
    ctx.font = '10px sans-serif';
    ctx.fillText(Math.round(maxY) + '°', 6, padT + 8);
    ctx.fillText('0°', 16, padT + ph);

    // 안전 제한 요구치(stall_safe) 수직 점선 표시
    const safeX = padL + (safeDemand / 180.0) * pw;
    ctx.strokeStyle = 'rgba(245, 158, 11, 0.5)';
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(safeX, padT);
    ctx.lineTo(safeX, padT + ph);
    ctx.stroke();
    ctx.setLineDash([]);

    // 지수 곡선 그리기
    ctx.strokeStyle = '#38bdf8';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    for (let d = 0; d <= 180; d++) {
      const a = computeExpAlpha(d, maxStep, k);
      const x = padL + (d / 180.0) * pw;
      const y = padT + ph - (a / maxY) * ph;
      if (d === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();

    // 안전 요구치 지점 포인트
    const safeAlpha = computeExpAlpha(safeDemand, maxStep, k);
    const safeY = padT + ph - (safeAlpha / maxY) * ph;
    ctx.fillStyle = '#10b981';
    ctx.beginPath();
    ctx.arc(safeX, safeY, 4.5, 0, Math.PI * 2);
    ctx.fill();
  }

  $: if (activeTab === 'params' && expCurveCanvas) {
    drawExpCurve(expCurveCanvas, maxStepAlpha, expCurveK, stallSafeDemand);
  }

  function isWsOpen() {
    return !!(ws && ws.readyState === WebSocket.OPEN);
  }

  function sendSimPacket(enable) {
    lastSimSendAt = Date.now();
    const packet = {
      cmd: 'set_sim',
      enable,
      p1: enable ? (Number(simP1) || 0) : 0,
      p2: enable ? (Number(simP2) || 0) : 0,
      b1: enable ? !!simBlock1 : false,
      b2: enable ? !!simBlock2 : false
    };
    if (isWsOpen()) {
      ws.send(JSON.stringify(packet));
    }
  }

  function scheduleSimSync() {
    if (!isWsOpen()) return;
    const now = Date.now();
    const elapsed = now - lastSimSendAt;
    if (elapsed >= 40 && !simTrailingTimer) {
      sendSimPacket(true);
      return;
    }
    if (!simTrailingTimer) {
      simTrailingTimer = setTimeout(() => {
        simTrailingTimer = null;
        if (isSimMode && isWsOpen()) {
          sendSimPacket(true);
        }
      }, Math.max(10, 40 - elapsed));
    }
  }

  // PC 오프라인 가상 시뮬레이션용 역코사인 각도 변환
  function calcArccosAngle(p, ext, flx) {
    const u = Math.min(Math.max(p / 4095.0, 0), 1);
    const acosIn = Math.min(Math.max(1.0 - 2.0 * u, -1), 1);
    const norm = Math.acos(acosIn) / Math.PI;
    return Math.round(ext + norm * (flx - ext));
  }

  function toggleSimMode(enabled) {
    isSimMode = enabled;
    if (isWsOpen()) {
      if (simTrailingTimer) {
        clearTimeout(simTrailingTimer);
        simTrailingTimer = null;
      }
      if (enabled) {
        addLog('warn', '웹 가상 압력 연동(HIL) 모드가 활성화되었습니다. 슬라이더 조작 시 실제 서보모터가 구동됩니다.');
        sendSimPacket(true);
      } else {
        addLog('info', '웹 가상 압력 모드를 해제하고 발 송신부 무선 입력으로 복귀합니다.');
        sendSimPacket(false);
      }
    } else if (enabled) {
      addLog('warn', 'PC 오프라인 가상 압력 시뮬레이션 모드가 활성화되었습니다.');
      runOfflineSimStep();
    }
  }

  function onSimInput() {
    if (!isSimMode) {
      toggleSimMode(true);
      return;
    }
    if (isWsOpen()) {
      scheduleSimSync();
    } else {
      runOfflineSimStep();
    }
  }

  function runOfflineSimStep() {
    p1 = Number(simP1) || 0;
    des1 = calcArccosAngle(p1, ext1, flx1);
    if (!simBlock1) {
      a1 = Math.round(a1 * 0.65 + des1 * 0.35);
    }
    dem1 = des1 - a1;
    const span1 = Math.max(18, flx1 - ext1);
    const effDem1 = Math.max(-180, Math.min(180, Math.round((dem1 * 180.0) / span1)));
    const alpha1 = computeExpAlpha(effDem1, maxStepAlpha, expCurveK);
    step1 = alpha1;
    target1 = Math.min(Math.max(a1 + alpha1, ext1), flx1);

    if (!isSingleMotor) {
      p2 = Number(simP2) || 0;
      des2 = calcArccosAngle(p2, ext2, flx2);
      if (!simBlock2) {
        a2 = Math.round(a2 * 0.65 + des2 * 0.35);
      }
      dem2 = des2 - a2;
      const span2 = Math.max(18, flx2 - ext2);
      const effDem2 = Math.max(-180, Math.min(180, Math.round((dem2 * 180.0) / span2)));
      const p2Alpha = computeExpAlpha(effDem2, maxStepAlpha, expCurveK);
      const comp = Math.round(couplingGamma * alpha1);
      const alpha2 = p2Alpha - comp;
      pure2 = p2Alpha;
      step2 = alpha2;
      target2 = Math.min(Math.max(a2 + alpha2, ext2), flx2);
    }
  }

  function connectWs() {
    if (window.location.protocol === 'file:' || window.location.hostname === '') {
      toggleSimMode(true);
      return;
    }

    const host = window.location.hostname || '192.168.4.1';
    ws = new WebSocket(`ws://${host}/ws`);

    ws.onopen = () => {
      isConnected = true;
      isSimMode = false;
      addLog('info', `ESP32 웹소켓 서버(ws://${host}/ws)에 연결되었습니다.`);
      sendCmd('get_config');
    };

    ws.onclose = () => {
      if (simTrailingTimer) {
        clearTimeout(simTrailingTimer);
        simTrailingTimer = null;
      }
      const wasConnected = isConnected;
      isConnected = false;
      if (wasConnected) {
        isSimMode = false;
      }
      if (wasConnected || !isSimMode) {
        addLog('err', '웹소켓 연결이 끊어졌습니다. 1.5초 후 재연결을 시도합니다...');
      }
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
          if (data.motors !== undefined)     numMotors = data.motors;
          if (data.sim !== undefined)        isSimMode = !!data.sim;
          if (data.ema_alpha !== undefined)  emaAlpha = Number(data.ema_alpha.toFixed(2));
          if (data.max_step !== undefined)   maxStepAlpha = Number(data.max_step.toFixed(1));
          if (data.exp_k !== undefined)      expCurveK = Number(data.exp_k.toFixed(2));
          if (data.gamma !== undefined)      couplingGamma = Number(data.gamma.toFixed(2));
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

        // 실시간 텔레메트리 수신 (가상 압력 HIL 연동 중에도 실제 서보/플렉스 상태 갱신)
        if (data.motors !== undefined) numMotors = data.motors;
        if (data.sim !== undefined && Date.now() - lastSimSendAt > 400) {
          isSimMode = !!data.sim;
        }
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
      addLog('warn', `[시뮬레이션 명령 실행] ${packet}`);
      if (cmd === 'toggle_pause') {
        isPaused = !isPaused;
        addLog(isPaused ? 'warn' : 'info', isPaused ? '[LOCK] 시뮬레이션 일시정지 활성화' : '[UNLOCK] 시뮬레이션 구동 재개');
      } else if (cmd === 'save_nvs') {
        addLog('info', '[NVS] 시뮬레이션: 파라미터 및 LUT 저장 명령 확인됨');
      } else if (cmd === 'reset_lut') {
        // 펌웨어 동작과 일치: 기존 탐색된 한계각(ext, flx)은 유지하고 보정 완료 플래그만 해제
        cal1 = false;
        cal2 = false;
        addLog('warn', '[LUT] 시뮬레이션: 기존 한계각을 유지한 채 arccos 기본 LUT로 복원됨');
      }
    }
  }

  function triggerCalibration() {
    const msg = isSingleMotor
      ? '플렉스 센서 캘리브레이션(1번 모터) 요청을 전송했습니다.'
      : '플렉스 센서 순차 캘리브레이션(2번 모터 -> 1번 모터) 요청을 전송했습니다.';
    addLog('calib', msg);
    sendCmd('start_calib');
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
    addLog('info', '여섯 번째 손가락 통합 대시보드가 시작되었습니다.');
    connectWs();
  });
</script>

<main class="container">
  <!-- 상단 헤더 -->
  <header>
    <div>
      <h1>🦾 여섯 번째 손가락 통합 제어 시스템</h1>
      <p class="subtitle">듀얼 서보(1·2번 마디 / 3번 마디) 지수함수 토크 제어 & 전향 교차 결합 보상</p>
    </div>
    <div class="header-right">
      <div class="status-badge">
        <div class="status-dot" class:online={isConnected} class:sim={isSimMode && !isConnected}></div>
        <span>{isConnected ? 'ESP32 실시간 연결됨' : (isSimMode ? 'PC 시뮬레이션 모드' : '연결 대기중')}</span>
      </div>
      <button class="btn" class:btn-accent={isPaused} class:btn-warn={!isPaused} on:click={() => sendCmd('toggle_pause')}>
        {isPaused ? '🔓 잠금 해제 (구동 재개)' : '⏸️ 일시정지 / 잠금 토글'}
      </button>
    </div>
  </header>

  <!-- 상단 핵심 상태 요약 바 (모든 탭 공통 표시) -->
  <section class="quick-grid">
    <div class="pill">
      <span>발 송신부 무선링크</span>
      {#if isSimMode && isConnected}
        <strong class="warn">🎛️ 웹 가상압력 구동중</strong>
      {:else if isSimMode}
        <strong class="warn">💻 PC 오프라인 시뮬</strong>
      {:else}
        <strong class:ok={isOnline} class:danger={!isOnline}>{isOnline ? '수신 양호 (50Hz)' : '타임아웃 (Fail-Safe)'}</strong>
      {/if}
    </div>
    <div class="pill">
      <span>동작/잠금 상태</span>
      <strong class:danger={isPaused} class:ok={!isPaused}>{isPaused ? '🔒 잠금 (토크 해제)' : '🟢 정상 구동중'}</strong>
    </div>
    <div class="pill">
      <span>플렉스 보정(LUT)</span>
      {#if isCalibrating}
        <strong class="primary">🔄 스윕 보정중...</strong>
      {:else if isSingleMotor}
        <strong class:ok={cal1} class:warn={!cal1}>{cal1 ? '✅ 보정 완료 (1모터)' : '📐 기본 arccos LUT'}</strong>
      {:else if cal1 && cal2}
        <strong class="ok">✅ 양채널 보정됨</strong>
      {:else if cal1 || cal2}
        <strong class="warn">⚠️ 일부 보정됨</strong>
      {:else}
        <strong class="warn">📐 기본 arccos LUT</strong>
      {/if}
    </div>
    <div class="pill">
      <span>과부하(Stall) 보호</span>
      <strong class:danger={hasStall} class:ok={!hasStall}>
        {hasStall ? `⚠️ 제한 작동(${stallSafeDemand}°)` : '정상 (해제)'}
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
      💻 시스템 터미널 <span class="tab-badge">{logs.length}</span>
    </button>
  </nav>

  <!-- [탭 1] 실시간 모니터링 -->
  {#if activeTab === 'monitor'}
    <div class="motor-grid">
      <!-- 1번 모터 -->
      <section class="card">
        <div class="card-head">
          <h2>1번 모터 (바깥쪽 1·2번 마디)</h2>
          <span class="tag">{cal1 ? '✅ 플렉스 선형 보정됨' : '📐 arccos 기본 LUT'}</span>
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
              <div class="sub">지수함수 산출 변화량</div>
            </div>
            <div class="v-sep"></div>
            <div>
              <div class="label">최종 서보 지령 각도</div>
              <div class="val">{target1}°</div>
              <div class="sub">현재각 + α₁ (한계 클램핑)</div>
            </div>
          </div>
        </div>
        <div class="range-info">
          <span>서보 가동범위 (한계각): <strong class="ok">{ext1}° ~ {flx1}°</strong> ({Math.max(2, flx1 - ext1 + 1)}단계)</span>
          <span class:danger={stall1}>{stall1 ? `⚠️ 과부하 제한 발동 (${stallSafeDemand}°)` : '과부하 없음'}</span>
        </div>
        <div class="bar-bg">
          <div class="range-window" style="left: {(ext1 / 180) * 100}%; width: {Math.max(0, (flx1 - ext1) / 180) * 100}%"></div>
          <div class="bar-fill green" style="width: {(a1 / 180) * 100}%"></div>
          <div class="marker-dot" style="left: {(des1 / 180) * 100}%" title="압력 목표 각도"></div>
        </div>
      </section>

      <!-- 2번 모터 -->
      <section class="card" class:dimmed={isSingleMotor}>
        <div class="card-head">
          <h2>2번 모터 (손날 쪽 3번 마디)</h2>
          <span class="tag">{cal2 ? '✅ 플렉스 선형 보정됨' : '📐 arccos 기본 LUT'}</span>
        </div>
        {#if isSingleMotor}
          <div class="single-notice">
            ℹ️ 단일 모터 롤백 모드(NUM_MOTORS = 1)로 동작 중이므로 2번 모터 채널은 비활성화되어 있습니다.
          </div>
        {/if}
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
              <div class="val purple">{step2 > 0 ? `+${step2}` : step2}°</div>
              <div class="sub">순수 {pure2 > 0 ? `+${pure2}` : pure2}° - 간섭보상(γ·α₁)</div>
            </div>
            <div class="v-sep"></div>
            <div>
              <div class="label">최종 서보 지령 각도</div>
              <div class="val">{target2}°</div>
              <div class="sub">Anchor 기준 + α₂</div>
            </div>
          </div>
        </div>
        <div class="range-info">
          <span>서보 가동범위 (한계각): <strong class="ok">{ext2}° ~ {flx2}°</strong> ({Math.max(2, flx2 - ext2 + 1)}단계)</span>
          <span class:danger={stall2}>{stall2 ? `⚠️ 과부하 제한 발동 (${stallSafeDemand}°)` : '과부하 없음'}</span>
        </div>
        <div class="bar-bg">
          <div class="range-window" style="left: {(ext2 / 180) * 100}%; width: {Math.max(0, (flx2 - ext2) / 180) * 100}%"></div>
          <div class="bar-fill green" style="width: {(a2 / 180) * 100}%"></div>
          <div class="marker-dot" style="left: {(des2 / 180) * 100}%" title="압력 목표 각도"></div>
        </div>
      </section>
    </div>

    <!-- 가상 발 압력 & 외란 테스트 시뮬레이터 패널 -->
    <div class="sim-panel">
      <div class="sim-head">
        <span class="sim-title">🎮 가상 발 압력 & 외란 테스트 시뮬레이터 (ESP32 연결 시 실제 서보 구동 / 오프라인 시 PC 검증용)</span>
        <label class="sim-chk">
          <input type="checkbox" checked={isSimMode} on:change={(e) => toggleSimMode(e.target.checked)} />
          시뮬레이션 수동 조작 활성화
        </label>
      </div>
      <div class="motor-grid" style="gap:10px;">
        <div>
          <div class="range-info">
            <span>1번 FSR 압력 (1·2번 마디): <strong>{simP1}</strong></span>
            <label>물체 접촉(현재각 고정): <input type="checkbox" bind:checked={simBlock1} on:change={onSimInput} /></label>
          </div>
          <input type="range" min="0" max="4095" step="15" bind:value={simP1} on:input={onSimInput} />
        </div>
        <div class:dimmed={isSingleMotor}>
          <div class="range-info">
            <span>2번 FSR 압력 (3번 마디): <strong>{simP2}</strong></span>
            <label>물체 접촉(현재각 고정): <input type="checkbox" bind:checked={simBlock2} on:change={onSimInput} disabled={isSingleMotor} /></label>
          </div>
          <input type="range" min="0" max="4095" step="15" bind:value={simP2} on:input={onSimInput} disabled={isSingleMotor} />
        </div>
      </div>
    </div>
  {/if}

  <!-- [탭 2] 파라미터 & 캘리브레이션 -->
  {#if activeTab === 'params'}
    <section class="card">
      <div class="card-head">
        <h2>지수함수 토크 제어 & 전향 간섭 보상 파라미터</h2>
        <span class="tag">조절 즉시 실시간 반영</span>
      </div>

      <div class="curve-preview-wrap">
        <div class="curve-caption">
          📈 <strong>각도 변화 요구치(Demand, 0°~180°)에 따른 서보 실제 변화량(α) 곡선</strong>
          (요구치 <span>{stallSafeDemand}</span>° 시 <strong class="ok">+{safeAlphaPreview}°</strong> 출력)
        </div>
        <canvas bind:this={expCurveCanvas} width="460" height="170"></canvas>
      </div>

      <div class="control-group">
        <div class="control-header">
          <span>1. 발 압력 센서 EMA 필터 계수 (ema_alpha)</span>
          <strong class="param-badge">{emaAlpha.toFixed(2)}</strong>
        </div>
        <p class="param-desc">낮을수록 노이즈 억제력이 커져 부드러워지고, 높을수록 발 압력 변화에 즉각 반응합니다. (발 송신부로 실시간 무선 동기화됨)</p>
        <div class="slider-row">
          <input type="range" min="0.05" max="0.95" step="0.05" bind:value={emaAlpha} on:input={() => { if (isSimMode && !isWsOpen()) runOfflineSimStep(); }} on:change={() => sendCmd('set_alpha', { val: emaAlpha })} />
          <input type="number" min="0.05" max="0.95" step="0.05" bind:value={emaAlpha} on:change={() => sendCmd('set_alpha', { val: emaAlpha })} />
        </div>
      </div>

      <div class="control-group">
        <div class="control-header">
          <span>2. 지수함수 최대 변화량 제한 (max_step_alpha)</span>
          <strong class="param-badge">{maxStepAlpha.toFixed(1)}°</strong>
        </div>
        <p class="param-desc">정규화 요구치가 최대(180°)일 때 1주기(20ms)당 허용할 서보의 최대 목표 변화량(최대 출력 토크 상한)입니다.</p>
        <div class="slider-row">
          <input type="range" min="5" max="70" step="1" bind:value={maxStepAlpha} on:input={() => { if (isSimMode && !isWsOpen()) runOfflineSimStep(); }} on:change={() => sendCmd('set_max_step', { val: maxStepAlpha })} />
          <input type="number" min="5" max="70" step="1" bind:value={maxStepAlpha} on:change={() => sendCmd('set_max_step', { val: maxStepAlpha })} />
        </div>
      </div>

      <div class="control-group">
        <div class="control-header">
          <span>3. 지수함수 곡률 상수 (exp_curve_k)</span>
          <strong class="param-badge">{expCurveK.toFixed(2)}</strong>
        </div>
        <p class="param-desc">작을수록(0.1) 1차 직선(선형 비례)에 가까워지고, 클수록(5.0) 미세 구간은 정밀하고 고압 구간에서 폭발적인 곡선이 됩니다.</p>
        <div class="slider-row">
          <input type="range" min="0.1" max="6.0" step="0.1" bind:value={expCurveK} on:input={() => { if (isSimMode && !isWsOpen()) runOfflineSimStep(); }} on:change={() => sendCmd('set_exp_k', { val: expCurveK })} />
          <input type="number" min="0.1" max="6.0" step="0.1" bind:value={expCurveK} on:change={() => sendCmd('set_exp_k', { val: expCurveK })} />
        </div>
      </div>

      <div class="control-group">
        <div class="control-header">
          <span>4. 3번 마디 전향 교차 결합 보상 계수 (coupling_gamma, γ)</span>
          <strong class="param-badge">{couplingGamma.toFixed(2)}</strong>
        </div>
        <p class="param-desc">1번 모터(1·2번 마디) 작동/파지 시 와이어 장력 반작용으로 3번 마디가 함께 움직이는 것을 방지하는 비율입니다: α₂_final = α₂_pure + slack₂ - (γ × α₁)</p>
        {#if isSingleMotor}
          <p class="single-inline-warn">ℹ️ 단일 모터 롤백 모드(NUM_MOTORS = 1)에서는 전향 교차 결합 보상이 적용되지 않습니다.</p>
        {/if}
        <div class="slider-row">
          <input type="range" min="0.00" max="0.60" step="0.02" bind:value={couplingGamma} on:input={() => { if (isSimMode && !isWsOpen()) runOfflineSimStep(); }} on:change={() => sendCmd('set_gamma', { val: couplingGamma })} />
          <input type="number" min="0.00" max="0.60" step="0.02" bind:value={couplingGamma} on:change={() => sendCmd('set_gamma', { val: couplingGamma })} />
        </div>
      </div>

      <div class="control-group">
        <div class="control-header">
          <span>5. 과부하(Stall) 요구치 제한 각도 & 제한 발동 시간</span>
          <strong class="param-badge">감지 {stallCutoffDemand}° → 제한 {stallSafeDemand}° ({Number(stallProtectSec).toFixed(1)}초)</strong>
        </div>
        <p class="param-desc">물체를 강하게 쥐거나 양방향으로 걸려 가동 범위 정규화 요구치(|effectiveDemand|)가 '감지 커트라인'을 초과한 상태로 '제한 시간' 이상 지속되면, 모터 보호를 위해 요구치를 '안전 제한 각도'로 하향 고정합니다.</p>

        <div class="stall-rows">
          <div class="slider-row">
            <span class="sub-lbl">안전 제한 각도 (°):</span>
            <input type="range" min="50" max="170" step="5" bind:value={stallSafeDemand} on:change={handleStallChange} />
            <input type="number" min="50" max="170" step="5" bind:value={stallSafeDemand} on:change={handleStallChange} />
          </div>
          <div class="slider-row">
            <span class="sub-lbl">감지 커트라인 (°):</span>
            <input type="range" min="60" max="180" step="5" bind:value={stallCutoffDemand} on:change={handleStallChange} />
            <input type="number" min="60" max="180" step="5" bind:value={stallCutoffDemand} on:change={handleStallChange} />
          </div>
          <div class="slider-row">
            <span class="sub-lbl">제한 발동 시간 (초):</span>
            <input type="range" min="1.0" max="15.0" step="0.5" bind:value={stallProtectSec} on:change={handleStallChange} />
            <input type="number" min="1.0" max="15.0" step="0.5" bind:value={stallProtectSec} on:change={handleStallChange} />
          </div>
        </div>
      </div>
    </section>

    <section class="card">
      <div class="card-head">
        <h2>캘리브레이션 & 비휘발성 메모리(Flash NVS) 관리</h2>
        <span class="tag">전원 재부팅 후에도 유지</span>
      </div>
      <div class="btn-grid">
        <button class="btn btn-accent" on:click={triggerCalibration}>
          {isSingleMotor ? '🔄 플렉스 센서 보정 시작 (1번 모터)' : '🔄 플렉스 센서 순차 보정 시작 (2번→1번)'}
        </button>
        <button class="btn btn-warn" on:click={() => sendCmd('reset_lut')}>
          📐 arccos 기본 LUT로 초기화
        </button>
        <button class="btn btn-primary btn-full" on:click={() => sendCmd('save_nvs')}>
          💾 모든 파라미터(1~5) 및 가변 단계 LUT 비휘발성 메모리(NVS) 영구 저장
        </button>
      </div>
    </section>
  {/if}

  <!-- [탭 3] 시스템 터미널 -->
  {#if activeTab === 'terminal'}
    <section class="card">
      <div class="card-head">
        <h2>실시간 시스템 메시지 & 오류 출력 터미널</h2>
        <div class="term-actions">
          <label class="auto-scroll-lbl">
            <input type="checkbox" bind:checked={autoScroll} /> 자동 스크롤
          </label>
          <button class="btn" style="padding:4px 10px; font-size:0.75rem;" on:click={clearLogs}>로그 지우기</button>
        </div>
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
        <input class="cmd-input" bind:value={manualCmd} placeholder='수동 명령 입력 (예: {"cmd":"get_config"} 또는 start_calib, save_nvs, toggle_pause)' on:keydown={(e) => e.key === 'Enter' && sendManualCommand()} />
        <button class="btn btn-primary" on:click={sendManualCommand}>전송</button>
      </div>
    </section>
  {/if}
</main>

<!-- 하단 상시 미니 시스템 로그 바 -->
<div class="bottom-log-bar" role="button" tabindex="0" on:click={() => activeTab = 'terminal'} on:keydown={(e) => e.key === 'Enter' && (activeTab = 'terminal')} title="클릭하면 시스템 터미널 탭으로 이동합니다">
  <div class="bottom-log-content">
    <span class="primary" style="font-weight:700;">[터미널]</span>
    <span class="bottom-log-msg">{lastLogText}</span>
  </div>
  <span class="primary" style="font-size:0.74rem; flex-shrink:0;">전체 로그 보기 ▲</span>
</div>

<style>
  :global(body) { background: #0b1120; color: #f8fafc; margin: 0; padding: 14px; padding-bottom: 64px; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Noto Sans KR", sans-serif; }
  .container { max-width: 820px; margin: 0 auto; display: flex; flex-direction: column; gap: 14px; }
  header { background: #1e293b; border: 1px solid #334155; border-radius: 12px; padding: 14px 16px; display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 10px; }
  h1 { font-size: 1.15rem; color: #38bdf8; margin: 0; }
  .subtitle { font-size: 0.76rem; color: #94a3b8; margin: 2px 0 0 0; }
  .header-right { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
  .status-badge { display: flex; align-items: center; gap: 6px; font-size: 0.8rem; padding: 5px 10px; border-radius: 999px; background: #0f172a; border: 1px solid #334155; }
  .status-dot { width: 8px; height: 8px; border-radius: 50%; background: #ef4444; }
  .status-dot.online { background: #10b981; box-shadow: 0 0 8px #10b981; }
  .status-dot.sim { background: #f59e0b; box-shadow: 0 0 8px #f59e0b; }

  .quick-grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 8px; }
  @media (max-width: 600px) { .quick-grid { grid-template-columns: repeat(2, 1fr); } }
  .pill { background: #1e293b; border: 1px solid #334155; border-radius: 10px; padding: 10px 12px; display: flex; justify-content: space-between; align-items: center; font-size: 0.8rem; }
  .ok { color: #10b981; }
  .warn { color: #f59e0b; }
  .danger { color: #ef4444; }
  .primary { color: #38bdf8; }

  .tab-nav { display: flex; background: #0f172a; border: 1px solid #334155; border-radius: 10px; padding: 4px; gap: 4px; }
  .tab-btn { flex: 1; padding: 10px 8px; border: none; background: transparent; color: #94a3b8; font-weight: 700; border-radius: 8px; cursor: pointer; display: flex; justify-content: center; align-items: center; gap: 6px; }
  .tab-btn.active { background: #38bdf8; color: #0b1120; }
  .tab-badge { font-size: 0.7rem; padding: 1px 6px; border-radius: 999px; background: rgba(15, 23, 42, 0.25); color: inherit; }

  .motor-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; }
  @media (max-width: 680px) { .motor-grid { grid-template-columns: 1fr; } }
  .card { background: #1e293b; border: 1px solid #334155; border-radius: 12px; padding: 16px; }
  .dimmed { opacity: 0.55; }
  .card-head { display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px; padding-bottom: 8px; border-bottom: 1px solid #334155; }
  .card-head h2 { font-size: 0.95rem; margin: 0; }
  .tag { font-size: 0.72rem; background: #0f172a; padding: 3px 8px; border-radius: 6px; border: 1px solid #334155; }
  .single-notice { margin-bottom: 10px; padding: 8px 10px; border-radius: 8px; background: rgba(245, 158, 11, 0.12); border: 1px solid rgba(245, 158, 11, 0.35); color: #f59e0b; font-size: 0.76rem; font-weight: 600; }
  .single-inline-warn { font-size: 0.73rem; color: #f59e0b; margin: 0 0 6px 0; font-weight: 600; }

  .metric-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-bottom: 10px; }
  .metric-box { background: #0f172a; border-radius: 8px; padding: 10px; text-align: center; }
  .highlight-span { grid-column: span 2; display: flex; justify-content: space-around; align-items: center; background: rgba(56, 189, 248, 0.08); border: 1px solid rgba(56, 189, 248, 0.3); }
  .v-sep { height: 36px; width: 1px; background: #334155; }
  .label { font-size: 0.72rem; color: #94a3b8; }
  .val { font-size: 1.4rem; font-weight: 800; color: #38bdf8; margin: 3px 0; }
  .val.green { color: #10b981; }
  .val.warn { color: #f59e0b; }
  .val.purple { color: #a855f7; }
  .sub { font-size: 0.7rem; color: #94a3b8; }

  .range-info { display: flex; justify-content: space-between; font-size: 0.74rem; color: #94a3b8; margin-bottom: 4px; }
  .bar-bg { width: 100%; height: 10px; background: #0f172a; border-radius: 5px; overflow: hidden; position: relative; border: 1px solid rgba(255, 255, 255, 0.06); }
  .bar-fill.green { height: 100%; background: #10b981; transition: width 0.08s ease; }
  .range-window { position: absolute; top: 0; bottom: 0; background: rgba(16, 185, 129, 0.18); border-left: 2px solid #10b981; border-right: 2px solid #10b981; }
  .marker-dot { position: absolute; top: -1px; width: 4px; height: 12px; background: #fff; border-radius: 2px; transform: translateX(-50%); transition: left 0.08s ease; }

  .sim-panel { background: rgba(56, 189, 248, 0.06); border: 1px dashed #38bdf8; border-radius: 10px; padding: 12px 14px; }
  .sim-head { display: flex; justify-content: space-between; align-items: center; margin-bottom: 8px; flex-wrap: wrap; gap: 6px; }
  .sim-title { font-size: 0.85rem; font-weight: 700; color: #38bdf8; }
  .sim-chk { font-size: 0.78rem; color: #94a3b8; cursor: pointer; }

  .curve-preview-wrap { display: flex; flex-direction: column; align-items: center; background: #0f172a; border: 1px solid #334155; border-radius: 10px; padding: 12px; margin-bottom: 12px; }
  .curve-caption { font-size: 0.78rem; color: #94a3b8; margin-bottom: 6px; text-align: center; }
  canvas { width: 100%; max-width: 460px; height: 170px; background: #0b1120; border-radius: 8px; border: 1px solid rgba(255, 255, 255, 0.07); }

  .control-group { background: #0f172a; border: 1px solid #334155; border-radius: 10px; padding: 12px 14px; margin-bottom: 10px; }
  .control-header { display: flex; justify-content: space-between; align-items: center; font-size: 0.86rem; font-weight: 700; margin-bottom: 4px; }
  .param-badge { font-size: 0.84rem; font-weight: 800; color: #38bdf8; background: rgba(56, 189, 248, 0.15); padding: 2px 10px; border-radius: 6px; }
  .param-desc { font-size: 0.74rem; color: #94a3b8; margin: 0 0 8px 0; line-height: 1.35; }
  .slider-row { display: flex; align-items: center; gap: 10px; }
  .stall-rows { display: flex; flex-direction: column; gap: 8px; margin-top: 6px; }
  .sub-lbl { font-size: 0.78rem; width: 130px; color: #94a3b8; flex-shrink: 0; }
  input[type=range] { flex: 1; accent-color: #38bdf8; cursor: pointer; height: 6px; }
  input[type=number] { width: 74px; background: #1e293b; border: 1px solid #334155; color: #f8fafc; padding: 5px 8px; border-radius: 6px; font-size: 0.82rem; font-weight: 700; text-align: center; }

  .btn-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-top: 4px; }
  .btn { padding: 10px 14px; border: none; border-radius: 8px; font-size: 0.84rem; font-weight: 700; cursor: pointer; background: #334155; color: #f8fafc; }
  .btn-primary { background: #38bdf8; color: #0b1120; }
  .btn-accent { background: #10b981; color: #0b1120; }
  .btn-warn { background: #f59e0b; color: #0b1120; }
  .btn-full { grid-column: span 2; }

  .term-actions { display: flex; gap: 8px; align-items: center; }
  .auto-scroll-lbl { font-size: 0.76rem; color: #94a3b8; cursor: pointer; }
  .terminal-box { background: #050811; border: 1px solid #334155; border-radius: 8px; height: 340px; overflow-y: auto; padding: 10px; font-family: "Consolas", "Monaco", monospace; font-size: 0.8rem; line-height: 1.5; }
  .log-line { margin-bottom: 4px; word-break: break-all; }
  .time { color: #64748b; margin-right: 6px; }
  .lvl.info { color: #38bdf8; font-weight: bold; margin-right: 6px; }
  .lvl.warn { color: #f59e0b; font-weight: bold; margin-right: 6px; }
  .lvl.err { color: #ef4444; font-weight: bold; margin-right: 6px; }
  .lvl.calib { color: #10b981; font-weight: bold; margin-right: 6px; }
  .cmd-row { display: flex; gap: 8px; margin-top: 10px; }
  .cmd-input { flex: 1; background: #0f172a; border: 1px solid #334155; color: #f8fafc; padding: 8px 12px; border-radius: 8px; font-family: monospace; }

  .bottom-log-bar { position: fixed; bottom: 0; left: 0; right: 0; background: rgba(15, 23, 42, 0.95); backdrop-filter: blur(6px); border-top: 1px solid #334155; padding: 8px 16px; display: flex; justify-content: space-between; align-items: center; font-size: 0.78rem; cursor: pointer; z-index: 50; }
  .bottom-log-content { display: flex; align-items: center; gap: 8px; overflow: hidden; white-space: nowrap; text-overflow: ellipsis; font-family: monospace; }
  .bottom-log-msg { color: #94a3b8; overflow: hidden; text-overflow: ellipsis; }
</style>
