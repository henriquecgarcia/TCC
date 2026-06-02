(() => {
  'use strict';

  const $ = (id) => document.getElementById(id);
  const statusBadge = $('connection-status');
  const packetCount = $('packet-count');
  const ioLog = $('io-log');
  const mapCanvas = $('map-canvas');
  const ctx = mapCanvas.getContext('2d');

  let ws;
  let reconnectTimer;
  let packets = 0;
  let mapState = null;
  let lastLocation = null;
  let plannedPath = [];

  const send = (payload) => {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      addLog('WebSocket offline; comando não enviado.');
      return;
    }
    ws.send(typeof payload === 'string' ? payload : JSON.stringify(payload));
    addLog(`TX ${typeof payload === 'string' ? payload : JSON.stringify(payload)}`);
  };

  const setText = (id, value) => {
    const el = $(id);
    if (el) el.textContent = value;
  };

  const fixed = (value, digits = 2) => {
    const n = Number(value);
    return Number.isFinite(n) ? n.toFixed(digits) : '--';
  };

  const boolLabel = (value) => value === true ? 'Sim' : value === false ? 'Não' : '--';

  function addLog(text) {
    const line = document.createElement('p');
    line.textContent = `[${new Date().toLocaleTimeString()}] ${text}`;
    ioLog.appendChild(line);
    while (ioLog.children.length > 90) ioLog.removeChild(ioLog.firstChild);
    ioLog.scrollTop = ioLog.scrollHeight;
  }

  function setConnection(online) {
    statusBadge.textContent = online ? 'Online' : 'Offline';
    statusBadge.classList.toggle('is-online', online);
    statusBadge.classList.toggle('is-offline', !online);
    setText('input-status', online ? 'Online' : 'Reconectando');
  }

  function connect() {
    clearTimeout(reconnectTimer);
    ws = new WebSocket(`ws://${location.host}/car`);

    ws.addEventListener('open', () => {
      setConnection(true);
      addLog('WebSocket conectado.');
      send('status');
      send('map');
    });

    ws.addEventListener('close', () => {
      setConnection(false);
      addLog('WebSocket desconectado.');
      reconnectTimer = setTimeout(connect, 1200);
    });

    ws.addEventListener('error', () => {
      setConnection(false);
    });

    ws.addEventListener('message', (event) => {
      packets += 1;
      packetCount.textContent = `${String(packets).padStart(4, '0')} pacotes`;
      handleMessage(event.data);
    });
  }

  function handleMessage(raw) {
    let data;
    try {
      data = JSON.parse(raw);
    } catch (_) {
      addLog(`RX ${raw}`);
      return;
    }

    if (data.ready) return;

    if (data.type === 'map') {
      updateMap(data.map);
      return;
    }

    if (data.map && data.map.type === 'map') {
      updateMap(data.map.map);
      return;
    }

    if (data.type === 'path') {
      plannedPath = Array.isArray(data.cells) ? data.cells : [];
      addLog(data.ok ? `Caminho recebido com ${plannedPath.length} células.` : `Falha no caminho: ${data.reason || '--'}`);
      drawMap();
      send('map');
      return;
    }

    if (data.type === 'cell_nav') {
      updateCellNav(data);
      return;
    }

    updateTelemetry(data);
  }

  function updateTelemetry(data) {
    setText('robot-status', data.status || '--');
    setText('robot-direction', `Direção: ${data.direction || '--'}`);

    if (data.location) {
      lastLocation = data.location;
      setText('grid-x', data.location.grid_x ?? '--');
      setText('grid-y', data.location.grid_y ?? '--');
      setText('theta-deg', `${fixed(data.location.theta_deg, 1)}°`);
      setText('world-position', `x=${fixed(data.location.world_x_m, 3)}m · y=${fixed(data.location.world_y_m, 3)}m`);
      setText('cell-size', `Célula: ${fixed((data.location.cell_size_m || 0) * 100, 0)} cm`);
    }

    setText('front-distance', Number.isFinite(Number(data.distance)) ? `${data.distance} mm` : '--');
    if (data.sensor_age) {
      setText('sensor-age', `MPU ${data.sensor_age.mpu_ms ?? '--'} ms · ToF ${data.sensor_age.tof_ms ?? '--'} ms`);
    }

    if (data.cell_nav) updateCellNav(data.cell_nav);

    if (data.motorD) {
      setText('motor-d-rpm', fixed(data.motorD.rpm, 1));
      setText('motor-d-pid', fixed(data.motorD.pidOutput, 1));
      setText('motor-d-target', fixed(data.motorD.targetRPM, 1));
    }
    if (data.motorE) {
      setText('motor-e-rpm', fixed(data.motorE.rpm, 1));
      setText('motor-e-pid', fixed(data.motorE.pidOutput, 1));
      setText('motor-e-target', fixed(data.motorE.targetRPM, 1));
    }

    drawMap();
  }

  function updateCellNav(nav) {
    setText('cell-nav-state', nav.state || nav.event || '--');
    const completed = nav.completed ?? '--';
    const requested = nav.requested ?? '--';
    setText('cell-nav-progress', `${completed}/${requested}`);
    setText('cell-nav-path', nav.path_mode === true ? `${nav.path_index ?? 0}/${nav.path_len ?? 0}` : 'Manual');
    setText('cell-nav-reverse', boolLabel(nav.reverse));
    setText('cell-nav-obstacle', boolLabel(nav.obstacle));
    setText('cell-nav-log', boolLabel(nav.logger_enabled));
    setText('cell-nav-abort', nav.abort_reason || nav.reason || '--');
    if (nav.event) addLog(`NAV ${nav.event} · ${nav.state || '--'}${nav.reverse ? ' · ré' : ''}`);
  }

  function updateMap(map) {
    if (!map) return;
    mapState = map;
    if (map.robot) lastLocation = {
      grid_x: map.robot.x,
      grid_y: map.robot.y,
      world_x_m: map.robot.world_x_m,
      world_y_m: map.robot.world_y_m,
      theta_deg: map.robot.theta_deg
    };
    if (Array.isArray(map.path) && map.path.length) plannedPath = map.path;
    setText('map-info', `${map.width || '--'}x${map.height || '--'} · célula ${fixed((map.cell_size_m || 0) * 100, 0)} cm`);
    drawMap();
  }

  function drawMap() {
    const w = mapCanvas.width;
    const h = mapCanvas.height;
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = '#f8fafc';
    ctx.fillRect(0, 0, w, h);

    const rows = mapState?.rows || [
      '1111111', '1000001', '1000001', '1000001', '1000001', '1000001', '1111111'
    ];
    const height = rows.length;
    const width = rows[0]?.length || 7;
    const pad = 26;
    const cell = Math.floor(Math.min((w - pad * 2) / width, (h - pad * 2) / height));
    const ox = Math.floor((w - cell * width) / 2);
    const oy = Math.floor((h - cell * height) / 2);

    const pathSet = new Set((plannedPath || []).map((p) => `${p.x},${p.y}`));
    const target = mapState?.target;
    const robotX = lastLocation?.grid_x ?? mapState?.robot?.x ?? 1;
    const robotY = lastLocation?.grid_y ?? mapState?.robot?.y ?? 1;

    for (let y = 0; y < height; y += 1) {
      for (let x = 0; x < width; x += 1) {
        const absoluteX = (mapState?.viewStartX || 0) + x;
        const absoluteY = (mapState?.viewStartY || 0) + y;
        const px = ox + x * cell;
        const py = oy + y * cell;
        const occupied = rows[y]?.[x] === '1';
        ctx.fillStyle = occupied ? '#0f172a' : '#e2e8f0';
        ctx.fillRect(px, py, cell - 5, cell - 5);

        if (pathSet.has(`${absoluteX},${absoluteY}`)) {
          ctx.fillStyle = 'rgba(124, 58, 237, .64)';
          ctx.fillRect(px + 7, py + 7, cell - 19, cell - 19);
        }

        if (target && target.x === absoluteX && target.y === absoluteY) {
          ctx.fillStyle = '#059669';
          ctx.beginPath();
          ctx.arc(px + cell / 2, py + cell / 2, Math.max(8, cell * .18), 0, Math.PI * 2);
          ctx.fill();
        }

        if (robotX === absoluteX && robotY === absoluteY) {
          ctx.fillStyle = '#2563eb';
          ctx.beginPath();
          ctx.arc(px + cell / 2, py + cell / 2, Math.max(12, cell * .28), 0, Math.PI * 2);
          ctx.fill();
          ctx.fillStyle = '#fff';
          ctx.font = `700 ${Math.max(12, cell * .18)}px system-ui`;
          ctx.textAlign = 'center';
          ctx.textBaseline = 'middle';
          ctx.fillText('R', px + cell / 2, py + cell / 2);
        }

        ctx.fillStyle = occupied ? 'rgba(255,255,255,.8)' : '#64748b';
        ctx.font = `600 ${Math.max(10, cell * .13)}px system-ui`;
        ctx.textAlign = 'left';
        ctx.textBaseline = 'top';
        ctx.fillText(`${absoluteX},${absoluteY}`, px + 7, py + 6);
      }
    }
  }

  document.addEventListener('click', (event) => {
    const button = event.target.closest('[data-command]');
    if (!button) return;
    send(button.dataset.command);
  });

  $('command-form').addEventListener('submit', (event) => {
    event.preventDefault();
    const value = $('command-input').value.trim();
    if (value) send(value);
    $('command-input').value = '';
  });

  $('path-form').addEventListener('submit', (event) => {
    event.preventDefault();
    send({ action: 'path_to', x: Number($('target-x').value), y: Number($('target-y').value), execute: true });
  });

  drawMap();
  connect();
})();
