const canvas = document.getElementById('map');
const ctx = canvas.getContext('2d');
const statusEl = document.getElementById('status');
const telemetryEl = document.getElementById('telemetry');
const toastEl = document.getElementById('toast');
let ws;
let toastTimer;
let reconnectTimer;
let last = { w: 32, h: 32, map: [], path: [], x: 0, y: 0, theta: 0, tof: 0, active: false };

function showToast(message) {
  if (!message) return;
  toastEl.textContent = message;
  toastEl.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => toastEl.classList.remove('show'), 4200);
}

function mapStringToArray(value) {
  if (typeof value !== 'string') return Array.isArray(value) ? value : last.map;
  const out = new Array(value.length);
  for (let i = 0; i < value.length; i++) out[i] = value.charCodeAt(i) - 48;
  return out;
}

function connect() {
  clearTimeout(reconnectTimer);
  ws = new WebSocket(`ws://${location.host}/ws`);

  ws.onopen = () => {
    statusEl.textContent = 'CONECTADO';
    showToast('WebSocket conectado.');
  };

  ws.onclose = () => {
    statusEl.textContent = 'DESCONECTADO';
    reconnectTimer = setTimeout(connect, 1500);
  };

  ws.onerror = () => {
    statusEl.textContent = 'ERRO WS';
  };

  ws.onmessage = e => {
    try {
      const msg = JSON.parse(e.data);

      if (msg.type === 'ack') {
        if (!msg.ok) showToast('Comando rejeitado pelo ESP32.');
        return;
      }

      if (msg.type === 'map') {
        last.w = msg.w || last.w;
        last.h = msg.h || last.h;
        last.map = mapStringToArray(msg.map);
        draw();
        return;
      }

      if (msg.type === 'path') {
        last.path = msg.path || [];
        draw();
        return;
      }

      if (msg.type === 'telemetry') {
        last.x = msg.x ?? last.x;
        last.y = msg.y ?? last.y;
        last.theta = msg.theta ?? last.theta;
        last.tof = msg.tof ?? last.tof;
        last.active = msg.active ?? last.active;
        if (msg.toast) showToast(msg.toast);
        draw();
        telemetryEl.textContent = `x: ${last.x.toFixed(3)} m\ny: ${last.y.toFixed(3)} m\ntheta: ${last.theta.toFixed(3)} rad\ntof: ${last.tof} mm\nativo: ${last.active}`;
      }
    } catch (err) {
      console.log(err);
    }
  };
}

function send(o) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(o));
  } else {
    showToast('WebSocket desconectado. Aguarde reconectar.');
  }
}

function draw() {
  const w = last.w || 32;
  const h = last.h || 32;
  const cw = canvas.width / w;
  const ch = canvas.height / h;
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      const v = last.map?.[y * w + x] ?? 0;
      ctx.fillStyle = v === 0 ? '#111' : (v === 1 ? '#5b3715' : '#c84727');
      ctx.fillRect(x * cw, y * ch, cw - 1, ch - 1);
    }
  }

  ctx.strokeStyle = '#ff9b21';
  ctx.lineWidth = 3;
  ctx.beginPath();
  (last.path || []).forEach((p, i) => {
    const px = p.x / 0.10 * cw;
    const py = p.y / 0.10 * ch;
    if (i === 0) ctx.moveTo(px, py);
    else ctx.lineTo(px, py);
  });
  ctx.stroke();

  const rx = (last.x || 0) / 0.10 * cw;
  const ry = (last.y || 0) / 0.10 * ch;
  ctx.save();
  ctx.translate(rx, ry);
  ctx.rotate(last.theta || 0);
  ctx.fillStyle = '#f7c873';
  ctx.beginPath();
  ctx.moveTo(14, 0);
  ctx.lineTo(-10, -8);
  ctx.lineTo(-10, 8);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

document.getElementById('sendGoal').onclick = () => send({ cmd: 'goal', gx: +document.getElementById('gx').value, gy: +document.getElementById('gy').value });
document.getElementById('stop').onclick = () => send({ cmd: 'stop' });
document.getElementById('clearDynamic').onclick = () => send({ cmd: 'clearDynamic' });
document.getElementById('setPose').onclick = () => send({ cmd: 'pose', x: +document.getElementById('px').value, y: +document.getElementById('py').value, theta: +document.getElementById('pt').value });
document.getElementById('forward').onclick = () => send({ cmd: 'forward' });
document.getElementById('backward').onclick = () => send({ cmd: 'backward' });
document.getElementById('turnRight90').onclick = () => send({ cmd: 'turnRight90' });
document.getElementById('turnLeft90').onclick = () => send({ cmd: 'turnLeft90' });

connect();
draw();
