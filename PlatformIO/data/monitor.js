(function () {
	'use strict';

	const terminal = document.getElementById('terminal');
	const input = document.getElementById('input-box');
	const form = document.getElementById('command-form');
	const clearButton = document.getElementById('clear-terminal');
	const reloadButton = document.getElementById('reload-page');
	const downloadButton = document.getElementById('download-logs');
	const statusBadge = document.getElementById('connection-status');
	const logCount = document.getElementById('log-count');

	let websocket = null;
	let reconnectTimer = null;
	let receivedLines = 0;
	const maxTerminalLines = 900;
	let terminalBuffer = ['Conectando ao ESP32...'];

	function setStatus(online, label) {
		statusBadge.textContent = label;
		statusBadge.classList.toggle('online', online);
		statusBadge.classList.toggle('offline', !online);
	}

	function updateLogCount() {
		logCount.textContent = `${String(receivedLines).padStart(4, '0').slice(-4)} linhas`;
	}

	function appendTerminal(text) {
		const cleanText = String(text).replace(/\n$/, '');
		terminalBuffer.push(cleanText);
		if (terminalBuffer.length > maxTerminalLines) {
			terminalBuffer = terminalBuffer.slice(-maxTerminalLines);
		}
		terminal.textContent = terminalBuffer.join('\n');
		terminal.scrollTop = terminal.scrollHeight;
	}

	function appendLine(text) {
		receivedLines += 1;
		updateLogCount();
		appendTerminal(text);
	}

	function clearTerminal() {
		terminalBuffer = [];
		terminal.textContent = '';
		receivedLines = 0;
		updateLogCount();
	}

	function downloadLogs() {
		const blob = new Blob([terminal.textContent], { type: 'text/plain' });
		const link = document.createElement('a');
		const formattedDate = new Date().toISOString().replace(/[:.]/g, '-');
		const url = window.URL.createObjectURL(blob);

		link.href = url;
		link.download = `logs-${formattedDate}.txt`;
		document.body.appendChild(link);
		link.click();
		document.body.removeChild(link);
		window.URL.revokeObjectURL(url);
	}

	function websocketUrl() {
		const protocol = window.location.protocol === 'https:' ? 'wss' : 'ws';
		return `${protocol}://${window.location.host}/ws`;
	}

	function scheduleReconnect() {
		window.clearTimeout(reconnectTimer);
		reconnectTimer = window.setTimeout(initWebSocket, 2000);
	}

	function onOpen() {
		setStatus(true, 'Online');
		appendLine('Conectado ao ESP32.');
	}

	function onClose() {
		setStatus(false, 'Reconectando');
		appendLine('Desconectado. Tentando reconectar em 2 segundos.');
		scheduleReconnect();
	}

	function onMessage(event) {
		const timeString = new Date().toLocaleTimeString();
		appendLine(`[${timeString}] ${event.data}`);
	}

	function initWebSocket() {
		if (!window.WebSocket) {
			setStatus(false, 'Sem WS');
			appendLine('Seu navegador nao suporta WebSockets.');
			return;
		}

		window.clearTimeout(reconnectTimer);
		setStatus(false, 'Conectando');

		websocket = new WebSocket(websocketUrl());
		websocket.onopen = onOpen;
		websocket.onclose = onClose;
		websocket.onmessage = onMessage;
		websocket.onerror = function () {
			setStatus(false, 'Erro');
		};
	}

	function sendCommand() {
		const command = input.value.trim();
		if (!command) {
			return;
		}

		if (!websocket || websocket.readyState !== WebSocket.OPEN) {
			appendLine(`Comando ignorado sem conexao: ${command}`);
			return;
		}

		websocket.send(command);
		appendLine(`> ${command}`);
		input.value = '';
		input.focus();
	}

	form.addEventListener('submit', function (event) {
		event.preventDefault();
		sendCommand();
	});

	clearButton.addEventListener('click', clearTerminal);
	reloadButton.addEventListener('click', function () {
		window.location.reload();
	});
	downloadButton.addEventListener('click', downloadLogs);

	updateLogCount();
	initWebSocket();
}());
