(function () {
	'use strict';

	const statusBadge = document.getElementById('connection-status');
	const bootCount = document.getElementById('boot-count');
	const brownoutCount = document.getElementById('brownout-count');
	const lastReset = document.getElementById('last-reset');
	const wasBrownout = document.getElementById('was-brownout');
	const eventCount = document.getElementById('event-count');
	const eventsBody = document.getElementById('events-body');
	const note = document.getElementById('note');
	const refreshButton = document.getElementById('refresh-logs');
	const clearButton = document.getElementById('clear-logs');

	function setStatus(online, label) {
		statusBadge.textContent = label;
		statusBadge.classList.toggle('is-online', online);
		statusBadge.classList.toggle('is-offline', !online);
	}

	function formatMs(ms) {
		const value = Number(ms || 0);
		if (value < 1000) return `${value} ms`;
		return `${(value / 1000).toFixed(2)} s`;
	}

	function render(data) {
		bootCount.textContent = data.bootCount ?? '--';
		brownoutCount.textContent = data.brownoutCount ?? '--';
		lastReset.textContent = data.lastResetReason || '--';
		wasBrownout.textContent = data.lastResetWasBrownout ? 'BROWNOUT' : 'NORMAL';
		note.textContent = data.note || note.textContent;

		const events = Array.isArray(data.events) ? data.events : [];
		eventCount.textContent = String(events.length).padStart(3, '0');

		if (!events.length) {
			eventsBody.innerHTML = '<tr><td colspan="4">Nenhum brownout registrado.</td></tr>';
			return;
		}

		eventsBody.innerHTML = events.map(function (event) {
			return `<tr><td>${event.event ?? '--'}</td><td>${event.boot ?? '--'}</td><td>${formatMs(event.uptime_ms)}</td><td>${event.reason || '--'}</td></tr>`;
		}).join('');
	}

	async function loadLogs() {
		setStatus(false, 'Carregando');
		try {
			const response = await fetch('/api/brownout-logs', { cache: 'no-store' });
			if (!response.ok) throw new Error(`HTTP ${response.status}`);
			render(await response.json());
			setStatus(true, 'Online');
		} catch (error) {
			setStatus(false, 'Erro');
			eventsBody.innerHTML = `<tr><td colspan="4">Falha ao carregar logs: ${error.message}</td></tr>`;
		}
	}

	async function clearLogs() {
		setStatus(false, 'Limpando');
		try {
			const response = await fetch('/api/brownout-logs/clear', { method: 'POST', cache: 'no-store' });
			if (!response.ok) throw new Error(`HTTP ${response.status}`);
			render(await response.json());
			setStatus(true, 'Online');
		} catch (error) {
			setStatus(false, 'Erro');
			eventsBody.innerHTML = `<tr><td colspan="4">Falha ao limpar logs: ${error.message}</td></tr>`;
		}
	}

	refreshButton.addEventListener('click', loadLogs);
	clearButton.addEventListener('click', clearLogs);
	loadLogs();
	window.setInterval(loadLogs, 10000);
}());
