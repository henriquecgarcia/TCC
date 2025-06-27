#!/bin/bash

# carrinho.sh - script de automação de build/upload para ESP32
# Autor: Henrique Campanha Garcia
# TCC - Ciência da Computação - UNIFESP São José dos Campos
# Versão: 1.1
# Uso: ./carrinho.sh [opções]
# Licença: MIT
# Descrição: Este script automatiza o processo de compilação, upload e monitoramento de um projeto Arduino para um ESP32.
# Requisitos: arduino-cli instalado e configurado, ESP32 conectado via USB.


# ======== Configurações iniciais ========
ESP_TYPE="esp32:esp32:esp32doit-devkit-v1"
PORT="/dev/ttyACM0"
PROJECT="carrinho"
CONFIG="115200"
SHOULD_LOG=false
COMPILE=false
UPLOAD=false
MONITOR=false

if [ -z "$(command -v arduino-cli)" ]; then
	echo "Erro: arduino-cli não encontrado. Instale-o e configure-o corretamente."
	exit 1
fi

if [ ! -d "$PROJECT" ]; then
	echo "Erro: Projeto '$PROJECT' não encontrado. Certifique-se de que o diretório existe."
	exit 1
fi

LOG_DIR="${PROJECT}/logs"
mkdir -p "$LOG_DIR"

# ======== Funções utilitárias ========
log_msg() {
	echo "$1"
	[ "$SHOULD_LOG" = true ] && echo "$1" >> "$LOG_FILE"
}

run_and_log() {
	if [ "$SHOULD_LOG" = true ]; then
		"$@" &>> "$LOG_FILE"
	else
		"$@"
	fi
}

print_help() {
	cat <<EOF
Usage: carrinho.sh [options]
Options:
	-c, --compile         Compile the project
	-u, --upload          Upload the project to the ESP32
	-m, --monitor         Monitor serial output
	-l, --log             Enable logging to a timestamped file
	-p, --port <port>     Set the serial port (default: $PORT)
	-e, --esp-type <type> Set the ESP32 board type (default: $ESP_TYPE)
	-cfg, --config <baud> Set baud rate for monitor (default: $CONFIG)
	-v, --version         Show script version
	-h, --help            Show this help message
EOF
}

# ======== Processamento de argumentos ========
while [[ $# -gt 0 ]]; do
	case "$1" in
		-c|--compile) COMPILE=true ;;
		-u|--upload) UPLOAD=true ;;
		-m|--monitor) MONITOR=true ;;
		-l|--log) SHOULD_LOG=true ;;
		-p|--port) PORT="$2"; shift ;;
		-e|--esp-type) ESP_TYPE="$2"; shift ;;
		-cfg|--config) CONFIG="$2"; shift ;;
		-v|--version) echo "carrinho.sh version 1.1"; exit 0 ;;
		-h|--help) print_help; exit 0 ;;
		*) echo "Unknown argument: $1"; print_help; exit 1 ;;
	esac
	shift
done

# ======== Inicialização do log se necessário ========
if [ "$SHOULD_LOG" = true ]; then
	LOG_FILE="${LOG_DIR}/log_$(date +%Y.%m.%d_%H_%M).txt"
	log_msg "Log iniciado em $LOG_FILE"
fi

# ======== Execuções ========
if [ "$COMPILE" = true ]; then
	log_msg "Compilando projeto..."
	run_and_log arduino-cli compile --fqbn "$ESP_TYPE" "$PROJECT"
	[ $? -ne 0 ] && log_msg "Erro na compilação." && exit 1
	log_msg "Compilação bem-sucedida."
fi

if [ "$UPLOAD" = true ]; then
	log_msg "Enviando código para $PORT..."
	run_and_log arduino-cli upload --fqbn "$ESP_TYPE" --port "$PORT" --input-dir "$PROJECT"/data
	[ $? -ne 0 ] && log_msg "Erro no upload." && exit 1
	log_msg "Upload bem-sucedido."
fi

if [ "$MONITOR" = true ]; then
	log_msg "Iniciando monitor serial em $PORT (baud: $CONFIG)..."
	if [ "$SHOULD_LOG" = true ]; then
		arduino-cli monitor -p "$PORT" --config "$CONFIG" | tee -a "$LOG_FILE"
	else
		arduino-cli monitor -p "$PORT" --config "$CONFIG"
	fi
fi