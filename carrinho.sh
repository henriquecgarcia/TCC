#!/bin/bash

ESP_TYPE="esp32:esp32:esp32doit-devkit-v1"
PORT="/dev/ttyACM0"
PROJECT="carrinho"
COMPILE=false
UPLOAD=false
MONITOR=false
CONFIG="115200"
# Commands: -c --compile, -u --upload, -h --help, -v --version, -p --port, -e --esp-type

if [ -z "$1" ]; then
	echo "No arguments provided. Considering default options, compiling and uploading the project."
	COMPILE=true
	UPLOAD=true
else
	echo "Arguments provided. Processing options..."
fi

# Checking arguments
for arg in "$@"; do
# Making it a switch case for better readability
	case "$arg" in
		--help|-h)
			echo "Usage: carrinho.sh [options]"
			echo "Options:"
			echo "  -c, --compile       Compile the project"
			echo "  -u, --upload        Upload the project to the ESP32"
			echo "  -h, --help          Show this help message"
			echo "  -v, --version       Show the version of the script"
			echo "  -p, --port <port>   Specify the port for the ESP32 (default: $PORT)"
			echo "  -e, --esp-type <type> Specify the ESP type (default: $ESP_TYPE)"
			exit 0
			;;
		--version|-v)
			echo "carrinho.sh version 1.0"
			exit 0
			;;
		--port|-p)
			if [ -n "$2" ]; then
				PORT="$2"
				shift 2
			else
				echo "Error: No port specified after --port or -p."
				exit 1
			fi
			;;
		--esp-type|-e)
			if [ -n "$2" ]; then
				ESP_TYPE="$2"
				shift 2
			else
				echo "Error: No ESP type specified after --esp-type or -e."
				exit 1
			fi
			;;
		-c|--compile)
			COMPILE=true
			;;
		-c|--compile)
			COMPILE=true
			;;
		-u|--upload)
			UPLOAD=true
			;;
		-m|--monitor)
			MONITOR=true
			;;
		--config|-cfg)
			if [ -n "$2" ]; then
				CONFIG="$2"
				shift 2
			else
				echo "Error: No config file specified after --config or -cfg."
				exit 1
			fi
			;;
		*)
			echo "Error: Unknown argument '$arg'. Use --help or -h for usage information."
			exit 1
			;;
	esac
done

if [ "$COMPILE" = true ]; then
	arduino-cli compile --fqbn $ESP_TYPE --port $PORT $PROJECT
	if [ $? -ne 0 ]; then
		echo "Compilation failed. Please check the code and try again."
		exit 1
	else
		echo "Compilation successful."
	fi
fi
if [ "$UPLOAD" = true ]; then
	arduino-cli upload --fqbn $ESP_TYPE --port $PORT $PROJECT
	if [ $? -ne 0 ]; then
		echo "Upload failed. Please check the connection and try again."
		exit 1
	else
		echo "Upload successful."
	fi
fi
if [ "$MONITOR" = true ]; then
	arduino-cli monitor -p $PORT --config $CONFIG
	if [ $? -ne 0 ]; then
		echo "Monitor failed. Please check the connection and try again."
		exit 1
	else
		echo "Monitor started successfully."
	fi
fi