# Remote Controled Car
This is a project to build a remote controlled car using an ESP32, a motor driver, 4 motors, a battery and a remote control. The remote control used is a normal gamepad controller, but we will only be using the triggers and the left joystick, where the right trigger will be used to move the car forward, the left trigger will be used to move the car backwards, and the left joystick will be used to steer the car, though the car will only be able to do one of these actions at a time.

The car has also some other components, like a LDR to detect light, DHT11 to detect temperature and humidity, and a ultrasonic sensor to detect distance ahead of the car. It will send this data to a MQTT broker, so it can be accessed from anywhere.

The MQTT broker used is the Flespi MQTT broker, and when a button is pressed on the MQTT client, a LED on the car will light change simbolizing that there was a light turned on inside the greenhouse has been turned on or off.

## How to use
To use this project, you will need to have the following libraries installed:
- `WiFi`
- `PubSubClient`
- `Bluepad32` (Used for the controller)

# Components
- ESP32
- Motor Driver
- 4 CC Motors
- Battery
- Remote Control
- LDR
- DHT11
- 2 LEDs
- Ultrasonic Sensor (HC-SR04)

## ESP32
The ESP32 used is the ESP32 DOIT DEVKIT V1, which has a ESP-WROOM-32 module, which has a dual core processor, 4MB of flash memory, 520KB of SRAM, and a lot of other features.

## Motor Driver
The motor driver used is the L298N, which is a dual H-Bridge motor driver, which can control 2 motors at the same time, and can control the speed of the motors.
> Obs: In this project, the motor driver and the motors were already built in a car chassis, so I don't have the exact model of the motors used.

> Obs2: In each side of the car, there are 4 motors, meaning the the Motor Driver is connected to 4 motors at once (2 in each side).

## Battery
>> idk, need to check

## Remote Control
>> idk, need to check

## LDR
The LDR used is a simple LDR, which is used to detect light. When the light is on, the resistance of the LDR decreases, and when the light is off, the resistance increases.

## DHT11
The DHT11 is a sensor that can detect temperature and humidity. It is a simple sensor, and it is not very accurate, but it is good enough for this project.

## Ultrasonic Sensor
The ultrasonic sensor used is the HC-SR04, which can detect distance ahead of the car. It works by sending a sound wave, and then waiting for the sound wave to return. The time it takes for the sound wave to return is used to calculate the distance.
> Formula for calculating the distance: `distance = (time * speed of sound) / 2`

## MQTT
The MQTT broker used is the Flespi MQTT broker, which is a free MQTT broker that can be used for testing purposes. It is a cloud based MQTT broker, and it is very easy to use.

---
# How it works
The ESP32 is the main component of the car, and it is responsible for controlling the car, and sending data to the MQTT broker.

The car can be controlled using the remote control, which is connected to the ESP32 via Bluepad32 (Bluetooth). On the controller, when it's pressed the right trigger, the car will move forward, when it's pressed the left trigger, the car will move backwards, and when the left joystick is moved, the car will turn.

While the car is moving, the LDR will detect the light, and the DHT11 will detect the temperature and humidity. Every 0.5 seconds, the ESP32 will send this data to the MQTT broker, so it can be accessed from anywhere.
> Reason for the time: So it does not overload the MQTT broker with data.

The ultrasonic sensor is used to detect distance ahead of the car, and when the distance is less than 10cm, the car will stop moving forward and will only allow the car to move backwards or turn.

Whenever the LDR detects low light, the ESP32 will send a message to the MQTT broker, and a LED on the car will light up. When the LDR detects high light, the ESP32 will send another message to the MQTT broker, and the LED will turn off.

On the MQTT viewer, there will be 2 main buttons, one to turn on or off the light inside the greenhouse, and another to turn on or off the light on the car. When the button to turn on or off the light inside the greenhouse is pressed, a LED added on the ESP32 protoboard will light up or turn off, simbolizing that the light inside the greenhouse has been turned on or off. Same thing happens when the button to turn on or off the light on the car is pressed.

## Pinout
The pinout used in this project is the following:
- Motor Driver:
	- ENA: not used
	- IN1: 25
	- IN2: 33
	- IN3: 32
	- IN4: 14
	- ENB: not used
- LDR: 34
- DHT11: 35
- Ultrasonic Sensor:
	- Trig: 26
	- Echo: 27
- LEDs:
	- Green: 2
	- Red: 4

## MQTT Topics
For MQTT topics, the following topics are used, all of them starting with `/Henrique/IoT/TF`:
- `/Henrique/IoT/TF/LDR`: Used to send the LDR data
- `/Henrique/IoT/TF/DHT/Temperature`: Used to send the temperature data
- `/Henrique/IoT/TF/DHT/Humidity`: Used to send the humidity data
- `/Henrique/IoT/TF/LED_CARRO`: Used to turn on or off the light on the car (The car will not publish to this topic, only subscribe)
- `/Henrique/IoT/TF/LED_ESTUFA`: Used to turn on or off the light inside the greenhouse

## MQTT Buttons
Within the MQTT, there are 2 buttons, one to turn on or off the light inside the greenhouse, and another to turn on or off the light on the car. Furthermore, there is a view section where the data from the LDR can be seen, and two sections where the temperature and humidity data can be seen.

Topics for the buttons:
- `/Henrique/IoT/TF/LED_CARRO`: Used to turn on or off the light on the car
- `/Henrique/IoT/TF/LED_ESTUFA`: Used to turn on or off the light inside the greenhouse

---

# Final remarks:
This project was made for the IoT class at the Federal University of São Paulo (UNIFESP), it was made by Henrique Campanha Garcia, using Visual Studio Code programming in Arduino, compiling using the Arduino IDE.

For the network connection, it was used my personal hotspot, witch has a meme ssid and password. The MQTT Broker used was the Flespi MQTT Broker and it's login will be changed after the project is finished.
