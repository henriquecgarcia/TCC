# Autonomous Car for Delivery
Final Course Project fo Computer Science at UNIFESP (Federal University of São Paulo).

## Project Details:
**Author**: Henrique Campanha Garcia
**Professor**: Dr. André Marcorin de Oliveira
**Course**: Computer Science
**University**: Federal University of São Paulo (UNIFESP)

## Description
This project is an autonomous car (drone) with the porpose of transporting packages from one place to another using a known map. The drone will be fully autonomous, using an ESP32, a motor driver, 4 motors, a battery and some other sensors for minor systems. To operate the drone, you will connect on it's IP connection, select wherever you want it to go, put the package on top of the drone and press for it to go.

The drone also has some other sensors that will help on it's navigation and controling the location where it is and preventing it from colliding with things on it's way.

### How to use:
To use this project, you need to have the following libraries installed:
- `WiFi`
- `ESPAsyncWebServer` (From me-do-dev)
- `SPIFFS`
- `VL53L0x` (From Adafruit)
- `MPU6050` (From Adafruit)

## Components:
- ESP32
- Motor Driver
- 4 DC Motors (3-6v DC Motor)
- 2 Encoder (Q2-E2)
- Battery
- 1 VL53L0x
- 1 MPU6050 Acelerometer & Gyroscope

### ESP32
The ESP32 used is the ESP32 DOIT DEVKIT V1, which has a ESP-WROOM-32 module, which has a dual core processor, 4MB of flash memory, 520KB of SRAM, and a lot of other features.

### Motor Driver & DC Motors
The motor driver used is the L298N, which is a dual H-Bridge motor driver, which can control 2 motors at the same time, and can control the speed of the motors.
> Obs: In each side of the car, there are 2 motors, meaning the the Motor Driver is connected to 4 motors at once (2 in each side).

The motors used are 3-6v DC Motors, which are small and lightweight, and can be controlled by the motor driver to move the car forward, backwards and turn. In the bought motors, they already had a reductor, which allows the car to move at a constant speed and to have more torque, allowing it to carry a package on top of it.

### Battery
The battery used is a 7.4v LiPo battery, which is lightweight and has a high capacity, allowing the car to move for a long time without needing to be recharged.
> Obs: The battery is connected to the motor driver, which allows the car to move. And the Motor Driver sends 5v to the ESP32, but since it only has 3.3v logic, the ESP32 converts the 5v to 3.3v using the vin pin.

### Encoders E2-Q2
The encoders are mounted on the forward wheels and counting each change on the reflective sensors, allowing the ESP32 to count the number of "changes" on the sensors, therefore, knowing that the used reference wheel there is 10 teeth, and the actual wheel has 65 mm of diameter, we can calculate the distance traveled by the car in a second by using the formula:
```
Distance = ((Number of changes) / 2 * 65mm) / (10 * 2)
RPM = (Number of changes / interval) * 60
```
> The division by 2 is because the encoder has 2 channels, and we are only using one of them, so we need to divide the number of changes by 2 to get the actual number of changes on the wheel. The division by 10 * 2 (20) is because the encoder has 10 teeth, and we are using the number of changes to calculate the distance traveled by the car in a second, so we need to divide the number of changes by 20 to get the actual distance traveled by the car in a second.
> Obs: The interval used is of 100ms, meaning that the car will calculate the distance traveled by the car every 100ms, and the speed of the car will be calculated every second.

### VL53L0x
Using the time-of-flight sensor, the VL53L0x can measure distances up to 3 meters with high accuracy. It uses a laser to measure the time it takes for the light to bounce back from an object, allowing it to calculate the distance.
> In the project, the sensor is mounted infront, facing forward to the front, to avoid any collision while on the path to the destination.

### MPU6050 Acelerometer & Gyroscope
The MPU6050 is a 6-axis motion tracking device that combines a 3-axis gyroscope and a 3-axis accelerometer on a single chip. It can be used to measure the orientation and motion of the car, allowing for more advanced control and navigation.
The MPU6050 is connected to the ESP32 via I2C, and it is used to measure the orientation and motion of the car, allowing for more advanced control and navigation. The MPU6050 can be used to detect the direction of the car, and to control the speed of the motors, so the car can go straight and not turn while going forward/backwards.
> Obs: The MPU6050 also has a temperature sensor, but it is not used in this project.

---

## How it works:
**Some awsome remarks of the project will be here, but I don't want to write them now, so I will write them later.**

## Pinout
The pinout used in this project is the following:
- Motor Driver:
	- ENA: 32
	- INA1: 12
	- INA2: 13
	- INB1: 26
	- INB2: 25
	- ENB: 33
- Encoders:
	- Front-Right:
		- Channel A: 4
		- Channel B: 15
	- Front-Left:
		- Channel A: 17
		- Channel B: 16
- MPU6050 & VL53L0x:
	- Via I2C.
	- SDA: 21
	- SCL: 22

## Final remarks:
This project was started in the class of Internet of Things (IoT) at the Federal University of São Paulo (UNIFESP), by Henrique Campanha Garcia, under the supervision of Professor Dr. André Marcorin de Oliveira. The project was updated and made to be fully autonomous and to be able to move in a path that it would not deviate from the pathm and to change the object avoidance system to use a time-of-flight sensor instead of an ultrasonic sensor. The project had to have a few bunch of changes to work with the new components, have a PID controler to control the speed of the motors. Some of the motor control was maintained from the previous project, but the new components and the new control system made it necessary to change a lot of the code.
The project has a plan to scan the environment and to create a map of the environment, so the car can navigate through the environment and avoid obstacles. Ideally, when fully autonomous, the vehicle would save the map using the SPIFFS filesystem, and then use the map to navigate through the environment, avoiding "static" obstacles, like walls and furniture, but also avoiding "dynamic" obstacles, like people and animals.
The way that it was setup, the car uses the encoders and the MPU6050 to control the direction of the car, avoiding that the car turns while going forward or backwards, and using the VL53L0x to avoid collisions with objects in front of the car, stopping the car when it detects an object closer than 10cm.

---
---
# Old stuff
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
