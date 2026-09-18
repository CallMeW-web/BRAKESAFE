BRAKESAFE
IoT-Based Accident-Prone Road Detection and Heatmap Monitoring System Using Brake Force Analytics in Bantayan Island

BRAKESAFE is an IoT-based road safety monitoring system that detects and classifies vehicle braking events using an ESP32, MPU6050 accelerometer, and GPS module. It records braking behavior with geographic coordinates and provides a local web-based dashboard for monitoring brake events and identifying accident-prone areas.

Designed for Bantayan Island, the system transforms vehicle braking data into location-based road safety information.

Features
Brake Force Detection: Detects vehicle deceleration using the MPU6050 accelerometer and classifies braking events by intensity.

GPS Location Tracking: Records latitude and longitude for each detected braking event using TinyGPSPlus.

Brake Classification: Categorizes events as Normal, Soft, Moderate, Hard, or Emergency.

Accident-Prone Area Detection: Identifies locations associated with emergency braking and supports manually adding accident-prone areas with location and reason.

Interactive Map: Displays brake events and accident-prone locations using Leaflet.js.

Web Dashboard: Provides real-time monitoring of system status, GPS coordinates, vehicle speed, brake level, deceleration, event count, Wi-Fi status, and buzzer status.

Data Export: Exports brake-event and accident-prone-area data in CSV format.

Wi-Fi and Access Point Mode: Connects to a configured Wi-Fi network or automatically starts an ESP32 Access Point when Wi-Fi is unavailable.

Local Data Persistence: Stores brake events and configuration locally using ESP32 flash preferences and operates without cloud dependency.

System Overview
Sensors collect deceleration and location data. The ESP32 processes the sensor data, classifies brake events, stores records locally, and hosts a local web dashboard. The dashboard provides access to brake event maps and accident-prone-area monitoring.

Hardware Requirements
Component	Purpose
ESP32 DevKit V1	Main microcontroller
MPU6050	Acceleration and deceleration measurement
GPS Module	Location and speed information
Green LED	Normal braking and system indication
Yellow LED	Moderate and hard braking indication
Red LED	Emergency braking indication
Buzzer	Emergency and APA alerts
Connecting Wires	Component connections
Power Supply	System power
ESP32 Pin Configuration
Component	ESP32 Pin
I2C SDA	GPIO 21
I2C SCL	GPIO 22
Green LED	GPIO 15
Yellow LED	GPIO 5
Red LED	GPIO 18
Buzzer	GPIO 19
GPS RX	GPIO 16
GPS TX	GPIO 17
The firmware communicates with the GPS module through UART2.

Software Requirements
Development Environment
Arduino IDE

ESP32 Board Support

ESP32 Libraries
TinyGPSPlus

MPU6050

WebServer

Preferences

DNSServer

ArduinoJson

Additional Arduino and C++ components include Wire, WiFi, vector, map, and cmath.

Frontend and Dashboard
HTML5

CSS3

JavaScript

Leaflet.js

Font Awesome

SweetAlert2

The dashboard provides pages for the main dashboard, Bantayan Island map, accident-prone areas, Wi-Fi settings, and system information.

Mapping
Leaflet.js

Esri ArcGIS World Dark Gray Base Map

Version Control
GitHub

Brake Classification
BRAKESAFE classifies braking intensity using deceleration values measured by the MPU6050.

Brake Level	Deceleration
Normal	Less than 0.20 g
Soft	0.20 g to less than 0.35 g
Moderate	0.35 g to less than 0.50 g
Hard	0.50 g to less than 0.70 g
Emergency	0.70 g or higher
Emergency braking can trigger additional accident-prone-area processing and an alert.

Accident-Prone Area Detection
BRAKESAFE maintains a collection of accident-prone areas containing:

Latitude

Longitude

Reason

Timestamp

Barangay

Municipality

Province

Direction

Automatic or manual classification

Emergency braking events can automatically contribute to APA detection. Users can also add APA locations manually through the web interface.

The system uses a 15 km detection radius and a 5 km automatic-add radius for APA processing.

Web Dashboard
The ESP32 hosts a local web server that provides access to the BRAKESAFE dashboard.

Dashboard
The dashboard displays:

GPS status

MPU6050 status

Vehicle speed

Current brake level

Emergency status

Latitude

Longitude

GPS time

Deceleration

Recorded event count

Last brake event

Wi-Fi status

The dashboard also provides CSV export options for brake events and APA data.

Map
The map interface displays brake-event locations and provides a visual representation of braking behavior across the monitored area.

Accident-Prone Areas
The APA interface allows users to:

Add an APA location

Provide a reason or description

View existing APA locations

Edit APA information

Delete APA locations

Wi-Fi Operation
BRAKESAFE supports two operating modes.

Station Mode
The ESP32 attempts to connect to a previously configured Wi-Fi network.

Access Point Mode
If no Wi-Fi credentials are configured or the configured network cannot be reached, the ESP32 starts its own access point:

text
BRAKESAFE_AP
The firmware also includes DNS server functionality to support the local access-point interface.

Data Storage
BRAKESAFE is designed for local data processing and storage.

The system stores:

Wi-Fi configuration

Brake event history

Accident-prone areas

System configuration

Persistent data is stored using the ESP32 Preferences system, allowing recorded events and APA information to be restored after restarting the device.

Note: This implementation does not require a cloud database for its core operation.


Installation
1. Install Arduino IDE
Install the latest Arduino IDE on your development computer.

2. Install ESP32 Board Support
Add the ESP32 board package to Arduino IDE through the Board Manager.

Select:

text
ESP32 Dev Module
or the appropriate ESP32 DevKit board being used.

3. Install Required Libraries
Install the following libraries through the Arduino IDE Library Manager:

text
TinyGPSPlus
MPU6050
ArduinoJson
ESP32-specific libraries such as WiFi, WebServer, Preferences, and DNSServer are provided through the ESP32 Arduino environment.

4. Connect the Hardware
Connect the GPS module, MPU6050, LEDs, and buzzer according to the pin configuration described above.

5. Upload the Firmware
Open:

text
firmware/BRAKESAFE.ino
Select the correct ESP32 board and COM port, then upload the firmware.

Running BRAKESAFE
After successfully uploading the firmware:

Power on the ESP32.

The system initializes the GPS module.

The MPU6050 is initialized after GPS module detection.

BRAKESAFE attempts to connect to configured Wi-Fi.

If Wi-Fi is unavailable, the ESP32 starts Access Point mode.

The local web server starts.

Brake data is continuously processed.

Detected brake events are stored locally.

GPS coordinates are associated with recorded events.

The dashboard can be accessed through the ESP32 local network.

The firmware operates using a local-only architecture and does not require cloud uploading for its primary monitoring functions.

Brake Event Data
Each recorded brake event can contain:

text
Latitude
Longitude
Speed
Deceleration
Brake Level
Timestamp
Barangay
Municipality
Province
Direction
This information allows the system to associate braking behavior with geographic locations and support road-safety analysis.

Map Monitoring
BRAKESAFE uses Leaflet.js to visualize collected geographic data.

Brake-event markers can be categorized according to braking intensity:

text
Soft Brake
Moderate Brake
Hard Brake
Emergency Brake
The map interface also provides a dedicated view for accident-prone areas.

Alert System
The system uses LEDs and a buzzer to provide local alerts.

text
Green LED   - Normal
Yellow LED  - Moderate or Hard
Red LED     - Emergency
Buzzer      - Emergency or APA Alert
Emergency braking activates additional processing. Accident-prone-area alerts can activate the buzzer for a defined duration.

System Architecture
BRAKESAFE follows a local IoT architecture.

Sensors include the MPU6050 and GPS module.

The ESP32 performs brake detection, GPS processing, APA detection, local storage, and web server hosting.

The web server provides the local dashboard.

The dashboard includes the brake event map and APA map.

Project Purpose
BRAKESAFE was developed as a capstone project to explore the use of IoT, sensor-based analytics, GPS technology, and web-based visualization for road safety monitoring.

The system aims to provide location-based braking information that can support the identification and monitoring of areas associated with frequent or severe braking events.

Project Information
Field	Details
Project Name	BRAKESAFE
Full Title	IoT-Based Accident-Prone Road Detection and Heatmap Monitoring System Using Brake Force Analytics in Bantayan Island
Platform	ESP32
Development Env.	Arduino IDE
Application Type	IoT and Local Web Dashboard
Deployment Area	Bantayan Island, Cebu, Philippines
Year	2026
Technologies
text
ESP32
Arduino IDE
C/C++
TinyGPSPlus
MPU6050
WebServer
Preferences
DNSServer
ArduinoJson
HTML5
CSS3
JavaScript
Leaflet.js
Font Awesome
SweetAlert2
Esri ArcGIS
GitHub
Disclaimer
BRAKESAFE is a capstone and research prototype intended for experimental and academic use. Sensor readings, GPS accuracy, braking classification, and accident-prone-area detection may be affected by hardware installation, vehicle conditions, GPS availability, sensor calibration, environmental conditions, and other factors.

The system should not be treated as a certified vehicle safety or emergency-response system.

License
This project is developed for academic and research purposes.

If a specific open-source license is selected for the repository, replace this section with the appropriate license information.

Acknowledgments
This project makes use of open-source technologies and libraries including:

Arduino ESP32

TinyGPSPlus

MPU6050 libraries

ArduinoJson

Leaflet.js

Font Awesome

SweetAlert2

BRAKESAFE - Turning Brake Force Data into Road Safety Information.
