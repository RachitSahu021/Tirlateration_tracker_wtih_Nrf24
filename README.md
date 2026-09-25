# NRF24 Three-Anchor Beacon Tracker

A short-range 2D beacon tracker using:

- Arduino Uno as a transmitting NRF24L01+ beacon
- ESP32 with three NRF24L01+ modules as fixed receiving anchors
- Python/Tkinter GUI for live trilateration display

The Uno transmits the same packet at four NRF24 power levels. Each ESP32 anchor measures packet reception and the NRF24 RPD bit to make a coarse distance estimate. The ESP32 trilaterates the three distances and sends JSON results over USB serial to the Python GUI.

## Repository Files

| File | Purpose |
|---|---|
| `uno_nrfbeacon.ino` | Arduino Uno beacon firmware |
| `esp32_nrf24_receiver.ino` | ESP32 three-anchor receiver and trilateration firmware |
| `beacon_tracker_gui.py` | Live Python tracking GUI |

## System Layout

```text
                 A0
                (0, +28.87 cm)
                    |
                   / \
                  /   \
                 /     \
                /       \
              A1---------A2
     (-25, -14.43)   (+25, -14.43)

            Arduino Uno beacon
The code currently assumes an equilateral triangle with a 50 cm side length.
Your measured sides are approximately 50–51 cm. For best results, measure each side accurately and update the anchor coordinates in both:
- esp32_nrf24_receiver.ino
- beacon_tracker_gui.py
For an approximately equilateral triangle with side length L:
A0 = ( 0,       L / sqrt(3))
A1 = (-L / 2,  -L / (2 * sqrt(3)))
A2 = ( L / 2,  -L / (2 * sqrt(3)))
For L = 50 cm, the coordinates are:
A0 = (  0.00,  28.87)
A1 = (-25.00, -14.43)
A2 = ( 25.00, -14.43)
For L = 51 cm, use approximately:
A0 = (  0.00,  29.45)
A1 = (-25.50, -14.72)
A2 = ( 25.50, -14.72)
Hardware Required
- 1 x Arduino Uno
- 1 x ESP32 development board
- 4 x NRF24L01+ modules
  - 1 for the Uno beacon
  - 3 for the ESP32 anchors
- Stable 3.3 V supply for NRF24 modules
- 10–47 uF capacitor across VCC and GND on every NRF24 module
- 7.6 V nominal Li-ion battery pack
- 5 V buck converter for the ESP32, recommended
- USB cable between ESP32 and PC
Uno Beacon Wiring
NRF24L01+ pin	Arduino Uno pin
GND	GND
VCC	3.3 V only
CE	D9
CSN	D10
SCK	D13
MOSI	D11
MISO	D12
IRQ	Not connected


Do not connect NRF24 VCC to 5 V.
ESP32 Anchor Wiring
Anchor A0
NRF24L01+ pin	ESP32 pin
CE	GPIO 33
CSN	GPIO 32
SCK	GPIO 18
MISO	GPIO 19
MOSI	GPIO 23
VCC	3.3 V
GND	GND


Anchor A1
NRF24L01+ pin	ESP32 pin
CE	GPIO 16
CSN	GPIO 17
SCK	GPIO 18
MISO	GPIO 19
MOSI	GPIO 23
VCC	3.3 V
GND	GND


Anchor A2
NRF24L01+ pin	ESP32 pin
CE	GPIO 26
CSN	GPIO 25
SCK	GPIO 14
MISO	GPIO 12
MOSI	GPIO 13
VCC	3.3 V
GND	GND


All NRF24 modules and controllers must share a common ground.
Power
Use the 7.6 V Li-ion pack as follows:
7.6 V battery
  |
  +-- Arduino Uno VIN or barrel jack
  |
  +-- 5 V buck converter --> ESP32 5V / USB input
  |
  +-- Stable 3.3 V regulator --> NRF24 modules
Do not connect the 7.6 V battery directly to ESP32 5V, ESP32 3V3, or any NRF24 module.
For three NRF24 modules, especially antenna-equipped PA+LNA versions, use an external 3.3 V regulator instead of relying on the ESP32 board's 3.3 V pin.
Arduino Setup
Install these board packages:
- Arduino AVR Boards
- ESP32 by Espressif Systems
Install this library:
- RF24 by TMRh20
Upload:
1. uno_nrfbeacon.ino to the Arduino Uno.
2. esp32_nrf24_receiver.ino to the ESP32.
The beacon and receiver must use the same:
NRF24 channel: 76
Data rate: 250 kbps
Pipe address: RDRBC
Python GUI
Install dependencies:
python -m pip install pyserial matplotlib
Run the GUI using the ESP32 serial port:
python beacon_tracker_gui.py --port COM7 --baud 9600
Replace COM7 with the ESP32 port shown by Arduino IDE or Device Manager.
The ESP32 firmware currently uses:
Serial.begin(9600);
The GUI defaults to 115200, so --baud 9600 is required unless both files are changed to use the same baud rate.
Calibration
The distance thresholds in esp32_nrf24_receiver.ino are environment-dependent:
static const float DIST_THRESH[4] = {
  35.0f,
  70.0f,
  140.0f,
  280.0f
};
Calibrate them in the final deployment area:
1. Place the Uno beacon at known distances from an anchor.
2. Record the strongest reliable PA level and RPD hit rate.
3. Adjust DIST_THRESH.
4. Repeat for all anchors.
5. Keep anchors fixed after calibration.
This is coarse RSSI/RPD-based positioning, not precision UWB ranging. Nearby metal, people, walls, Wi-Fi interference, battery voltage, and antenna orientation can affect the calculated position.
Troubleshooting
Problem	Checks
radio_init_failed	Check NRF24 3.3 V power, ground, CE, CSN, and SPI wiring
No position fix	All three anchors must receive valid packets
Distance shows 500	That anchor has insufficient valid RPD/packet data
ESP32 gets hot	Do not feed the battery directly to ESP32; use a 5 V buck converter
Unstable NRF24 reception	Add capacitors at NRF24 modules and use a stronger 3.3 V regulator
GUI cannot connect	Verify the COM port and use --baud 9600


Notes
- The ESP32 disables Wi-Fi and Bluetooth to reduce 2.4 GHz interference.
- Each anchor listens for 20 ms before the ESP32 switches to the next anchor.
- A complete three-anchor scan takes approximately 60 ms plus processing time.
- The GUI stores and displays the latest 200 valid smoothed positions.

One small code fix I recommend before committing: remove the `Serial.println("READY,esp32_receiver");` inside the ESP32 `loop()`. It prints continuously and can flood the serial connection.
