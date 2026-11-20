# Deneyap line sensor firmware

`deneyap_line.ino` reads the sensor's eight analog outputs on A0–A7 and sends the
millivolts to the T3 Gemstone O1 over UART about 50 times per second. The same JSON lines
are also visible in the USB serial monitor. It does no motor control; the black/white
calibration is stored on the T3 Gemstone O1.

## Uploading to the Mini v2 with Deneyap Kart IDE

1. Install **Deneyap Kart IDE** on your computer. On first launch, wait for the downloads
   on the **Preparing Development Environment** screen to finish.
   [Deneyap Kart IDE installation guide](https://docs.deneyapkart.org/en/IDE/Deneyap-Kart-IDE-Setup)
2. Switch off the power before changing connections. Disconnect the 5 V, TX and
   RX wires coming from the T3 Gemstone O1. Connect the Mini v2 to the computer with a
   data-capable cable that fits the board's USB socket.
3. Open the [`deneyap_line.ino`](deneyap_line.ino) file via **Settings → Open File**.
   Select the **Deneyap Mini v2** board in the top toolbar.
4. In the **Tools** menu, set **Upload Mode** to **Internal USB** and **USB CDC On
   Boot** to **Enabled**. Select the Mini v2's port and click **Compile and
   Upload**.
   [Deneyap Kart IDE interface guide](https://docs.deneyapkart.org/en/IDE/Deneyap-Kart-IDE-Introduction)
5. If the port doesn't appear, hold down the **BUT** button, press and release **RES**,
   then release **BUT**. Click the **Refresh Ports** button, select the port that
   appears in upload mode and try again.
   [Deneyap Mini v2 troubleshooting guide](https://deneyapkart.org/docs/sorun-giderme-dymv2)
6. When the upload is complete, select the board's port and a speed of **115200 baud**
   in the **Serial Monitor** tab and click **Connect**. The expected output is a JSON
   line like the one below.

   ```json
   {"ugv_adc":1,"seq":123,"mv":[1200,1210,1220,1230,1240,1250,1260,1270]}
   ```

The values in the example are illustrative; the real values should change between black and white surfaces.
If the firmware doesn't start after uploading, press RESET once.

## Mini v2 connections

Switch off the power to both boards before changing wires. Connect the sensor's
**VCC pin to 3V3** and solder the **3.3V** jumper on the board.

| Sensor | Mini v2 | GPIO |
|---|---|---|
| D1 | A0 | 7 |
| D2 | A1 | 8 |
| D3 | A2 | 9 |
| D4 | A3 | 10 |
| D5 | A4 | 11 |
| D6 | A5 | 12 |
| D7 | A6 | 13 |
| D8 | A7 | 16 |
| VCC and IR | 3V3 | — |
| GND | GND | — |

| Mini v2 | T3 Gemstone O1 |
|---|---|
| TX / D1 (GPIO43) | Physical pin 10, GPIO15, RX |
| RX / D0 (GPIO44) | Physical pin 8, GPIO14, TX |
| GND | Physical pin 6, GND |
| 5V | T3 Gemstone O1 5 V (physical pin 2 in the diagram) |

The sensor's **D1–D8** outputs go to the Mini's **A0–A7** inputs; the Mini's D1 pin
is not a sensor input here but the UART TX pin going to the T3 Gemstone O1.
While the robot is running, the Mini v2 is powered from the T3 Gemstone O1's 5 V line and sends
its readings over UART. Disconnect the 5 V line coming from the T3 Gemstone O1 before
programming over USB.

On the T3 Gemstone O1, `type: serial`, `serial.port: /dev/ttyS3`, `serial.baudrate: 115200`
must be set. In the UI, follow the **Line sensor calibration → Start calibration → White
ready → Black ready** steps. First check that the raw mV values change with the
surface; calibration can't fix a wiring/pin error.

## Arduino CLI

Verified build target for the Mini v2:

```sh
arduino-cli compile --fqbn deneyap:esp32:deneyapminiv2:CDCOnBoot=cdc,UploadMode=default firmware/deneyap_line
```

The Kart 1A v2 can use the same source code; it must be rebuilt with **Deneyap Kart 1A v2**
selected and wired according to that board's A0–A7/TX/RX labels.
The pin definitions come from the board package; don't change the GPIO numbers in the source by hand.
