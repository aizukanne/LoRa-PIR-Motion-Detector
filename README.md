# LoRa-PIR-Motion-Detector
This is projects provides schematics and PCB design files for a solar powered LoRa IoT Node hardware as well as a collection of arduino sketches for a PIR motion detector using LoRaWAN for data uplinks.

The hardware can be used with a variety of sensors connected to any of the analog I/O pins (A0 - A7) or digital I/O pins (D7 - D10, D14, D15). Please note the following:
<ul>
	<li>D14 and A0 share same physical pin</li>
	<li>D15 and A1 share same physical pin</li>
	<li>LED is connected between D7 and GND through a 1K resistor</li>
</ul>

<p align="center">
  <img src="https://github.com/aizukanne/LoRa-PIR-Motion-Detector/blob/master/Hardware/PCB_Smart-LoRa-Node-PCB.png" alt="24-pin Header"/>
</p>

This projects uses a PIR motion sensor. 
The arduino folder contains two sketches. One for Over the Air Activation (OTAA) and another for Activation by Personalization (ABP). The Keys insert are dummys so ensure you enter the keys from TTN or other LoRaWAN Server as specified. It uses IBM LMIC (LoraMAC-in-C) library for arduino.

This code was developed for LoRa Node running on the ATMEGA328 with the RFM95W LoRa Module. Details of the board design are in the hardware folder. You can modify the sketches to work with any LoRa Node using RFM95W by setting the PIN Mapping correctly in the section. Below is a sample which works with the hardware referenced in the hardware section.

		// Pin mapping
		const lmic_pinmap lmic_pins = {
			.nss = 6,
			.rxtx = LMIC_UNUSED_PIN,
			.rst = 5,
			.dio = {2, 3, LMIC_UNUSED_PIN},
		};

The frequency of updates is set as shown below where the sketch is configured to send an update every 30seconds.

// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 30;

This sketch sends an update containing the status of the motion sensor, GPS coordinates of the device and device ID and Network ID in a concatenated payload. However the device and network ID may be removed to reduce payload size and airtime and devices can be identified using the device ID from the LoRa Server.

If a GPS module is connected, set define GPS to 1 in the line 	#define GPS 0.
		
Hope this is helpful to someone.
