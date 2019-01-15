# LoRa-PIR-Motion-Detector
This is a collection of arduino sketches for a PIR motion detector using LoRaWAN for data uplinks

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
