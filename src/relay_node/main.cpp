#include <Arduino.h>
#include <SoftwareSerial.h>
#include "protocol.h"
#include "ttc.h"

const int hazards = 2;
const int brake = 3;

SoftwareSerial relay(12, 13); // RX, TX

Parser parser;
TTCState ttc;

void setup() {
  relay.begin(9600);
  Serial.begin(9600); // to computer

  ParserInit(&parser); // initialise parser
  ttcInit(&ttc);

  pinMode(hazards, OUTPUT);
  pinMode(brake, OUTPUT);
}

void loop() {
  while (relay.available()){
    uint8_t b = relay.read(); //one byte at a time

    Frame frame;
    if (ParserProcess(&parser, b, &frame)) {

      uint8_t closest = 255;

      for (int i = 0; i < NUM_SENSORS; i++) {

        // only the closest value gets sent as update
        if (frame.distances[i] != 255 && frame.distances[i] < closest) {
          closest = frame.distances[i];
        }

        Serial.print(frame.distances[i]);
        Serial.print(", ");
      }

      AEBStatus status = ttcUpdate(&ttc, closest);

      // drive the outputs: hazards on for either warning level,
      // brake only on a full AEB trigger
      digitalWrite(hazards, (status == BRAKE || status == AEB) ? HIGH : LOW);
      digitalWrite(brake,   (status == AEB) ? HIGH : LOW);

      Serial.println(status);
    }
  }
}
