#include "BKD_Spio.h"
Spio spio(40, 41, 38, 39);
void setup() {
  spio.init();
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  spio.onLoop();
  spio.bufferOutput = 0;
  // spio.bufferOutput = spio.bufferInput;
  Serial.print (spio. bufferOutput, BIN); Serial.print(" "); 
  Serial.println(spio.bufferInput, BIN);

  delay(500); 
  spio.onLoop();
  spio.bufferOutput = 0xFF;
  // spio.bufferOutput = spio.bufferInput;
  Serial.print (spio. bufferOutput, BIN); Serial.print(" "); 
  Serial.println(spio.bufferInput, BIN);
   delay(500); 
}

