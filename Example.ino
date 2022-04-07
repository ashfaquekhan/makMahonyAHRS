#include "makMahony.h"
MAKMAHONY test;
float roll,pitch,yaw;
void setup() {
    Serial.begin(115200);
    test.MAKMAHONY_begin();
}

void loop() {
  test.Serial_roll_pitch_yaw(roll,pitch,yaw);
  Serial.print(roll); Serial.print(",");
  Serial.print(pitch); Serial.print(",");
  Serial.println(yaw);
}
