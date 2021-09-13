#include <Servo.h>
#include <Adafruit_DotStar.h>

Servo servo;
Adafruit_DotStar rgb = Adafruit_DotStar(1, INTERNAL_DS_DATA, INTERNAL_DS_CLK, DOTSTAR_BGR);

//hitec HS-55
int off = 168;
int on = 60;

//KST X08HP
//int off = 140;
//int on = 60;

void setup() {
  Serial.begin(9600);
  servo.attach(1);
  servo.write(on);
  rgb.setPixelColor(0, 0, 64, 0); rgb.show();
}

void loop() {
  if (Serial.available() > 0) {
    int incoming = Serial.read();
    if (incoming == '0' || incoming == 0x00) {
      servo.write(off);
      rgb.setPixelColor(0, 64, 0, 0); rgb.show();
    } else if (incoming == '1' || incoming == 0x01) {
      servo.write(on);
      rgb.setPixelColor(0, 0, 64, 0); rgb.show();
    }
  }
}

