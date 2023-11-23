#include <TinyUSB_Mouse_and_Keyboard.h>
#include <OneButton.h>
#include <Tlv493d.h>
#include <SimpleKalmanFilter.h>

Tlv493d mag = Tlv493d();
SimpleKalmanFilter xFilter(1, 1, 0.2), yFilter(1, 1, 0.2), zFilter(1, 1, 0.2);

// Setup buttons
OneButton button1(27, true);
OneButton button2(24, true);

float xOffset = 0, yOffset = 0, zOffset = 0;
float xCurrent = 0, yCurrent = 0, zCurrent = 0;

int calSamples = 300;
int sensivity = 8;
int magRange = 3;
int outRange = 127;
float xyThreshold = 0.4;

int inRange = magRange * sensivity;
float zThreshold = xyThreshold * 1.5;

bool isOrbit = false;

void setup() {
  button1.attachClick(goHome);
  button1.attachLongPressStop(goHome);

  button2.attachClick(fitToScreen);
  button2.attachLongPressStop(fitToScreen);

  Mouse.begin();
  Keyboard.begin();

  Serial.begin(9600);
  Wire1.begin();

  mag.begin(Wire1);
  mag.setAccessMode(mag.MASTERCONTROLLEDMODE);
  mag.disableTemp();

  for (int i = 1; i <= calSamples; i++) {
    delay(mag.getMeasurementDelay());
    mag.updateData();

    xOffset += mag.getX();
    yOffset += mag.getY();
    zOffset += mag.getZ();

    Serial.print(".");
  }

  xOffset = xOffset / calSamples;
  yOffset = yOffset / calSamples;
  zOffset = zOffset / calSamples;

  Serial.println();
  Serial.println(xOffset);
  Serial.println(yOffset);
  Serial.println(zOffset);
}

void loop() {
  button1.tick();
  button2.tick();

  delay(mag.getMeasurementDelay());
  mag.updateData();

  // Adjusted for 90-degree counterclockwise rotation
  yCurrent = xFilter.updateEstimate(mag.getX() - xOffset);
  xCurrent = -yFilter.updateEstimate(mag.getY() - yOffset); // Inverted y-axis
  zCurrent = zFilter.updateEstimate(mag.getZ() - zOffset);

  if (abs(xCurrent) > xyThreshold || abs(yCurrent) > xyThreshold) {
    int xMove = map(xCurrent, -inRange, inRange, -outRange, outRange);
    int yMove = map(yCurrent, -inRange, inRange, -outRange, outRange);

    if (abs(zCurrent) < zThreshold && !isOrbit) {
      Keyboard.press(KEY_LEFT_SHIFT);
      isOrbit = true;
    }

    Mouse.press(MOUSE_MIDDLE);
    Mouse.move(yMove, xMove, 0);
  } else {
    Mouse.release(MOUSE_MIDDLE);
    if (isOrbit) {
      Keyboard.release(KEY_LEFT_SHIFT);
      isOrbit = false;
    }
  }

  Keyboard.releaseAll();

  Serial.print(xCurrent);
  Serial.print(",");
  Serial.print(yCurrent);
  Serial.print(",");
  Serial.print(zCurrent);
  Serial.println();
}

void goHome() {
  Keyboard.press(KEY_LEFT_GUI);
  Keyboard.press(KEY_LEFT_SHIFT);
  Keyboard.write('h');

  delay(10);
  Keyboard.releaseAll();
  Serial.println("pressed home");
}

void fitToScreen() {
  Mouse.press(MOUSE_MIDDLE);
  Mouse.release(MOUSE_MIDDLE);
  Mouse.press(MOUSE_MIDDLE);
  Mouse.release(MOUSE_MIDDLE);

  Serial.println("pressed fit");
}
