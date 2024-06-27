#include <L298N.h>

int arr[8] = { A1, A3, 8, 5, 6, A5, 4, A0 };
int arr2[8];
int I = 0;
int lastError = 0;
float Kp = 0.2; // Increase Kp for more responsiveness
float Ki = 0.0008;
float Kd = 0.8; // Adjust Kd to reduce oscillations
int x = 120;
int y = 140;
int basespeeda = x;
int basespeedb = x;
uint8_t multiP = 1;
uint8_t multiI = 1;
uint8_t multiD = 1;
boolean onoff = 1;
int val, cnt = 0, v[3];
L298N motor1(11, 2, 3); // right
L298N motor2(10, 13, 12); // left

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 8; i++) {
    pinMode(arr[i], INPUT);
  }
}

bool onesBetweenZeros(int arr[], int size) {
  bool zeroFound = false;
  int onesCount = 0;

  for (int i = 0; i < size - 1; i++) {
    if (arr[i] == 0 && !zeroFound) {
      zeroFound = true;
    } else if (zeroFound && arr[i] == 1) {
      onesCount++;
    } else if (zeroFound && onesCount > 0 && arr[i] == 0) {
      return true;
    }
  }
  return false;
}

bool detectPatterns(int arr[]) {
  for (int i = 0; i < 8; i++) {
    if (i <= 5 && arr[i] == 0 && arr[i+1] == 1 && arr[i+2] == 0) {
      return true;
    }
    if (i <= 4 && arr[i] == 0 && arr[i+1] == 1 && arr[i+2] == 1 && arr[i+3] == 0) {
      return true;
    }
    if (i <= 3 && arr[i] == 0 && arr[i+1] == 1 && arr[i+2] == 1 && arr[i+3] == 1 && arr[i+4] == 0) {
      return true;
    }
    if (i <= 2 && arr[i] == 0 && arr[i+1] == 1 && arr[i+2] == 1 && arr[i+3] == 1 && arr[i+4] == 1 && arr[i+5] == 0) {
      return true;
    }
    if (i <= 1 && arr[i] == 0 && arr[i+1] == 1 && arr[i+2] == 1 && arr[i+3] == 1 && arr[i+4] == 1 && arr[i+5] == 1 && arr[i+6] == 0) {
      return true;
    }
    if (i <= 0 && arr[i] == 0 && arr[i+1] == 1 && arr[i+2] == 1 && arr[i+3] == 1 && arr[i+4] == 1 && arr[i+5] == 1 && arr[i+6] == 1 && arr[i+7] == 0) {
      return true;
    }
  }
  return false; // Pattern not found
}

void loop() {
  if (Serial.available()) {
    while (Serial.available() == 0);
    valuesread();
    processing();
  }
  if (onoff) {
    int s = 0, c = 0;
    for (int i = 0; i < 8; i++) {
      arr2[i] = digitalRead(arr[i]);
      if (arr2[i] == 0) {
        s += (i * 1000);
        c++;
      }
    }
    for (int i = 0; i < 8; i++) {
      Serial.print(arr2[i]);
    }
    if (onesBetweenZeros(arr2, 8)) {
      motor1.stop();
      motor2.stop();
    } else {
      Serial.println();
      if (c != 0) {
        Serial.println(s / c);
      } else {
        Serial.println("No black");
      }
      if (c != 0) {
        s /= c;
        int error = 3500 - s;
        int P = error;
        I += error;
        int D = error - lastError;
        lastError = error;
        float motorspeed = ((Kp / pow(10, multiP)) * P + (Ki / pow(10, multiI)) * I + (Kd / pow(10, multiD)) * D);
        int motorspeeda = constrain(basespeeda - motorspeed, 0, 255);
        int motorspeedb = constrain(basespeedb + motorspeed, 0, 255);
        Serial.print(error);
        Serial.print(" ");
        Serial.print(motorspeeda);
        Serial.print(" ");
        Serial.println(motorspeedb);
        motor2.setSpeed(motorspeedb);
        motor2.forward();
        motor1.setSpeed(motorspeeda);
        motor1.forward();
      } else {
        if (lastError > 0) {
          motor2.stop();
          motor1.setSpeed(y);
          motor1.backward();
        } else if (lastError < 0) {
          motor1.stop();
          motor2.setSpeed(y);
          motor2.backward();
        } else {
          motor1.stop();
          motor2.stop();
        }
      }
    }
  } else {
    motor1.stop();
    motor2.stop();
  }
}

void valuesread() {
  val = Serial.read();
  cnt++;
  v[cnt] = val;
  if (cnt == 2) cnt = 0;
}

void processing() {
  int a = v[1];
  if (a == 1) Kp = v[2];
  if (a == 2) multiP = v[2];
  if (a == 3) {
    x = v[2];
    basespeeda = basespeedb = v[2];
  }
  if (a == 4) {
    // multiI = v[2];
  }
  if (a == 5) Kd = v[2];
  if (a == 6) multiD = v[2];
  if (a == 7) onoff = v[2];
}
