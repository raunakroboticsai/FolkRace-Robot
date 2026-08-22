#include <SparkFun_TB6612.h>

#define IMPELLER_PIN 11
#define IMPELLER_PWM 80

const int startButton = 9;

bool l = 0;
bool r = 0;
bool s = 0;
bool u = 0;
bool x = 0;
bool y = 0;
int e = 0;

// custom variable
bool leftOut = 0;
bool rightOut = 0;
int paths = 0;
bool endFound = 0;

int blackValue = 900;
int whiteValue = 100;
int FT = 150;
int P, D, I, previousError, PIDvalue, error;
int lsp = 150;
int rsp = 150;
int lfspeed = 220;
int turnspeed;
float Kp = 0.04;
float Kd = 0.05;
float Ki = 0;

String str;

// TB6612 pins
#define AIN1 2
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

int minValues[8], maxValues[8], threshold[8];

void setup() {
  Serial.begin(9600);
  pinMode(9, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
  pinMode(2, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(13, OUTPUT);

  pinMode(IMPELLER_PIN, OUTPUT);
  analogWrite(IMPELLER_PIN, 0);

  lfspeed = 220;
  turnspeed = 100;
}

void loop() {

  while (digitalRead(9)) {}
  delay(1000);
  calib();

  analogWrite(IMPELLER_PIN, IMPELLER_PWM);

  while (digitalRead(9)) {}
  delay(1000);

  while (endFound == 0) {
    linefollow();
    checknode();
    reposition();
  }

  // RIGHT HAND RULE PATH OPTIMIZATION
  for (int m = 0; m < 4; m++) {
    str.replace("RUR", "S");
    str.replace("SUR", "L");
    str.replace("RUS", "L");
    str.replace("LUR", "U");
  }

  int endpos = str.indexOf('E');

  while (digitalRead(9)) {}
  delay(1000);

  for (int i = 0; i <= endpos; i++) {
    char node = str.charAt(i);
    paths = 0;
    while (paths < 2) {
      linefollow();
      checknode();
      if (paths == 1) {
        reposition();
      }
    }

    switch (node) {
      case 'L':
        botstop();
        delay(50);
        botleft();
        break;

      case 'S':
        break;

      case 'R':
        botstop();
        delay(50);
        botright();
        break;

      case 'E':
        red();
        botstop();
        delay(5000);
        break;
    }
  }
}

void calib() {
  for (int i = 0; i < 5; i++) {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }

  for (int i = 0; i < 2500; i++) {
    motor1.drive(60);
    motor2.drive(-60);

    for (int i = 0; i < 5; i++) {
      if (analogRead(i) < minValues[i]) minValues[i] = analogRead(i);
      if (analogRead(i) > maxValues[i]) maxValues[i] = analogRead(i);
    }
  }

  for (int i = 0; i < 5; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print("   ");
  }
  Serial.println();

  motor1.drive(0);
  motor2.drive(0);
}

void checknode() {
  lightsoff();
  l = 0; r = 0; x = 0; y = 0; s = 0; u = 0; e = 0;
  paths = 0;

  if ((analogRead(0) > threshold[0]) &&
      (analogRead(4) > threshold[4]) &&
      (analogRead(2) > threshold[2])) {
    u = 1;
    botstraight();
    delay(100);
  }

  if (u == 0) {
    for (int i = 0; i < FT; i++) {
      PID();
      if (analogRead(0) < threshold[0]) l = 1;
      if (analogRead(4) < threshold[4]) r = 1;
    }

    if (analogRead(2) < threshold[2]) s = 1;
    if ((analogRead(0) < threshold[3]) &&
        (analogRead(4) < threshold[4]) &&
        (analogRead(2) < threshold[2])) e = 2;
  }

  paths = l + s + r;
}

void red() { digitalWrite(8, HIGH); }
void blue() { digitalWrite(2, HIGH); }

void lightsoff() {
  digitalWrite(2, LOW);
  digitalWrite(8, LOW);
  digitalWrite(13, LOW);
}

void linefollow() {
  paths = 0;
  while ((analogRead(0) > threshold[0]) &&
         (analogRead(4) > threshold[4]) &&
         (analogRead(1) < threshold[1] ||
          analogRead(2) < threshold[2] ||
          analogRead(3) < threshold[3])) {
    PID();
  }
  lightsoff();
}

void PID() {
  int error = analogRead(1) - analogRead(3);

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  if (lsp > 255) lsp = 255;
  if (lsp < 0)   lsp = 0;
  if (rsp > 255) rsp = 255;
  if (rsp < 0)   rsp = 0;

  motor1.drive(rsp);
  motor2.drive(lsp);
}

void reposition() {
  lightsoff();

  if (e == 2) {
    str += 'E';
    endFound = 1;
    motor1.brake();
    motor2.brake();
    delay(80);

    botstop();
    red();
    return;
  }

  // RIGHT HAND RULE PRIORITY
  else if (r == 1) {
    if (paths > 1) str += 'R';
    botright();
  }

  else if (s == 1) {
    if (paths > 1) str += 'S';
  }

  else if (l == 1) {
    if (paths > 1) str += 'L';
    botleft();
  }

  else if (u == 1) {
    str += 'U';
    botuturn();
  }
  lightsoff();
}

void botleft() {
  motor1.drive(-1 * turnspeed);
  motor2.drive(turnspeed);
  delay(150);
  while (analogRead(2) > threshold[2]) {
    motor1.drive(-1 * turnspeed);
    motor2.drive(turnspeed);
  }
  motor1.drive(0);
  motor2.drive(0);
  delay(50);
}

void botright() {
  motor1.drive(turnspeed);
  motor2.drive(-1 * turnspeed);
  delay(250);
  while (analogRead(2) > threshold[2]) {
    motor1.drive(turnspeed);
    motor2.drive(-1 * turnspeed);
  }
  motor1.drive(0);
  motor2.drive(0);
  delay(50);
}

void botstraight() {
  motor1.drive(lfspeed);
  motor2.drive(lfspeed);
}

void botstop() {
  motor1.drive(0);
  motor2.drive(0);
}

void botuturn() {
  motor1.drive(-1 * lfspeed);
  motor2.drive(lfspeed);
  delay(200);
  while (analogRead(2) > threshold[2]) {
    motor1.drive(-1 * turnspeed);
    motor2.drive(turnspeed);
  }
  motor1.drive(0);
  motor2.drive(0);
  delay(50);
}

void forwardstep() {
}
