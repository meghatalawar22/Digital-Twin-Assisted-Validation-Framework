#include <Servo.h>

#define BT Serial1          // HC-05 on Mega pins 18 (TX1) and 19 (RX1)

// ================= IR SENSORS =================
int leftSensor  = A3;
int rightSensor = A2;

// ================= ULTRASONIC =================
#define trigPin 3
#define echoPin 2

// ================= BUZZER =================
#define buzzerPin 4

// ================= MOTOR DRIVER (Mega 2560) =================
int ENA = 11;
int IN1 = 10;
int IN2 = 9;
int IN3 = 8;
int IN4 = 7;
int ENB = 6;

// ================= SERVO =================
#define servoPin 5
Servo myServo;

// ================= SPEEDS =================
int speedA = 60;
int speedB = 60;
int turnSpeed = 90;

// ================= KICK START (after a stop) =================
int kickSpeed = 130;       // speed used to get the car moving again
int kickTime  = 150;       // ms: how long the kick lasts

// ================= DETECTION =================
int obstacleDist = 20;     // cm: car STOPS at or below this distance
int warnDist     = 30;     // cm: buzzer starts beeping slowly here
int resumeDist   = 35;     // cm: path must be clear ABOVE this distance to restart
int clearNeeded  = 6;      // readings in a row that must be clear before restarting

// ================= BUZZER BEEP TUNING =================
int beepSlow   = 600;      // ms between beeps at warnDist (slow)
int beepFast   = 100;      // ms between beeps just before the stop distance (fast)
int beepOnTime = 60;       // ms: length of each beep

unsigned long lastBeep = 0;
bool stopped = false;      // true while the car is stopped by an obstacle
int clearCount = 0;        // consecutive clear readings while stopped

// ================= TELEMETRY =================
// mode codes: 0 FORWARD, 1 TURN RIGHT, 2 TURN LEFT, 3 BOTH BLACK STOP,
//             4 OBSTACLE STOP, 5 LATCHED STOP, 6 KICK OK, 7 KICK ABORTED
int mode = 0;
int outA = 0, outB = 0;    // last motor commands (signed)
unsigned int seq = 0;      // record counter (lets MATLAB count dropped records)

// ================= SETUP =================
void setup()
{
  BT.begin(115200);        // HC-05 set to 115200 with AT+UART=115200,0,0

  pinMode(leftSensor, INPUT);
  pinMode(rightSensor, INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  myServo.attach(servoPin);
  myServo.write(90);       // keep the ultrasonic sensor facing forward

  stopMotors();
  delay(1000);
}

// ================= MAIN LOOP =================
void loop()
{
  int frontDist = getDistance();
  int l = digitalRead(leftSensor);
  int r = digitalRead(rightSensor);

  // controller state at the START of this loop pass (inputs for MATLAB)
  bool stIn  = stopped;
  int  clrIn = clearCount;

  // Buzzer: silent far away, slow beeps -> fast beeps -> solid when stopped
  updateBuzzer(frontDist);

  // ---------- CAR IS STOPPED BY AN OBSTACLE ----------
  if (stopped)
  {
    stopMotors();
    mode = 5;

    // Count readings that are clearly clear (above resumeDist)
    if (frontDist > resumeDist) clearCount++;
    else                        clearCount = 0;

    // Path has been clear for several readings in a row -> restart
    if (clearCount >= clearNeeded)
    {
      bool ok = kickStart();   // careful kick (stops if the object is still there)
      clearCount = 0;

      if (ok)
      {
        stopped = false;
        mode = 6;
      }
      else
      {
        mode = 7;              // object still there: stay stopped
      }
    }

    sendTelemetry(l, r, frontDist, stIn, clrIn);
    return;
  }

  // ---------- OBSTACLE DETECTED: STOP ----------
  if (frontDist <= obstacleDist)
  {
    stopMotors();
    stopped = true;
    clearCount = 0;
    mode = 4;

    sendTelemetry(l, r, frontDist, stIn, clrIn);
    return;
  }

  // ---------- NO OBSTACLE: LANE FOLLOWING ----------
  laneFollow(l, r);
  sendTelemetry(l, r, frontDist, stIn, clrIn);
}

// ================= TELEMETRY (non-blocking) =================
// Record: T,seq,ms,L,R,dist,stIn,clrIn,mode,outA,outB,buzzer,stOut,clrOut
void sendTelemetry(int l, int r, int dist, bool stIn, int clrIn)
{
  char buf[64];
  int n = snprintf(buf, sizeof(buf),
      "T,%u,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
      seq++, millis(), l, r, dist, (int)stIn, clrIn, mode,
      outA, outB, digitalRead(buzzerPin), (int)stopped, clearCount);

  // never block the control loop: if the Bluetooth buffer is full, drop this record
  if (BT.availableForWrite() >= n) BT.write((uint8_t*)buf, n);
}

// ================= BUZZER =================
void updateBuzzer(int dist)
{
  // At or below the stop distance: continuous sound
  if (dist <= obstacleDist)
  {
    digitalWrite(buzzerPin, HIGH);
    return;
  }

  // Warning zone: beep faster as the obstacle gets closer
  if (dist <= warnDist)
  {
    int interval = map(dist, obstacleDist, warnDist, beepFast, beepSlow);
    unsigned long now = millis();

    if (now - lastBeep >= (unsigned long)interval)
    {
      lastBeep = now;
      digitalWrite(buzzerPin, HIGH);          // start a beep
    }
    else if (now - lastBeep >= (unsigned long)beepOnTime)
    {
      digitalWrite(buzzerPin, LOW);           // end the beep
    }
    return;
  }

  // Far away: silent
  digitalWrite(buzzerPin, LOW);
}

// ================= KICK START =================
// Short burst to overcome standing friction.
// Checks the distance while moving. Returns false (and stops) if an obstacle is still there.
bool kickStart()
{
  analogWrite(ENA, kickSpeed);
  analogWrite(ENB, kickSpeed);
  outA = kickSpeed;
  outB = kickSpeed;

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  unsigned long startTime = millis();

  while (millis() - startTime < (unsigned long)kickTime)
  {
    if (getDistance() <= obstacleDist)
    {
      stopMotors();
      return false;
    }
  }

  return true;
}

// ================= LANE FOLLOWING =================
void laneFollow(int left, int right)
{
  // Both sensors on white
  if (left == LOW && right == LOW)
  {
    forward();
    mode = 0;
  }
  // Left sensor sees black line
  else if (left == HIGH && right == LOW)
  {
    turnRight();
    mode = 1;
  }
  // Right sensor sees black line
  else if (left == LOW && right == HIGH)
  {
    turnLeft();
    mode = 2;
  }
  // Both sensors on black
  else
  {
    stopMotors();
    mode = 3;
  }
}

// ================= ULTRASONIC =================
int getDistance()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000);

  if (duration == 0) return 200;

  return duration * 0.034 / 2;
}

// ================= LANE FOLLOWING MOVEMENTS =================
void forward()
{
  analogWrite(ENA, speedA);
  analogWrite(ENB, speedB);
  outA = speedA;
  outB = speedB;

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Sharp Left Turn
void turnLeft()
{
  analogWrite(ENA, turnSpeed);
  analogWrite(ENB, turnSpeed);
  outA = -turnSpeed;
  outB =  turnSpeed;

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Sharp Right Turn
void turnRight()
{
  analogWrite(ENA, turnSpeed);
  analogWrite(ENB, turnSpeed);
  outA =  turnSpeed;
  outB = -turnSpeed;

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// ================= STOP =================
void stopMotors()
{
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  outA = 0;
  outB = 0;

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
