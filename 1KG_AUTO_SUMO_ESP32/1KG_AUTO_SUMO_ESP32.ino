/*dipswitch Reading
	---------------------------------------------------------
	| SW1 | SW2 | SW3 |  Analog Value  | Tactics   |  Return
	---------------------------------------------------------
	|  0  |  0  |  0  |     > 933      | Default   |	0
	---------------------------------------------------------
	|  0  |  0  |  1  |     > 773      | 90 Right  |	1
	---------------------------------------------------------
	|  0  |  1  |  0  |     > 658      | Front     |	2
	---------------------------------------------------------
	|  0  |  1  |  1  |     > 563      | Moon Right|	3
	---------------------------------------------------------
	|  1  |  0  |  0  |     > 487      | 90 Left   |	4
	---------------------------------------------------------
	|  1  |  0  |  1  |     > 440      | 180 Turn  |	5
	---------------------------------------------------------
	|  1  |  1  |  0  |     > 400      | Moon Left |	6
	---------------------------------------------------------
	|  1  |  1  |  1  |     < 400      | 180 Turn  |	7
	---------------------------------------------------------
  
	
	SW: 0 = OFF    1 = ON

  Sensor Kotak A:
  blue = -
  chocolate = +
  black = signal
	
*/

#include <EEPROM.h>

// EEPROM Address.
#define EEADD_EDGE_L 0
#define EEADD_EDGE_R (EEADD_EDGE_L + sizeof(int))

// Default value for edge sensors threshold if not calibrated.
#define DEFAULT_EDGE_THRESHOLD 650
int EDGE_L_THRESHOLD, EDGE_R_THRESHOLD;
// Time window for entering edge line calibration mode
#define CALIBRATION_DELAY 500
// Variables
#define LEFT 0
#define RIGHT 1
int duration = 5;  // 5 SECOND start delay
int searchDir = LEFT;
int IdleSpeed = 50;
int MaxSpeed = 80;
//Pin Assignment modified
int LPWM_1 = 18;   // Left motor forward
int LPWM_2 = 16;   // Left motor backward
int RPWM_1 = 19;  // Right motor forward
int RPWM_2 = 17;  // Right motor backward

int REdge = A1;  // Right Line Sensor
int LEdge = A0;  // Left Line Sensor

int LSens = 13;  // Left opponent sensor
int LFSens = 7;  // Left front opponent sensor
int MSens = 6;   // Middle opponent sensor
int RFSens = 5;  // Right front opponent sensor
int RSens = 12;  // Right opponent Sensor

int start = 2;   // Start button
int OB_LED = 2;
//int trimpot = A5;    // trimpot pin
int dipswitch = A6;  // Tactic dipswitch 3
//int voltage = A7;    // battery voltage reading

void setup() {
  Serial.begin(9600);  // Computer & Arduino Interface started in 9600 bits per second, We need to write for usb communication debug.
  Serial.println("Freetronics 1KG MiniSumo");
  //Opponent Sensor Connection
  pinMode(LSens, INPUT_PULLUP);
  pinMode(LFSens, INPUT_PULLUP);
  pinMode(MSens, INPUT_PULLUP);
  pinMode(RFSens, INPUT_PULLUP);
  pinMode(RSens, INPUT_PULLUP);
  
  //Line Sensor Connection
  pinMode(LEdge, INPUT);
  pinMode(REdge, INPUT);
  //Motor Connection
  PinMode(LPWM_1, OUTPUT);
  pinMode(LPWM_2, OUTPUT);
  pinMode(RPWM_1, OUTPUT);
  pinMode(RPWM_2, OUTPUT);
  
  //Board connection
  pinMode(OB_LED, OUTPUT);
  pinMode(dipswitch, INPUT);  // tactic switch input
  pinMode(start, INPUT_PULLUP);      //start button input
  //pinMode(voltage, INPUT);

  analogWrite(RPWM_1, 0);
  analogWrite(RPWM_2, 0);
  analogWrite(LPWM_1, 0);
  analogWrite(LPWM_1, 0);

  while (millis() < CALIBRATION_DELAY) {
    digitalWrite(OB_LED, HIGH);
    Serial.println("Press Start for Calibration");
    if (!digitalRead(start)) {
      Serial.println("Calibrating starts in 3");
      digitalWrite(OB_LED, HIGH);
      delay(500);
      digitalWrite(OB_LED, LOW);
      delay(500);
      Serial.println("Calibrating starts in 2");
      digitalWrite(OB_LED, HIGH);
      delay(500);
      digitalWrite(OB_LED, LOW);
      delay(500);
      Serial.println("Calibrating starts in 1");
      digitalWrite(OB_LED, HIGH);
      delay(500);
      digitalWrite(OB_LED, LOW);
      calibrateEdgeSensor();
    }
  }
  digitalWrite(OB_LED, LOW);
  EDGE_L_THRESHOLD = sensorThreshold(LEFT);
  EDGE_R_THRESHOLD = sensorThreshold(RIGHT);


  startRoutine();
}

void loop() {
  //IdleSpeed = map(analogRead(trimpot), 0, 1023, 0, 100);
  //MaxSpeed = 1.6 * map(analogRead(trimpot), 0, 1023, 0, 100);
  if (analogRead(LEdge) < EDGE_L_THRESHOLD && analogRead(REdge) > EDGE_R_THRESHOLD) {
    delay(20);
    if (analogRead(LEdge) < EDGE_L_THRESHOLD && analogRead(REdge) > EDGE_R_THRESHOLD) {
      backoff(RIGHT);
      searchDir ^= 1;
    }
  } else if (analogRead(REdge) < EDGE_R_THRESHOLD && analogRead(LEdge) > EDGE_L_THRESHOLD) {
    delay(20);
    if (analogRead(REdge) < EDGE_R_THRESHOLD && analogRead(LEdge) > EDGE_L_THRESHOLD) {
      backoff(LEFT);
      searchDir ^= 1;
    }
  } else {
    if (digitalRead(LSens) && digitalRead(MSens) && digitalRead(RSens) && digitalRead(RFSens) && digitalRead(LFSens)) {
      search();
    } else {  //kalau sensor detected call function attack
      attack();
    }
  }
  if (!digitalRead(start)) {
    Serial.println("Sumo Stop");
    stopmotor();
    delay(1000);
    startRoutine();
  }
}

/*******************************************************************************
	Start Routine
	This function should be called once only when the game start.
*******************************************************************************/
void startRoutine() {
  //delay(500);
  while (digitalRead(start)) {
    sensordebug();
    if (analogRead(LEdge) < 500 || analogRead(REdge) < 500) {
      digitalWrite(OB_LED, HIGH);
    } else {
      digitalWrite(OB_LED, LOW);
    }
    delay(20);
  }
  if (!digitalRead(start))
  // What's here ??
    ;
  Serial.println("Start Sumo");

  for (int i = 1; i <= duration; i++) {  //set delay time
    digitalWrite(OB_LED, HIGH);
    delay(700);
    digitalWrite(OB_LED, LOW);
    delay(300);
  }

  uint32_t startTimestamp = millis();

  if (readDipSwitch() == 0) {  // forward attack
    Set_Motor(50, 50);
    delay(30);
    stopmotor();
  }

  else if (readDipSwitch() == 1) {  //90 right
    Set_Motor(60, -60);
    delay(100);
    Set_Motor(-10, 10);
    delay(15);
    stopmotor();
    while (digitalRead(MSens)) {
      // Quit if opponent is not found after timeout.
      if (millis() - startTimestamp > 180) {
        break;
      }
    }
  }

  else if (readDipSwitch() == 2) {  //forward atttack
    Set_Motor(50, 50);
    delay(30);
    stopmotor();
  }

  else if (readDipSwitch() == 3) {  //moon right
    Set_Motor(60, -60);
    delay(100);
    Set_Motor(-10, 10);
    delay(15);
    Set_Motor(50, 70);
    delay(1100);
    Set_Motor(-5, -5);
    delay(10);
    Set_Motor(-60, 70);
    delay(180);
    Set_Motor(10, -10);
    delay(15);
    stopmotor();
    while (digitalRead(LSens)) {
      // Quit if opponent is not found after timeout.
      if (millis() - startTimestamp > 770) {
        break;
      }
    }
  }

  else if (readDipSwitch() == 4) {  //90 left
    Set_Motor(-60, 60);
    delay(100);
    Set_Motor(10, -10);
    delay(15);
    stopmotor();
    while (digitalRead(MSens)) {
      // Quit if opponent is not found after timeout.
      if (millis() - startTimestamp > 180) {
        break;
      }
    }
  }

  else if (readDipSwitch() == 5) {  //180 turn
    Set_Motor(-70, 70);
    delay(200);
    stopmotor();
    while (digitalRead(MSens)) {
      // Quit if opponent is not found after timeout.
      if (millis() - startTimestamp > 250) {
        break;
      }
    }
  }

  else if (readDipSwitch() == 6) {  //moon left
    Set_Motor(-60, 60);
    delay(100);
    Set_Motor(10, -10);
    delay(15);
    Set_Motor(70, 50);
    delay(1100);
    Set_Motor(-5, -5);
    delay(10);
    Set_Motor(70, -60);
    delay(180);
    Set_Motor(-10, 10);
    delay(15);
    stopmotor();
    while (digitalRead(RSens) || digitalRead(MSens)) {
      // Quit if opponent is not found after timeout.
      if (millis() - startTimestamp > 770) {
        break;
      }
    }
  }

  else if (readDipSwitch() == 7) {  //forward and wait
    Set_Motor(65, 65);
    delay(180);
    Set_Motor(-5, -5);
    delay(10);
    stopmotor();
    //delay(2000);
    while (digitalRead(LSens) || digitalRead(LFSens) || digitalRead(MSens) || digitalRead(RFSens) || digitalRead(LSens)) {
      // Quit if opponent is not found after timeout.
      if (!digitalRead(LSens) || !digitalRead(LFSens) || !digitalRead(MSens) || !digitalRead(RFSens) || !digitalRead(RSens)) {
        break;
      }
      if (millis() - startTimestamp > 3000) {
        break;
      }
    }
  }
}

/*******************************************************************************
	Attack
	Track and attack the opponent in full speed.
	Do nothing if opponent is not found.
*******************************************************************************/
void attack() {
  uint32_t attackTimestamp = millis();
  // Opponent in front center.
  // Go straight in full speed.
  if (!digitalRead(MSens)) {
    Set_Motor(75, 75);
    delay(50);
  }
  // Opponent in front left.
  // Turn left.
  else if (!digitalRead(RFSens)) {
    Set_Motor(75, -75);
    delay(20);
    while (digitalRead(MSens)) {
      // Quit if opponent is not found after timeout.
      if (millis() - attackTimestamp > 200) {
        break;
      }
    }
  }
  // Opponent in front right.
  // Turn right.
  else if (!digitalRead(LFSens)) {
    Set_Motor(-75, 75);
    delay(20);
    while (digitalRead(MSens)) {
      // Quit if opponent is not found after timeout.
      if (millis() - attackTimestamp > 200) {
        break;
      }
    }
  }
  // All Front Sensor detect opponent
  else if (!digitalRead(LFSens) && !digitalRead(RFSens) && !digitalRead(MSens)) {
    Set_Motor(100, 100);
    delay(50);
  }
  // Opponent in left side.
  // Rotate left until opponent is in front.
  else if (!digitalRead(LSens)) {
    Set_Motor(-75, 75);
    delay(80);
    while (digitalRead(MSens)) {
      // Quit if opponent is not found after timeout.
      if (millis() - attackTimestamp > 240) {
        break;
      }
    }
  }
  // Opponent in right side.
  // Rotate right until opponent is in front.
  else if (!digitalRead(RSens)) {
    Set_Motor(75, -75);
    delay(80);
    while (digitalRead(MSens)) {
      // Quit if opponent is not found after timeout.
      if (millis() - attackTimestamp > 240) {
        break;
      }
    }
  }
}

/*******************************************************************************
	Search
*******************************************************************************/
void search() {

  if (searchDir == LEFT) {
    Set_Motor(IdleSpeed, IdleSpeed);
  } else {
    Set_Motor(IdleSpeed, IdleSpeed);
  }
  delay(10);
}

/*******************************************************************************
	Back Off
	This function should be called when the ring edge is detected.
*******************************************************************************/
void backoff(uint8_t dir) {

  // Reverse.
  Set_Motor(-75, -75);
  delay(175);

  // Rotate..
  if (dir == LEFT) {
    Set_Motor(-50, 50);
  } else {
    Set_Motor(50, -50);
  }
  delay(250);

  // Start looking for opponent.
  // Timeout after a short period.
  uint32_t uTurnTimestamp = millis();
  while (millis() - uTurnTimestamp < 75) {
    // Opponent is detected if either one of the opponent sensor is triggered.
    if (!digitalRead(LSens) || !digitalRead(LFSens) || !digitalRead(MSens) || !digitalRead(RFSens) || !digitalRead(RSens)) {
      // Stop the motors.
      stopmotor();
      // Return to the main loop and run the attach program.
      return;
    }
  }
  // If opponent is not found, move forward and continue searching in the main loop..
  Set_Motor(IdleSpeed, IdleSpeed);
}

int readDipSwitch() {
  int adc = analogRead(dipswitch);
  if (adc > 933) return 0;
  if (adc > 773) return 1;
  if (adc > 658) return 2;
  if (adc > 563) return 3;
  if (adc > 487) return 4;
  if (adc > 440) return 5;
  if (adc > 400) return 6;
  return 7;
}

float readBatteryVoltage() {
  int adc = analogRead(voltage);
  float adcvolt = (float)adc * (5.0f / 1023.0f);
  float vbatt = adcvolt * (1.0f + (10.0f / 3.9f));
  return vbatt;
}

void calibrateEdgeSensor() {
  int minL = 1024;
  int minR = 1024;
  int maxL = 0;
  int maxR = 0;
  uint32_t timestamp = millis();

  do {
    int tempL = analogRead(LEdge);
    int tempR = analogRead(REdge);

    if (minL > tempL) minL = tempL;
    if (minR > tempR) minR = tempR;

    if (maxL < tempL) maxL = tempL;
    if (maxR < tempR) maxR = tempR;

    if (millis() - timestamp >= 200) {
      timestamp += 100;
      Serial.println("Calibrating...");
      digitalWrite(OB_LED, !digitalRead(OB_LED));
      tone(buzzer, 523, 100);
      noTone(buzzer);
    }
  } while (digitalRead(start));
  while (!digitalRead(start))
    ;
  Serial.println("DONE CALIBRATE wait for values");
  digitalWrite(OB_LED, HIGH);
  tone(buzzer, 523, 500);
  delay(500);
  digitalWrite(OB_LED, LOW);
  noTone(buzzer);
  delay(500);
  digitalWrite(OB_LED, HIGH);
  tone(buzzer, 125, 500);
  delay(500);
  digitalWrite(OB_LED, LOW);
  noTone(buzzer);
  delay(500);
  digitalWrite(OB_LED, HIGH);
  tone(buzzer, 750, 500);
  delay(500);
  digitalWrite(OB_LED, LOW);
  noTone(buzzer);
  delay(500);
  if (maxL > minL) {
    int threshold = ((maxL - minL) * 3 / 5) + minL;
    EEPROM.writeInt(EEADD_EDGE_L, threshold);
  }
  if (maxR > minR) {
    int threshold = ((maxR - minR) * 3 / 5) + minR;
    EEPROM.writeInt(EEADD_EDGE_R, threshold);
  }
  EEPROM.commit();
  Serial.println("NEW CALIBRATED VALUE");
  Serial.print("Left: ");
  Serial.print(sensorThreshold(LEFT));
  Serial.print(" | ");
  Serial.print("Right: ");
  Serial.println(sensorThreshold(RIGHT));
}

int sensorThreshold(int side) {
  int eepromAddress;
  if (side == LEFT) {
    eepromAddress = EEADD_EDGE_L;
  } else if (side == RIGHT) {
    eepromAddress = EEADD_EDGE_R;
  } else {
    return 0;
  }
  int threshold;
  EEPROM.readInt(eepromAddress, threshold);

  if ((threshold <= 0) || (threshold >= 1023)) {
    threshold = DEFAULT_EDGE_THRESHOLD;
  }

  return threshold;
}

void Set_Motor(float Lval, float Rval) {
  Lval = Lval * 2.55;
  Rval = Rval * 2.55;
  Lval = constrain(Lval, -255, 255);
  Rval = constrain(Rval, -255, 255);
  if (Lval >= 0) {
    analogWrite(LPWM_1, Lval);
    digitalWrite(LPWM_2, LOW);
  } else {
    Lval = abs(Lval);
    analogWrite(LPWM_2, Lval);
    digitalWrite(LPWM_1, LOW);
  }
  if (Rval >= 0) {
    analogWrite(RPWM_1, Rval);
    digitalWrite(RPWM_2, LOW);
  } else {
    Rval = abs(Rval);
    analogWrite(RPWM_2, Rval);
    digitalWrite(RPWM_1, LOW);
  }
}

void stopmotor() {
  analogWrite(LPWM_1, 0);
  analogWrite(LPWM_2, 0);
  analogWrite(RPWM_1, 0);
  analogWrite(RPWM_2, 0);
}

void sensordebug() {
  //int speed = analogRead(trimpot);
  int tactic = analogRead(dipswitch);
  Serial.print("Start Button : ");
  Serial.print(digitalRead(start));
  Serial.print("   ");
  Serial.print("Dipswitch : ");
  Serial.print(readDipSwitch());
  Serial.print(" | ");
  Serial.print(tactic);
  Serial.print("\t");
  //Serial.print("Trimpot : ");
  //Serial.print(speed);
  //Serial.print("\t");
  //Serial.print("Voltage : ");
  //Serial.print(readBatteryVoltage());
  //Serial.print("\t");
  Serial.print("Line Sensor : ");
  Serial.print(analogRead(LEdge));
  Serial.print(" | ");
  Serial.print(analogRead(REdge));
  Serial.print("\t");
  Serial.print("Opponent Sensor : ");
  Serial.print(digitalRead(LSens));
  Serial.print(digitalRead(LFSens));
  Serial.print(digitalRead(MSens));
  Serial.print(digitalRead(RFSens));
  Serial.println(digitalRead(RSens));
}
