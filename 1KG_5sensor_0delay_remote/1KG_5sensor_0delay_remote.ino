/*dipswitch Reading
	---------------------------------------------------------
	| SW1 | SW2 | SW3 |  Analog Value  | Tactics   |  Return
	---------------------------------------------------------
	|  0  |  0  |  0  |     > 933      | Default   |  0
	---------------------------------------------------------
	|  0  |  0  |  1  |     > 773      | 90 Right  |  1
	---------------------------------------------------------
	|  0  |  1  |  0  |     > 658      | Front     |  2
	---------------------------------------------------------
	|  0  |  1  |  1  |     > 563      | Moon Right|  3
	---------------------------------------------------------
	|  1  |  0  |  0  |     > 487      | 90 Left   |  4
	---------------------------------------------------------
	|  1  |  0  |  1  |     > 440      | 180 Turn  |  5
	---------------------------------------------------------
	|  1  |  1  |  0  |     > 400      | Moon Left |  6
	---------------------------------------------------------
	|  1  |  1  |  1  |     < 400      | 180 Turn  |  7
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
int EDGE_L = 300, EDGE_R = 300;
#define CALIBRATION_DELAY 300
// Variables
#define LEFT 0
#define RIGHT 1
int delay_enable = false; // true to use start delay, vice-versa
int duration = 5;  //5 SECOND delay
int searchDir = LEFT;
int IdleSpeed = 50;
int MaxSpeed = 80;
//Pin Assignment modified
int LPwm = 3;   //left side speed
int LDir = 4;   //left side direction
int RPwm = 11;  //right side speed
int RDir = 10;  //right side direction

int REdge = A1;  // Right Line Sensor
int LEdge = A0;  // Left Line Sensor

int RSens = 13;  // right opponent Sensor
int RFSens = 7;  // right_Middel opponent Sensor
int MSens = 6;   // Middel opponent Sensor
int LFSens = 5;  // Left_Middel opponent Sensor
int LSens = 12;  // Left opponent Sensor

int start = A2;  // start start
int buzzer = 8;  // buzzer
int led = A4;
int trimpot = A5;    // trimpot pin
int dipswitch = A6;  // Tactic dipswitch 3
int voltage = A7;    // battery voltage reading

void setup() {
	Serial.begin(9600);  // Computer & Arduino Interface started in 9600 bits per second, We need to write for usb communication debug.
	Serial.println("Ikedo MiniSumo");
	//Opponent Sensor Connection
	pinMode(RSens, INPUT_PULLUP);
	pinMode(RFSens, INPUT_PULLUP);
	pinMode(MSens, INPUT_PULLUP);
	pinMode(LFSens, INPUT_PULLUP);
	pinMode(LSens, INPUT_PULLUP);
	//Line Sensor Connection
	pinMode(LEdge, INPUT);
	pinMode(REdge, INPUT);
	//Motor Connection
	pinMode(RPwm, OUTPUT);
	pinMode(RDir, OUTPUT);
	pinMode(LPwm, OUTPUT);
	pinMode(LDir, OUTPUT);
	//Board connection
	pinMode(buzzer, OUTPUT);
	pinMode(led, OUTPUT);
	pinMode(trimpot, INPUT);    //trimpot input
	pinMode(dipswitch, INPUT);  // tactic switch input
	pinMode(start, INPUT);      //start button input
	pinMode(voltage, INPUT);
	
	analogWrite(RPwm, 0);
	digitalWrite(RDir, LOW);
	analogWrite(LPwm, 0);
	digitalWrite(LPwm, LOW);
	//digitalWrite(start, LOW);
	digitalWrite(buzzer, LOW);
	
	delay(1200);
	
	while (millis() < 1200 + CALIBRATION_DELAY) {
		digitalWrite(led, HIGH);
		Serial.println("Press Start for Calibration");
		if (digitalRead(start)) {
			Serial.println("Calibrating starts in 3");
			digitalWrite(led, HIGH);
			tone(buzzer, 523, 500);
			delay(500);
			digitalWrite(led, LOW);
			noTone(buzzer);
			delay(500);
			Serial.println("Calibrating starts in 2");
			digitalWrite(led, HIGH);
			tone(buzzer, 125, 500);
			delay(500);
			digitalWrite(led, LOW);
			noTone(buzzer);
			delay(500);
			Serial.println("Calibrating starts in 1");
			digitalWrite(led, HIGH);
			tone(buzzer, 750, 500);
			delay(500);
			digitalWrite(led, LOW);
			noTone(buzzer);
			calibrateEdgeSensor();
		}
	}
	digitalWrite(led, LOW);
	EDGE_L_THRESHOLD = sensorThreshold(LEFT);
	EDGE_R_THRESHOLD = sensorThreshold(RIGHT);
	
	
	startRoutine();
}

/*******************************************************************************
	Main program loop.
*******************************************************************************/
void loop() {
	IdleSpeed = map(analogRead(trimpot), 0, 1023, 0, 100);
	MaxSpeed = 1.6 * map(analogRead(trimpot), 0, 1023, 0, 100);
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
		while (1)
		;
	}
}

/*******************************************************************************
	Start Routine
	This function should be called once only when the game start.
*******************************************************************************/
void startRoutine() {
	while (!digitalRead(start)) {
		sensordebug();
		if (analogRead(LEdge) < EDGE_L_THRESHOLD || analogRead(REdge) < EDGE_R_THRESHOLD) {
			digitalWrite(led, HIGH);
			} else {
			digitalWrite(led, LOW);
		}
		delay(20);
	}
	if (digitalRead(start)) {
		if (delay_enable == true) {
			for (int i = 0; i < duration; i++) {  //set delay time
				Serial.println("Sumo Starts in: " + String(duration - i));
				digitalWrite(led, LOW);
				noTone(buzzer);
				delay(700);
				digitalWrite(led, HIGH);
				tone(buzzer, 523, 300);
				delay(300);
			}
		}
		Serial.println("Start Sumo");
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
			Set_Motor(70, -70);             //(60, -60)
			delay(120);                     //(100)
			Set_Motor(-10, 15);             //(-10, 10)
			delay(20);                      //(15)
			Set_Motor(50, 70);
			delay(500);
			while (digitalRead(LSens) || digitalRead(LFSens) || digitalRead(MSens) || digitalRead(RFSens) || digitalRead(LSens)) {
				// Quit if opponent is not found after timeout.
				if (!digitalRead(LSens) || !digitalRead(LFSens) || !digitalRead(MSens) || !digitalRead(RFSens) || !digitalRead(RSens)) {
					break;
				}
				Set_Motor(50, 70);
				delay(500);
				Set_Motor(-5, -5);
				delay(10);
				Set_Motor(-50, 80);  //(-60, 70)
				delay(250);          //(180)
				Set_Motor(10, -10);
				delay(15);
				stopmotor();
				// Quit if opponent is not found after timeout.
				if (!digitalRead(LSens) || !digitalRead(LFSens) || !digitalRead(MSens) || !digitalRead(RFSens) || !digitalRead(RSens)) {
					break;
				}
				if (millis() - startTimestamp > 3000) {
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
			Set_Motor(-75, 75);             //(-60, 60)
			delay(150);                     //100)
			Set_Motor(25, -15);             //(10, -10)
			delay(20);                      //(15)
			Set_Motor(85, 40);              //(70, 45)
			delay(600);
			while (digitalRead(LSens) || digitalRead(LFSens) || digitalRead(MSens) || digitalRead(RFSens) || digitalRead(RSens)) {
				// Quit if opponent is not found after timeout.
				if (!digitalRead(LSens) || !digitalRead(LFSens) || !digitalRead(MSens) || !digitalRead(RFSens) || !digitalRead(RSens)) {
					break;
				}
				Set_Motor(85, 40);  //(70, 45)
				delay(400);
				Set_Motor(-5, -5);
				delay(10);
				Set_Motor(70, -50);  //(70, -60)
				delay(250);          //(180)
				Set_Motor(-10, 10);
				delay(15);
				stopmotor();
				// Quit if opponent is not found after timeout.
				if (!digitalRead(LSens) || !digitalRead(LFSens) || !digitalRead(MSens) || !digitalRead(RFSens) || !digitalRead(RSens)) {
					break;
				}
				if (millis() - startTimestamp > 3000) {
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
			//delay(1000);         //(2000)
			while (digitalRead(LSens) || digitalRead(LFSens) || digitalRead(MSens) || digitalRead(RFSens) || digitalRead(RSens)) {
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
	delay(150);
	
	// Start looking for opponent.
	// Timeout after a short period.
	uint32_t uTurnTimestamp = millis();
	while (millis() - uTurnTimestamp < 200) {
		// Opponent is detected if either one of the opponent sensor is triggered.
		if (!digitalRead(LFSens) || !digitalRead(MSens) || !digitalRead(RFSens))) {
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
			digitalWrite(led, !digitalRead(led));
			tone(buzzer, 523, 100);
			noTone(buzzer);
		}
	} while (digitalRead(start));
	if (!digitalRead(start));
	Serial.println("DONE CALIBRATE wait for values");
	digitalWrite(led, HIGH);
	tone(buzzer, 523, 500);
	delay(500);
	digitalWrite(led, LOW);
	noTone(buzzer);
	delay(500);
	digitalWrite(led, HIGH);
	tone(buzzer, 125, 500);
	delay(500);
	digitalWrite(led, LOW);
	noTone(buzzer);
	delay(500);
	digitalWrite(led, HIGH);
	tone(buzzer, 750, 500);
	delay(500);
	digitalWrite(led, LOW);
	noTone(buzzer);
	delay(500);
	if (maxL > minL) {
		int threshold = ((maxL - minL) * 3 / 5) + minL;
		EEPROM.put(EEADD_EDGE_L, threshold);
	}
	if (maxR > minR) {
		int threshold = ((maxR - minR) * 3 / 5) + minR;
		EEPROM.put(EEADD_EDGE_R, threshold);
	}
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
	EEPROM.get(eepromAddress, threshold);
	
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
		analogWrite(LPwm, Lval);
		digitalWrite(LDir, HIGH);
		} else {
		Lval = abs(Lval);
		analogWrite(LPwm, Lval);
		digitalWrite(LDir, LOW);
	}
	if (Rval >= 0) {
		analogWrite(RPwm, Rval);
		digitalWrite(RDir, HIGH);
		} else {
		Rval = abs(Rval);
		analogWrite(RPwm, Rval);
		digitalWrite(RDir, LOW);
	}
}

void stopmotor() {
	digitalWrite(RDir, LOW);
	analogWrite(RPwm, 0);
	digitalWrite(LDir, LOW);
	analogWrite(LPwm, 0);
}

void sensordebug() {
	int speed = analogRead(trimpot);
	int tactic = analogRead(dipswitch);
	Serial.print("Start Button : ");
	Serial.print(digitalRead(start));
	Serial.print("   ");
	Serial.print("Dipswitch : ");
	Serial.print(readDipSwitch());
	Serial.print(" | ");
	Serial.print(tactic);
	Serial.print("\t");
	Serial.print("Trimpot : ");
	Serial.print(speed);
	Serial.print("\t");
	Serial.print("Voltage : ");
	Serial.print(readBatteryVoltage());
	Serial.print("\t");
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
