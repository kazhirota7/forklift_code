#include <Stepper.h>

# include <Servo.h>

Servo myservo;
// Stepper Motor X
const int stepPin = 2; //X.STEP
const int dirPin = 5; // X.DIR
const int sPY = 3;
const int dPY = 6;
const int sPZ = 4;
const int dPZ = 7;
const int button = 12; // Put the button at SpnEn (12)
int butVal = 0;
int lastbutVal = 0;
int dis = 0;
int lift = 0;
int complete = 1;
int t = 0;
int turn = 0;
int pos = 95;
int turndist = 0;
int statement = 0;

//const int rs = 8, en = 9, d4 = 10, d5 = 11, d6 = 1, d7 = A1; // Change these in script for robot.
//
//LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

enum states {
  Driving,
  Turning,
  Lifting,
  Idle
};

states Current_State;

void setup() {
  // Sets the two pins as Outputs
  Serial.begin(9600);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(sPY, OUTPUT);
  pinMode(dPY, OUTPUT);
  pinMode(sPZ, OUTPUT);
  pinMode(sPZ, OUTPUT);
  pinMode(button, INPUT);
  digitalWrite(stepPin, LOW); // At the beginning, the motor is not stepping
  digitalWrite(dirPin, LOW); // HIGH or LOW: Changes the rotations direction, arms are fully down
  digitalWrite(sPY, LOW);
  digitalWrite(dPY, LOW);
  // this is for the drive train
  digitalWrite(sPZ, LOW);
  digitalWrite(dPZ, LOW);  //We assume LOW is forwards and HIGH is backwards (CONFIRM?)

  myservo.attach(13);
  myservo.write(pos);
  butVal = digitalRead(button);
  Current_State = Idle;
  //  lcd.begin(16, 2);
  lastbutVal = butVal;

  for (lift = 0; lift < 500; lift++) { // This will turn the stepper motor 400 pulses (CHANGE TO LENGTH, STARTING SHORT)
    digitalWrite(stepPin, HIGH);
    digitalWrite(sPY, HIGH);
    delayMicroseconds(750);
    digitalWrite(stepPin, LOW);
    digitalWrite(sPY, LOW);
    delayMicroseconds(750);
  }
  //
  //  delay(500);
//  stepper.setSpeed(1000);
//  delay(5000);
//  stepper.setSpeed(0);
  for (dis = 0; dis < 625; dis++) { // This will turn the stepper motor 400 pulses (CHANGE TO LENGTH, STARTING SHORT)
    digitalWrite(sPZ, HIGH);
    delayMicroseconds(8000);
    digitalWrite(sPZ, LOW);
    delayMicroseconds(8000);
  }
  //    Serial.print(dis);
  //  dis = 0;

  //  delay(500);

  //  for (lift = 0; lift < 2500; lift++) { // This will turn the stepper motor 400 pulses (CHANGE TO LENGTH, STARTING SHORT)
  //    digitalWrite(stepPin, HIGH);
  //    digitalWrite(sPY, HIGH);
  //    delayMicroseconds(1000);
  //    digitalWrite(stepPin, LOW);
  //    digitalWrite(sPY, LOW);
  //    delayMicroseconds(1000);
  //  }

  //  for (dis = 0; dis < 625; dis++) { // This will turn the stepper motor 400 pulses (CHANGE TO LENGTH, STARTING SHORT)
  //    digitalWrite(sPZ, HIGH);
  //    delayMicroseconds(8000);
  //    digitalWrite(sPZ, LOW);
  //    delayMicroseconds(8000);
  //  }

  dis = 0;

  for (pos = 95; pos <= 110; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  delay(1000);
  for (pos = 110; pos >= 70; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  delay(1000);
  for (pos = 70; pos <= 95; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
}

void loop() {
  // For our robot, we want a single push to move the arms up a certain distance
  // and won't go down till the button is pressed a second time.
  butVal = digitalRead(button);
  Serial.println("button: ");
  Serial.println(digitalRead(button));
  Serial.println("State: ");
  Serial.println(Current_State);
  Serial.println("Complete: ");
  Serial.println(complete);
  //  lcd.noCursor();



  //  Serial.println(dis);

  if (butVal == lastbutVal && complete == 1) {
    Current_State = Idle;

  } else if (butVal == lastbutVal && complete != 1) {
    Current_State = Current_State;
    //    lcd.clear();
    //    lcd.print("In Progress");
  }
  if (butVal != lastbutVal && complete == 1) {
    Current_State = Driving;
    //    lcd.clear();
    //    lcd.print("In Progress");
    complete = 0;
  } else if (butVal != lastbutVal && complete != 1) {
    Current_State = Current_State;
    //    lcd.clear();
    //    lcd.print("In Progress");
  }

  switch (Current_State) {

    case Idle:
      if (t < 10 && statement == 0 && complete == 0) {
        // This will be the code to keep the LCD display say "Complete" if under a time
        Serial.println("Completed");
        //        lcd.clear();
        //        lcd.print("Completed");
        delay(100);

        t += 1;3,
        statement = 1;
      } else if (t < 10 && statement == 1 && complete == 0) {
        delay(200);
        t += 1;

      } else {
        // Display "Ready" if time t has passed
        //        lcd.clear();
        Serial.println("Ready");
        //        lcd.print("Ready");
        delay(1000);
        complete = 1;
        t = 0;
        statement = 0;
        dis = 0;
        lift = 0;
        turn = 0;
        pos = 0;
        turndist = 0;
        statement = 0;
      }
      break;

    case Driving:
      if (dis < 625 && turn == 0) { // This if statement will ensure that the motor won't keep turning after it has
        // already turned the desired length
        // This will set the direction
        digitalWrite(dPZ, LOW);
        for (dis = 0; dis < 625; dis++) { // This will turn the stepper motor 400 pulses (CHANGE TO LENGTH, STARTING SHORT)
          digitalWrite(sPZ, HIGH);
          delayMicroseconds(8000);
          digitalWrite(sPZ, LOW);
          delayMicroseconds(8000);
        }
        turn = 1;
        Current_State = Turning;
        Serial.println("First Forward");
      } else if (dis > 0 && turn == 0) {
        // This is to reverse to initial position
        digitalWrite(dPZ, HIGH);
        for (dis = 625; dis > 0; dis--) { // This will turn the stepper motor 400 pulses (CHANGE TO LENGTH, STARTING SHORT)
          digitalWrite(sPZ, HIGH);
          delayMicroseconds(8000);
          digitalWrite(sPZ, LOW);
          delayMicroseconds(8000);
        }
        Current_State = Idle;
        Serial.println("First Reverse");
      } else if (dis > 0 && turn == 1) {
        // This is to reverse to initial position
        digitalWrite(dPZ, HIGH);
        for (dis = 625; dis > 0; dis--) { // This will turn the stepper motor 400 pulses (CHANGE TO LENGTH, STARTING SHORT)
          digitalWrite(sPZ, HIGH);
          delayMicroseconds(8000);
          digitalWrite(sPZ, LOW);
          delayMicroseconds(8000);
        }
        Current_State = Lifting;
        Serial.println("Turned Straight");
      } else if (dis < 625 && turn == 1) {
        digitalWrite(dPZ, HIGH);
        for (dis = 0; dis < 625; dis++) { // This will turn the stepper motor 400 pulses (CHANGE TO LENGTH, STARTING SHORT)
          digitalWrite(sPZ, HIGH);
          delayMicroseconds(8000);
          digitalWrite(sPZ, LOW);
          delayMicroseconds(8000);
        }
        turn = 0;
        Current_State = Turning;
        Serial.println("Turned Reverse");
      }
      break;

    case Turning:
      // Left Turn as we will only be turning left and moving till we end up straight
      for (pos = 97; pos <= 117; pos += 1) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15 ms for the servo to reach the position
      }

      // After moving the servo, we test which turn condition we are in and move accordingly
      if (dis == 625 && turn == 1) {
        digitalWrite(dPZ, LOW);
        for (turndist = 0; turndist < 625; turndist++) { // This will turn the stepper motor 400 pulses (CHANGE TO LENGTH, STARTING SHORT)
          digitalWrite(sPZ, HIGH);
          delayMicroseconds(8000);
          digitalWrite(sPZ, LOW);
          delayMicroseconds(8000);
        }
        Serial.println("Left Forward");
        Current_State = Driving;
      } else if (dis == 625 && turn == 0) {
        digitalWrite(dPZ, HIGH);
        for (turndist = 0; turndist < 625; turndist++) { // This will turn the stepper motor 400 pulses (CHANGE TO LENGTH, STARTING SHORT)
          digitalWrite(sPZ, HIGH);
          delayMicroseconds(8000);
          digitalWrite(sPZ, LOW);
          delayMicroseconds(8000);
        }
        Serial.println("Left Reverse");
        Current_State = Lifting;
      }

      //This part is to straighten out the servo after turning
      for (pos = 117; pos >= 97; pos -= 1) { // goes from 180 degrees to 0 degrees
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15 ms for the servo to reach the position
      }
      break;

    case Lifting:
      if (lift < 3500 && turn == 1) { // This if statement will ensure that the motor won't keep turning after it has
        // already turned the desired length
        digitalWrite(dirPin, LOW);
        digitalWrite(dPY, LOW);
        for (lift = 500; lift < 3500; lift++) { // This will turn the stepper motor 400 pulses (CHANGE TO LENGTH, STARTING SHORT)
          digitalWrite(stepPin, HIGH);
          digitalWrite(sPY, HIGH);
          delayMicroseconds(750);
          digitalWrite(stepPin, LOW);
          digitalWrite(sPY, LOW);
          delayMicroseconds(750);
        }

        Serial.println("Lifting");
        Current_State = Driving;
      }
      if (lift == 3500 && turn == 0) { // This if statement will ensure that the motor won't keep turning after it has
        // already turned the desired length
        digitalWrite(dirPin, HIGH);
        digitalWrite(dPY, HIGH);
        for (lift = 3500; lift > 500; lift --) { // This will turn the stepper motor 400 pulses (CHANGE TO LENGTH, STARTING SHORT)
          digitalWrite(stepPin, HIGH);
          digitalWrite(sPY, HIGH);
          delayMicroseconds(750);
          digitalWrite(stepPin, LOW);
          digitalWrite(sPY, LOW);
          delayMicroseconds(750);
        }
        Serial.println("Lowering");
        Current_State = Driving;
      }
      break;
  }

  lastbutVal = butVal;
}
