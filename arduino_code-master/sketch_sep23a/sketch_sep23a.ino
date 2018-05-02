#include <Wire.h>

#define INFINITE 4294967295
#define LED_PIN 13

//for raspberry pi communication
const byte numChars = 32;
char receivedChars[numChars];

static boolean recvInProgress = false;
static byte ndx = 0;
char startMarker = '<';
char endMarker = '>';
char rc;

boolean newData = false;


//for motor control
int dir_pins[3] = {46, 34, 22};
int step_pins[3] = {50, 38, 26};
unsigned long time_stamps[3] = {0, 0, 0};
unsigned long delays[3] = {INFINITE, INFINITE, INFINITE};
boolean dirs[3] = {true, true, true};


void recvWithStartEndMarkers() {
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
        //Serial.println("receiving data");
        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }
        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void showNewData() {
    if (newData == true) {
//       Serial.println(receivedChars);
//         for (int i=0; i<10; i++)
//         {
//            digitalWrite(LED_PIN, true);
//            delay(100);
//            digitalWrite(LED_PIN, false);
//            delay(100);
//      }
        newData = false;
        parseData();
//        showParsedData();
    }
    
}

boolean sign(float num)
{
  if(num)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void parseData() {
//  lcd.clear();
  char * strtokIndx; // this is used by strtok() as an index
  
  //strtokIndx = strtok(receivedChars,":");      // get the first part - the string
  //strcpy(receivedChars, strtokIndx); // copy it to messageFromPC
  
  strtokIndx = strtok(receivedChars, ":"); // this continues where the previous call left off
  dirs[0] = strtokIndx[0] == '-' ? true : false;
  delays[0] = fabs(atof(strtokIndx));     // convert this part to an integer
//  lcd.print(String(strtokIndx));
//  lcd.print(" ");
//  lcd.setCursor(8,0);
  
  strtokIndx = strtok(NULL, ":"); 
  dirs[1] = strtokIndx[0] == '-' ? true : false;
  delays[1] = fabs(atof(strtokIndx));     // convert this part to a float
//  lcd.print(String(strtokIndx));
//  lcd.print(" ");
//  lcd.setCursor(0,1);
  
  strtokIndx = strtok(NULL, ":"); 
  dirs[2] = strtokIndx[0] == '-' ? true : false;
  delays[2] = fabs(atof(strtokIndx));     // convert this part to a float
//  lcd.print(String(strtokIndx));
//  lcd.print(" ");
//  lcd.setCursor(8,1);
  
}



void getCommand()
{
  recvWithStartEndMarkers();
  showNewData();
  
  //get speed command for each motor from rpi
//  for(int i = 0; i < 3; i++)
//  {
//    delays[i] = 100;
//  }
}

void step(int motor_idx)
{
  //TODO: take care of the time stamp overflow

  
  //change rotating direction
  

  //check time stamp
  if(time_stamps[motor_idx] + delays[motor_idx] <= micros())
  {
    if(dirs[motor_idx])
  {
    digitalWrite(dir_pins[motor_idx], HIGH);
  }
  else
  {
    digitalWrite(dir_pins[motor_idx], LOW);
  }
    digitalWrite(step_pins[motor_idx], HIGH);
    time_stamps[motor_idx] = time_stamps[motor_idx] + delays[motor_idx];
    digitalWrite(step_pins[motor_idx], LOW);
  }
    
}


void showParsedData() {
 Serial.print("m1: ");
 Serial.println(delays[0]);
 Serial.print("m2: ");
 Serial.println(delays[1]);
 Serial.print("m3: ");
 Serial.println(delays[2]);
}

void setup() {
    Serial.begin(115200);

    //Serial.println("<Arduino is ready>");
//    lcd.begin(16,2);
   // lcd.backlight();
    // set cursor to positon x=0, y=0
   // lcd.setCursor(0,0);
    // print Hello!
   // lcd.print("Waiting..");
  
  // put your setup code here, to run once:
    pinMode(22, OUTPUT);
    pinMode(24, OUTPUT);
    pinMode(26, OUTPUT);
    pinMode(28, OUTPUT);

    pinMode(34, OUTPUT);
    pinMode(36, OUTPUT);
    pinMode(38, OUTPUT);
    pinMode(40, OUTPUT);

    pinMode(46, OUTPUT);
    pinMode(48, OUTPUT);
    pinMode(50, OUTPUT);
    pinMode(52, OUTPUT);

    digitalWrite(48, HIGH);
    digitalWrite(52, HIGH);

    digitalWrite(24, HIGH);
    digitalWrite(28, HIGH);

    digitalWrite(36, HIGH);
    digitalWrite(40, HIGH);
  
  for(int idx = 0; idx < 3; idx ++)
  {
//    pinMode(dir_pins[idx], OUTPUT);
//    pinMode(step_pins[idx], OUTPUT);
    digitalWrite(dir_pins[idx], LOW);
    digitalWrite(step_pins[idx], LOW);
  }

//  Serial.println("pin setup done");
//  pinMode(LED_PIN, OUTPUT);
//  for (int i=0; i<3; i++)
//  {
//    digitalWrite(LED_PIN, true);
//    delay(500);
//    digitalWrite(LED_PIN, false);
//    delay(500);
//  }
}

void loop() {
  // put your main code here, to run repeatedly:
  getCommand();
  for(int i = 0; i < 3; i++)
  {
    step(i);
  }

}
