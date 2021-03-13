/* Advanced Line Follower Code (Basic version same as Maze solver ver. 12.0)
 * Config :- Generalized
 * Algorithm :- Manual (Non-PID)
 * Author :- Malhar Chakraborty
 */

//Generalized Config
//w=1, b=0 for white line on black board
#define W 0 
#define B 1

//Speed Setup or Controller
#define fspeed     255
#define corspeed   150
#define turnspeed  150


#define leftCenterSensor   19  //analog pin A5
#define leftNearSensor     18  //analog pin A4
#define leftFarSensor      17  //analog pin A3
#define rightCenterSensor  20  //analog pin A6
#define rightNearSensor    21  //analog pin A7
#define rightFarSensor      2   //digital pin D2

//Motor PWM pins
#define leftMotor1    3  //forward pin
#define leftMotor2    9
#define rightMotor1  10  //forward pin
#define rightMotor2  11

//Other peripherals pins
#define led  12
#define butn  4

short interval = 0; //For generelized Blink without delay function
bool psensor[6]; //The Sensor Array
bool ledState = HIGH;
short phase = 0;
unsigned long previousMillis = 0; //For blink without delay function
unsigned char mode;
char path[100];
short pathlen=0;
short Speed;

//List of all function (Not Prototype... (prototypes not required/mandatory in Arduino))

//Main functions
void readSensor();
void blink_without_delay(short); //Generelized Blink (without delay) -- parameter -> on-off gap time
void Straight(short);
void correct(); //Correct the normal line following
void Stop(); //Stop all motors
void Leap();
void Left();
void Right();
void Yaw(char , short);  //Generalized Yaw function


void setup()
{ 
  //Sensor pins Mode config
  pinMode(leftCenterSensor, INPUT);
  pinMode(leftNearSensor, INPUT);
  pinMode(leftFarSensor, INPUT);
  pinMode(rightCenterSensor, INPUT);
  pinMode(rightNearSensor, INPUT);
  pinMode(rightFarSensor, INPUT);

  //Motor pins Mode config
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);

  //Other peripherals pin Mode config
  pinMode(butn, INPUT_PULLUP);
  pinMode(led, OUTPUT);
  
  Serial.begin(115200); //For debugging only
  
  digitalWrite(led, HIGH);
  
  //led will blink for infinite times until the push button is pressed (This is the time for manual calibration of the sensors)
  for( ; ; )
  {
    if(digitalRead(butn)!=0)
    {
      blink_without_delay(100);
    }
    else
    {
      break;
    }
  }
}

//Setting up the Sensor Array
void readSensor()
{
  psensor[0] = digitalRead(leftFarSensor);
  psensor[1] = digitalRead(leftNearSensor);
  psensor[2] = digitalRead(leftCenterSensor);
  if (analogRead(rightCenterSensor)>500)
    psensor[3] = 1;
  else 
    psensor[3] = 0;
  if (analogRead(rightNearSensor)>500)
    psensor[4] = 1;
  else 
    psensor[4] = 0;
  psensor[5] = digitalRead(rightFarSensor);

  /*FIVE (5) POSIBILITIES The Robo Will encounter (including when the robo is normally following line, their Sensor Data ----
   *  0 0 1 1 0 0  == The Robo is on the line, perfectly alligned
   *  1 1 1 1 0 0  == The Robo is on an intersection => "Straight/Left" or "only Left"
   *  0 0 1 1 1 1  == ..................intersection => "Straight/Right" or "only Right"
   *  1 1 1 1 1 1  == ..................intersection => "T-intsersection" or "Cross-intersection"
   *  0 0 0 0 0 0  == ..................intersection => "Dead End" or "out of Line"
   */

  //For Advance Correction (if the Robo goes out of line)
  //This portion of code will remember from which side the Robo went out of Line
  if(psensor[0]==W)
  {
    if(path[pathlen] == 'R')
    {
      ++pathlen;
      path[pathlen] = 'L';
    }
    else
    {
      path[pathlen] = 'L';
    }
  }

  else if(psensor[5]==W)
  {
    if(path[pathlen] == 'L')
    {
      ++pathlen;
      path[pathlen] = 'R';
    }
    else
    {
      path[pathlen] = 'R';
    }
  }
  
  //Case: "0 0 1 1 0 0" or "0 1 1 0 0 0" or "0 0 0 1 1 0" or "0 0 1 1 1 0" or "0 1 1 1 0 0" 
  //for line : "1 1 0 0 0 0" or "1 1 1 0 0 0" or "1 0 0 0 0 0" or "0 0 0 0 0 1" or "0 0 0 0 1 1" or "0 0 0 1 1 1"
  //2nd, 3rd, 4th and 5th cases are for the turning/yawing correction
  if ((psensor[0]==B && psensor[1]==B && psensor[2]==W && psensor[3]==W && psensor[4]==B && psensor[5]==B)||
      (psensor[0]==B && psensor[1]==W && psensor[2]==W && psensor[3]==W && psensor[4]==B && psensor[5]==B)|| 
      (psensor[0]==B && psensor[1]==B && psensor[2]==W && psensor[3]==W && psensor[4]==W && psensor[5]==B)|| 
      (psensor[0]==B && psensor[1]==W && psensor[2]==W && psensor[3]==B && psensor[4]==B && psensor[5]==B)|| 
      (psensor[0]==B && psensor[1]==B && psensor[2]==B && psensor[3]==W && psensor[4]==W && psensor[5]==B))
  {
    mode = 'O';
  }
  //For 2nd degree correction
  else if ((psensor[0]==W && psensor[1]==W && psensor[2]==B && psensor[3]==B && psensor[4]==B && psensor[5]==B)||
           (psensor[0]==W && psensor[1]==W && psensor[2]==W && psensor[3]==B && psensor[4]==B && psensor[5]==B)||   //exclude this?
           (psensor[0]==W && psensor[1]==B && psensor[2]==B && psensor[3]==B && psensor[4]==B && psensor[5]==B)||
           (psensor[0]==B && psensor[1]==B && psensor[2]==B && psensor[3]==B && psensor[4]==B && psensor[5]==W)||
           (psensor[0]==B && psensor[1]==B && psensor[2]==B && psensor[3]==B && psensor[4]==W && psensor[5]==W)||
           (psensor[0]==B && psensor[1]==B && psensor[2]==B && psensor[3]==W && psensor[4]==W && psensor[5]==W))   //this?
  {
    mode = 'C';                        
  }
    
  //Case: "0 0 0 0 0 0"
  else if (psensor[0]==B && psensor[1]==B && psensor[2]==B && psensor[3]==B && psensor[4]==B && psensor[5]==B)
  {
    mode = 'D'; //Dead End
  }

  else if ((psensor[0]==W && psensor[1]==W && psensor[2]==W && psensor[3]==W && psensor[4]==B && psensor[5]==B) ||
           (psensor[0]==W && psensor[1]==W && psensor[2]==W && psensor[3]==W && psensor[4]==W && psensor[5]==B))
           {
            mode = 'L';
           }

  else if ((psensor[0]==B && psensor[1]==B && psensor[2]==W && psensor[3]==W && psensor[4]==W && psensor[5]==W) ||
           (psensor[0]==B && psensor[1]==W && psensor[2]==W && psensor[3]==W && psensor[4]==W && psensor[5]==W))
           {
            mode = 'R';
           }
  
  //Case: "1 1 1 1 1 1" checkpoint detector
  if ((psensor[0]==W && psensor[1]==W && psensor[2]==W && psensor[3]==W && psensor[4]==W && psensor[5]==W))
  {
    mode = 'X'; 
  }

  
  Serial.print(psensor[0]);
  Serial.print(psensor[1]);
  Serial.print(psensor[2]);
  Serial.print(psensor[3]);
  Serial.print(psensor[4]);
  Serial.print(psensor[5]);
  Serial.print("  -> MODE : ");
  Serial.println(mode);
}
//Sensor array config done


void blink_without_delay(short interval)
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) 
  {
    previousMillis = currentMillis;
    if (ledState == LOW) 
    {
      ledState = HIGH;
    } 
    else 
    {
      ledState = LOW;
    }
    digitalWrite(led, ledState);
  }
}

void loop() 
{
  unsigned long cur_mil = millis();
  readSensor();
  
  //for Checkpoint detection Reset
  if (cur_mil - previousMillis >= 1000) 
  {
    digitalWrite(led, LOW);
  }
  
    switch(mode)
    {
      case 'O' :  //On-Line
      {
        //Serial.println("on-line");
        Straight(fspeed);
        //Straight(255);
        //Go straight and on the same time correct position
        //Should not consist of any delay
        readSensor();
        break;
      }
      case 'C' :  //2deg correction
      {
        Speed = (corspeed);
        Serial.println("2nd DEGREE CORRECTION");
        if((psensor[0]==W))
        {
          Serial.println("2deg LEFT correction");
          analogWrite(rightMotor1, Speed);
          analogWrite(rightMotor2, 0);
          analogWrite(leftMotor2, 0);
          analogWrite(leftMotor1, 0);
        }
        else if((psensor[5]==W))
        {
          Serial.println("2deg RIGHT correction");
          analogWrite(leftMotor1, Speed);
          analogWrite(leftMotor2, 0);
          analogWrite(rightMotor2, 0);
          analogWrite(rightMotor1, 0);
        }
        //else if ((psensor[0]==W && psensor[1]==W && psensor[2]==W)||
        //         (psensor[0]==W && psensor[1]==W))
        else
        {
          Straight(fspeed);
        }
        readSensor();
        break;
      }

      case 'L' :
      {
        Stop();
        Straight(fspeed);
        delay(100);
        Left();
        readSensor();
        break;
      }

      case 'R' :
      {
        Stop();
        Straight(fspeed);
        delay(100);
        Right();
        readSensor();
        break;
      }
      
      case 'X' :  //Cross-intersection (Checkpoint detector if 1 1 1 1 1 1)
      {
        Stop();
        Serial.println("++++++++++++++++ CHECKPOINT detected +++++++++++++++++++");
        //Flash the LED and buzz the buzzer
        
        previousMillis = cur_mil;
        digitalWrite(led, HIGH);
        Serial.println(cur_mil);
        
        Straight(fspeed);
        readSensor();
        break;
      }
      
      case 'D' :  //Dead-End
      {
        Stop();
        //Flash the LED
        readSensor();

        //Print from which direction the Robo Drifted out of line
        Serial.print(path);
        Serial.println(" ");

        //Re-Allignment
        if(path[pathlen] == 'L')
        {
          Left();
        }
        else if(path[pathlen] == 'R')
        {
          Right();
        }
        pathlen=0;
        break;
      }
    }
}

void Straight(short Speed)
{
  analogWrite(leftMotor1, Speed);
  analogWrite(leftMotor2, 0);
  analogWrite(rightMotor1, Speed);
  analogWrite(rightMotor2, 0);
  Serial.println("Straight");
  correct();
}

void correct()
{
  readSensor();
  Serial.println("Correct");
  if(psensor[1]==W && psensor[5]==B)
  {
    Serial.println("LEFT CORRECT");
    analogWrite(leftMotor1, Speed/2);
    delay(10);
  }
  else if(psensor[4]==W && psensor[0]==B)
  {
    Serial.println("RIGHT CORRECT");
    analogWrite(rightMotor1, Speed/2);
    delay(10);
  }
  readSensor();
}


void Left()
{
  //Stop();
  unsigned long cur_mil = millis();
  do
  {
    Yaw('L', turnspeed);  //High Speed Left Yaw
    Serial.println("LEFT TURN EXECUTED");
    readSensor();
  }while(psensor[0]!=W);
  Stop();
  Yaw('R', turnspeed); //For BRAKING!!!
  delay(5);
  Stop();
}

void Right()
{
  unsigned long cur_mil = millis();
  do
  {
    Yaw('R', turnspeed);  //High Speed Right Yaw
    Serial.println("RIGHT TURN EXECUTED");
    readSensor();
  }while(psensor[5]==B);
  Stop();
  Yaw('L', turnspeed);  //for BRAKING!!!
  delay(5);
  Stop();
}

void Yaw(char direc , short Spd)
{
  switch(direc)
  {
    case 'L' :
    {
      Serial.println("Left");
      analogWrite(leftMotor1, 0);
      analogWrite(leftMotor2, Spd);
      analogWrite(rightMotor1, Spd);
      analogWrite(rightMotor2, 0);
      break;
    }
    case 'R' :
    {
      Serial.println("Right");
      analogWrite(leftMotor1, Spd);
      analogWrite(leftMotor2, 0);
      analogWrite(rightMotor1, 0);
      analogWrite(rightMotor2, Spd);
      break;
    }
  }
}

void Stop()
{
  analogWrite(leftMotor1, 0);
  analogWrite(rightMotor1, 0);
  analogWrite(leftMotor2, 0);
  analogWrite(rightMotor2, 0);
}
