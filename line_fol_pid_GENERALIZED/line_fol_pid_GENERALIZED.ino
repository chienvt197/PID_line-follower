//Advanced Line Follower Code (Basic version same as Maze solver ver. 12.0)

//Generalized Config
//w=1, b=0 for white line on black board
#define W 0 
#define B 1
#define fspeed     250 //this's not worable yet
#define corspeed   150 //this is base/initial motor speed
#define turnspeed  160

#define Kp 27
#define Ki 0
#define Kd 15

#define leftCenterSensor   19  //analog pin A5
#define leftNearSensor     18  //analog pin A4
#define leftFarSensor      17  //analog pin A3
#define rightCenterSensor  20  //analog pin A6
#define rightNearSensor    21  //analog pin A7
#define rightFarSensor      2   //digital pin D2
#define Lleapsensor        16  //analog pin A2
#define Rleapsensor        14  //analog pin A0
#define Vin                15  //analog pin A1

//Motor PWM pins
#define leftMotor1    3  //forward pin
#define leftMotor2    9
#define rightMotor1  10  //forward pin
#define rightMotor2  11

//Other peripherals pins
#define led  12
#define buz   8
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
float Vbb; //divided Battery voltage on pin A1
float Bat_V; //actual Battery voltage
short error; //line following error, pid will always try to minimize this error
float p; //proportional term
float i; //integral term
float d; //differential term
float pid; //pid term
float prev_error; //previous error term
float prev_i; //previous integral term
short initial_motor_speed = 0;


//List of all function (Not Prototype... (prototypes not required/mandatory in Arduino))

//Tone Functions
void ready_tone(); //Uses delay()

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
void Cal_V(); //battery voltage calculator function

void setup()
{ 
  //Sensor pins Mode config
  pinMode(leftCenterSensor, INPUT);
  pinMode(leftNearSensor, INPUT);
  pinMode(leftFarSensor, INPUT);
  pinMode(rightCenterSensor, INPUT);
  pinMode(rightNearSensor, INPUT);
  pinMode(rightFarSensor, INPUT);
  pinMode(Lleapsensor, INPUT);
  pinMode(Rleapsensor, INPUT);

  //Motor pins Mode config
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);

  //Other peripherals pin Mode config
  pinMode(butn, INPUT_PULLUP);
  pinMode(buz, OUTPUT);
  pinMode(led, OUTPUT);
  
  Serial.begin(115200); //For debugging only
  
  digitalWrite(led, HIGH);
  ready_tone();

  Cal_V();
  Serial.print("Battery Voltage : ");
  Serial.println(Bat_V);

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

//Calculate the Battery Voltage
void Cal_V()
{
  Vbb = (analogRead(Vin) * 4.443)/1000;
  Bat_V = Vbb/(0.35263158);
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
   *  1 1 1 1 1 1  == ..................intersection => "T-intsersection" or "Cross-intersection" or "End of Maze"
   *  0 0 0 0 0 0  == ..................intersection => "Dead End"
   */
   
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
  if(psensor[0]==W && psensor[1]==B && psensor[2]==B && psensor[3]==B && psensor[4]==B && psensor[5]==B)
  {
    mode = 'O';
    error = 3;
  }
  
  else if(psensor[0]==W && psensor[1]==W && psensor[2]==B && psensor[3]==B && psensor[4]==B && psensor[5]==B)
  {
    mode = 'O';
    error = 3;
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
  
  else if(psensor[0]==B && psensor[1]==W && psensor[2]==W && psensor[3]==B && psensor[4]==B && psensor[5]==B)
  {
    mode = 'O';
    error = 2;
  }
  
  else if(psensor[0]==B && psensor[1]==W && psensor[2]==W && psensor[3]==W && psensor[4]==B && psensor[5]==B)
  {
    mode = 'O';
    error = 1;
  }
  
  else if((psensor[0]==B && psensor[1]==B && psensor[2]==W && psensor[3]==W && psensor[4]==B && psensor[5]==B) ||
          (psensor[0]==B && psensor[1]==B && psensor[2]==B && psensor[3]==W && psensor[4]==B && psensor[5]==B) ||
          (psensor[0]==B && psensor[1]==B && psensor[2]==W && psensor[3]==B && psensor[4]==B && psensor[5]==B))
  {
    mode = 'O';
    error = 0;
  }

  else if(psensor[0]==B && psensor[1]==B && psensor[2]==W && psensor[3]==W && psensor[4]==W && psensor[5]==B)
  {
    mode = 'O';
    error = -1;
  }
  
  else if(psensor[0]==B && psensor[1]==B && psensor[2]==B && psensor[3]==W && psensor[4]==W && psensor[5]==B)
  {
    mode = 'O';
    error = -2;
  }
  
  else if(psensor[0]==B && psensor[1]==B && psensor[2]==B && psensor[3]==B && psensor[4]==W && psensor[5]==W)
  {
    mode = 'O';
    error = -3;
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

  else if(psensor[0]==B && psensor[1]==B && psensor[2]==B && psensor[3]==B && psensor[4]==B && psensor[5]==W)
  {
    mode = 'O';
    error = -3;
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

  //Case: "1 1 1 1 0 0" or "1 1 1 1 1 0" second case for (in case) correction error
  else if (psensor[0]==W && psensor[1]==W && psensor[2]==W && psensor[3]==W && psensor[4]==B && psensor[5]==B)         
  {
    mode = 'L';
    error = 100;
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
  //Case: "0 0 1 1 1 1" or "0 1 1 1 1 1" second case for (in case) correction error
  else if (psensor[0]==B && psensor[1]==B && psensor[2]==W && psensor[3]==W && psensor[4]==W && psensor[5]==W)
  {
    mode = 'R';
    error = 101;
  }
  //Case: "1 1 1 1 1 1"
  else if (psensor[0]==W && psensor[1]==W && psensor[2]==W && psensor[3]==W && psensor[4]==W && psensor[5]==W)
  {
    mode = 'X'; //checkpoint detector
    error = 102;
  }
  //Case: "0 0 0 0 0 0"
  else if (psensor[0]==B && psensor[1]==B && psensor[2]==B && psensor[3]==B && psensor[4]==B && psensor[5]==B)
  {
    mode = 'D'; //Dead End
    error = 103;
  }

  
  Serial.print(psensor[0]);
  Serial.print(psensor[1]);
  Serial.print(psensor[2]);
  Serial.print(psensor[3]);
  Serial.print(psensor[4]);
  Serial.print(psensor[5]);
  Serial.print("  -> MODE : ");
  Serial.print(mode);
  Serial.print("  ERROR : ");
  Serial.println(error);
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
  Cal_V();
  initial_motor_speed = corspeed/Bat_V;
  
  if(cur_mil - previousMillis >=100)
  {
    analogWrite(buz, 0);
  }
  if (cur_mil - previousMillis >= 1000) 
  {
    digitalWrite(led, LOW);
  }
  
    switch(mode)
    {
      case 'O' :  //On-Line
      {
        //Serial.println("on-line");
        cal_pid();
        forward_correct();
        //Go straight and on the same time correct position
        //Should not consist of any delay
        readSensor();
        Cal_V();
        break;
      }


      case 'L' :
      {
        Stop();
        delay(50);
        readSensor(); //new
        analogWrite(leftMotor1, 150);
        analogWrite(rightMotor1, 150);
        analogWrite(leftMotor2, 0);
        analogWrite(rightMotor2, 0);
        delay(150);
        Stop();
        //delay(50);
        Left();
        readSensor();
        Cal_V();
        break;
      }

      case 'R' :
      {
        Stop();
        delay(50);
        readSensor();
        analogWrite(leftMotor1, 150);
        analogWrite(rightMotor1, 150);
        analogWrite(leftMotor2, 0);
        analogWrite(rightMotor2, 0);
        delay(150);  
        Stop();
        //delay(50);
        Right();
        readSensor();
        Cal_V();
        break;
      }

      
      case 'X' :  //Cross-intersection (Checkpoint detector if 1 1 1 1 1 1)
      {
        Stop();
        Serial.println("++++++++++++++++ CHECKPOINT detected +++++++++++++++++++");
        //Flash the LED and buzz the buzzer
        
        Left(); //giving particular turn the prioirity
        
        previousMillis = cur_mil;
        analogWrite(buz, 150);
        digitalWrite(led, HIGH);
        Serial.println(cur_mil);
        
        cal_pid();
        forward_correct();
        
        readSensor();
        Cal_V();
        break;
      }
      
      case 'D' :  //Dead-End
      {
        Stop();     
        readSensor();
        Cal_V();
        Serial.println("CASE D!!!");
        Serial.print(path);
        Serial.println(" ");
        unsigned long timer = 0;
          do
          {
            timer++;
            Serial.print("Detecting Line :> ");
            Serial.println(timer);
            blink_without_delay(25);
            analogWrite(leftMotor1, 150);
            analogWrite(rightMotor1, 150);
            analogWrite(leftMotor2, 0);
            analogWrite(rightMotor2, 0);
            readSensor();
            if(timer > 30)
            {
              goto End;
            }
          }while(error>3 || error<-3);
          End:
          Stop();
          
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

void cal_pid()
{
  p = error;
  i = i + prev_i;
  d = error - (prev_error);
  pid = (Kp*p)+(Ki*i)+(Kd*d);
  prev_i = i;
  prev_error = error;
}

void forward_correct()
{
  // Calculating the effective motor speed:
  int left_motor_speed = initial_motor_speed - pid;
  int right_motor_speed = initial_motor_speed + pid;

  // The motor speed should not exceed the max PWM value
  left_motor_speed = constrain(left_motor_speed, 0, 255);
  right_motor_speed = constrain(right_motor_speed, 0, 255);

  analogWrite(leftMotor1, left_motor_speed); //Left Motor Speed
  analogWrite(rightMotor1, right_motor_speed); //Right Motor Speed
}

void Left()
{
  Stop();
  unsigned long cur_mil = millis();
  /*do
  {
    Yaw('L', turnspeed/Bat_V);  //High Speed Left Yaw
    Serial.println("LEFT TURN EXECUTED");
    readSensor();
  }while(psensor[0]!=W); */
  Yaw('L', turnspeed/Bat_V);
  delay(10);
  Stop();
  do
  {
    Yaw('L', turnspeed/Bat_V);  //High Speed Left Yaw
    Serial.println("LEFT TURN");
    readSensor();
  }while(error > 2 || error < -2);
  Stop();
}

void Right()
{
  unsigned long cur_mil = millis();
  /*do
  {
    Yaw('R', turnspeed/Bat_V);  //High Speed Right Yaw
    Serial.println("RIGHT TURN EXECUTED");
    readSensor();
  }while(psensor[5]!=W); */
  Yaw('R', turnspeed/Bat_V);
  delay(10);
  Stop();
  do
  {
    Yaw('R', turnspeed/Bat_V);  //High Speed Right Yaw
    Serial.println("RIGHT TURN");
    readSensor();
  }while(error > 2 || error < -2);
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

void ready_tone()
{
  tone(buz, 1000, 170);
  delay(150);
  tone(buz, 1150, 177);
  delay(150);
  tone(buz, 1300, 190);
  delay(150);
  tone(buz, 0, 500);
}
