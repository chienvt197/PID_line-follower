uint8_t inA = 5, inB = 6, inC = 7, inD = 8, enA = 9, enB = 10;
uint16_t sensor[6] = {0, 0, 0, 0, 0, 0}; // Num of sensors
const uint16_t SensorPin[6] = {2, A0, A1, A2, A3, 3}; // 2 digital, 4 analog
uint16_t black, white;
float err = 0.0, err1 = 0.0, err2 = 0.0, pos, goal, blackThreshold, whiteThreshold;
const float L = 0.165, R = 0.025, v = 0.51; //m/s
float cur, last_cur, vR, vL, centre_to_sensor, dt;

void setup() {
  // put your setup code here, to run once:
  pinMode(inA, OUTPUT);
  pinMode(inB, OUTPUT);
  pinMode(inC, OUTPUT);
  pinMode(inD, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  for (uint8_t i = 0; i < 6; i++) {
    pinMode(SensorPin[i], INPUT);
  }
  calibrate();
  pos = goal = posLine();
  cur = millis();
}

void calibrate() {
  uint16_t Maxval[6] = {0, 0, 0, 0, 0, 0}, Minval[6] = {0, 0, 0, 0, 0, 0};
  while (millis() < 10000) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
    analogWrite(enA, 100);

    digitalWrite(inC, LOW);
    digitalWrite(inD, HIGH);
    analogWrite(enB, 100);

    for (uint8_t i = 1; i < 5; i++) {
      Maxval[i] = max(Maxval[i], analogRead(SensorPin[i]));
      Minval[i] = min(Minval[i], analogRead(SensorPin[i]));
    }
  }

  for (uint8_t i = 0; i < 5; i++) {
    black = min(black, Minval[i]);
    white = max(white, Maxval[i]);
  }

  blackThreshold = black + (white - black) * 1 / 4;
  whiteThreshold = white - (white - black) * 1 / 4;

  digitalWrite(inA, HIGH);
  digitalWrite(inB, HIGH);
  analogWrite(enA, 25);

  digitalWrite(inC, HIGH);
  digitalWrite(inD, HIGH);
  analogWrite(enB, 25);
  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:
  LineFollower();
}

bool on_line() {
  ReadSensor();
  return sensor[0] == 0 && sensor[1] > whiteThreshold && sensor[2] < blackThreshold
         && sensor[3] < blackThreshold && sensor[4] > whiteThreshold && sensor[5] == 0;
}

bool intersection() {
  ReadSensor();
  return sensor[0] == 1 && sensor[1] < blackThreshold && sensor[2] < blackThreshold
         && sensor[3] < blackThreshold && sensor[4] < blackThreshold && sensor[5] == 1;
}

bool right() {
  ReadSensor();
  return sensor[0] == 0 && sensor[1] > whiteThreshold && sensor[2] > whiteThreshold
         && sensor[3] < blackThreshold && sensor[4] < blackThreshold && sensor[5] == 1;
}

bool left() {
  ReadSensor();
  return sensor[0] == 1 && sensor[1] < blackThreshold && sensor[2] < blackThreshold
         && sensor[3] > whiteThreshold && sensor[4] > whiteThreshold && sensor[5] == 0;
}

bool deadline() {
  if (intersection()) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
    analogWrite(enA, 100);

    digitalWrite(inC, HIGH);
    digitalWrite(inD, LOW);
    analogWrite(enB, 100);

    delay(1000);

    digitalWrite(inA, HIGH);
    digitalWrite(inB, HIGH);
    analogWrite(enA, 25);

    digitalWrite(inC, HIGH);
    digitalWrite(inD, HIGH);
    analogWrite(enB, 25);

    return intersection();
  }
}

void LineFollower() {
  if (sensor[0] == 1) {
    while (sensor[2] > blackThreshold) {
      digitalWrite(inA, HIGH);
      digitalWrite(inB, LOW);
      analogWrite(enA, 180);

      digitalWrite(inC, HIGH);
      digitalWrite(inD, LOW);
      analogWrite(enB, 100);
    }
  }

  else if (sensor[1] == 1) {
    while (sensor[4] > blackThreshold) {
      digitalWrite(inA, HIGH);
      digitalWrite(inB, LOW);
      analogWrite(enA, 100);

      digitalWrite(inC, HIGH);
      digitalWrite(inD, LOW);
      analogWrite(enB, 180);
    }
  }

  if(deadline()){
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    analogWrite(enA, 0);

    digitalWrite(inC, LOW);
    digitalWrite(inD, LOW);
    analogWrite(enB, 0);
  }

  else if(right()){
    while(!on_line){
      digitalWrite(inA, LOW);
      digitalWrite(inB, HIGH);
      analogWrite(enA, 100);
  
      digitalWrite(inC, HIGH);
      digitalWrite(inD, LOW);
      analogWrite(enB, 100);
    }
  }

  else if (left()){
    while(!on_line){
      digitalWrite(inA, HIGH);
      digitalWrite(inB, LOW);
      analogWrite(enA, 100);
  
      digitalWrite(inC, LOW);
      digitalWrite(inD, HIGH);
      analogWrite(enB, 100);
    }
  }

  else{
    computePID(0.035, 0.15, 0.5);
  }
}

void computePID(float Kp, float Ti, float Td) {
  dt = (millis() - last_cur) / 1000;
  last_cur = millis();
  err2 = err1;
  err1 = err;
  err = atan((goal - pos) / centre_to_sensor) * PI / 180;
  pos = posLine();
  float deltapos = Kp * (err - err1) + (dt / Ti) * err1 + (Td / dt) * (err - 2 * err1 + err2);
  float w = deltapos / dt;

  vR = 30 * (L * w + 2 * v) / (PI * R); // rpm
  vL = 30 * (L * w - 2 * v) / (PI * R); // rpm

  vR = map(vR, 0, 600, 0, 255); //rpm -> pwm
  vL = map(vR, 0, 600, 0, 255); //rpm -> pwm
}

float posLine() {
  uint16_t avg = 0, sum = 0;
  uint16_t map_result[4] = {0, 0, 0, 0};
  sensor[0] = digitalRead(SensorPin[0]);
  sensor[5] = digitalRead(SensorPin[5]);
  for (uint8_t i = 1; i < 5; i++) {
    sensor[i] = analogRead(SensorPin[i]);
    map_result[i - 1] = map(sensor[i], black, white, 0, 1023); //range of a analog sensor
    avg += map_result[i - 1] * i;
    sum += map_result[i - 1];
  }
  return avg / sum;
}

void ReadSensor() {
  for (uint8_t i = 0; i < 6; i++) {
    sensor[i] = analogRead(SensorPin[i]);
  }
}
