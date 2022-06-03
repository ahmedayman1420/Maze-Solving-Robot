/*------ Arduino Line Follower Code----- */

/*-------defining analog sensors Inputs------*/
#define LS A0      // left analog sensor      (analog in)
#define MS A1      // middle analog sensor      (analog in)
#define RS A2      // right analog sensor      (analog in)
#define MRS A3 //MAZE RIGHT
#define MLS A4 //MAZE LEFT
/*-------defining motor Outputs------*/
#define LM1 8       // left motor      (digital out)

#define LM2 7      // left motor      (digital out)

#define RM1 3       // right motor      (digital out)

#define RM2 2       // right motor      (digital out)

#define ENL 5       // left motor enable      (analog out)

#define ENR 6       // right motor  enable      (analog out)


/*--------sensor readings--------*/
int farleft = 0;
int farright = 0;

int leftSensor =0;
int middleSensor =0;
int rightSensor =0;

/*--------PID Constants--------*/
double Kp = 0.038;  //response to errors  
double Kd = 0.76;   //response to change of errors
double Ki = 0.001;       //response to the acumalation of errors    
double error = 0.0, P = 0.0, I = 0.0, D = 0.0, PID_value = 0.0;
/*--------Motor Constants--------*/
uint8_t startSpeed = 75;  //start speed
uint8_t minSpeed = 0;  //start speed
uint8_t maxSpeed = 150;  //start speed
/*-----------settup&loop Functions--------*/
void setup()

{
  pinMode(LS, INPUT);
  pinMode(MS, INPUT);
  pinMode(RS, INPUT);
  pinMode(MRS, INPUT);
  pinMode(MLS, INPUT);


  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
  pinMode(ENL, OUTPUT);
  pinMode(ENR, OUTPUT);


  _delay_ms(2000);  //waits for 2 sec upon starting to have time to center the car on the track
  Serial.begin(9600); //for testing
  startMotors();
}


void loop()

{
   leftSensor = analogRead(LS);
  middleSensor = analogRead(MS);
  rightSensor = analogRead(RS);

  farleft = analogRead(MLS) / 512;
  farright = analogRead(MRS) / 512;

    bool condition = leftSensor > 600 && middleSensor >600 && rightSensor > 600 && farleft == HIGH && farright == HIGH;

   if (farright == HIGH && farleft == HIGH)
  {
    int e1 = middleSensor-rightSensor;
     int e2 = leftSensor-middleSensor;
    error = (e1+e2)/2.0;
    Serial.println(error);
   }
   else if (farleft == LOW && farright == LOW)
   {
    mazeLeft();
   }
   else if (farright == LOW && farleft == HIGH)
   {
    mazeRight();
   }
   else if(farleft==LOW && farright == HIGH)
   {
    mazeLeft();
    }
    else if(condition)
    {
      mazeLeft();
      }
  
  calcError();
  calcPID();
  motorControl();
//  digitalWrite(LM1, LOW);
//  digitalWrite(LM2, HIGH);
//  digitalWrite(RM1, LOW);
//  digitalWrite(RM2, HIGH);
//    analogWrite(ENL, 200); //Left Motor Speed

  

}
/*-----------FUNCTIONS-----------*/
void startMotors()
{
  /*
  * we start the motors to go always in forward direction,as we will controll it's 
  * speed via the enable port
  */
  digitalWrite(LM1, LOW);
  digitalWrite(LM2, HIGH);
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, HIGH);
}
void calcError()
{
  //assuming I read (1023-->white & 0-->black)
  leftSensor = analogRead(LS);
  middleSensor = analogRead(MS);
  rightSensor = analogRead(RS);

  farleft = analogRead(MLS) / 512;
  farright = analogRead(MRS) / 512;


  //_delay_ms(500);
    Serial.print(farleft);
    Serial.print("\t");
    Serial.print(leftSensor);
    Serial.print("\t");
    Serial.print(middleSensor);
    Serial.print("\t");
    Serial.print(rightSensor);
    Serial.print("\t");
    Serial.print(farright);
    Serial.print("\t");
    Serial.println('\n');
  

  /*
   * we subtract the error by sub the readings
   * e1 = RS-MS
   * e2 = MS-LS
   * e = (e1+e2)2

   * GOAL is to get MS=high  & LS=low &RS=low------(always in a centered position)
   * e1= -ve
   * e2= +ve
   * e = 0
  */
 
  
}
void calcPID()
{
  D = error - P;   //current_error-prev_error
  P = error;      //current_error
  I = I + error;  //cum_sum of errors

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

}

void motorControl()
{
  /*
  * so,if e > 0 : this means that my car went right
  *               , so to get it back to center we need to fast the right wheel 
  * if e < 0 : this means that my car went left
  *             , so to get it back to center we need to fast the left wheel 
  *
  */
  analogWrite(ENL, constrain((startSpeed + PID_value), minSpeed, maxSpeed)); //Left Motor Speed
  analogWrite(ENR, constrain((startSpeed - PID_value), minSpeed, maxSpeed)); //Right Motor Speed

    
    Serial.print((startSpeed - PID_value));
    Serial.print("\t");
    Serial.print(PID_value);
    Serial.print("\t");
    Serial.print(constrain((startSpeed - PID_value), minSpeed, maxSpeed));
    Serial.print("\t");
    Serial.println(constrain((startSpeed + PID_value), minSpeed, maxSpeed));
  
  
}

void mazeRight()
{
    stopMove();
  _delay_ms(200);

      leftSensor = analogRead(LS);
  middleSensor = analogRead(MS);
  rightSensor = analogRead(RS);

  farleft = analogRead(MLS) / 512;
  farright = analogRead(MRS) / 512;
  bool condition = leftSensor > 600 && middleSensor >600 && rightSensor > 600 && farleft == HIGH && farright == HIGH;
  if (!condition)
    return;
  
//     farright = analogRead(MRS) / 512;
//
//    while(farright == HIGH)
//  {
//    moveBackward();
//     farright = analogRead(MRS) / 512;
//
//    }
//    startMotors();
//      stopMove();
//  _delay_ms(200);
  bool isStop=false;

  digitalWrite(LM1, LOW);
    digitalWrite(LM2, HIGH);
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW);
    analogWrite(ENL, startSpeed); //Left Motor Speed
    analogWrite(ENR, startSpeed); //Right Motor Speed
  _delay_ms(100);

  while(!isStop)
  {
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, HIGH);
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW);
    analogWrite(ENL, startSpeed); //Left Motor Speed
    analogWrite(ENR, startSpeed); //Right Motor Speed

      leftSensor = analogRead(LS);
    middleSensor = analogRead(MS);
    rightSensor = analogRead(RS);
  
    farleft = analogRead(MLS) / 512;
    farright = analogRead(MRS) / 512;
    isStop = farleft == HIGH && middleSensor < 600 && rightSensor>500 && farright == HIGH;
   }


   stopMove();
   startMotors();

  _delay_ms(200);

     
  }
void mazeLeft()
{
  stopMove();
//  _delay_ms(200);
//  farleft = analogRead(MLS) / 512;
//
//  while(farleft == HIGH)
//  {
//    moveBackward();
//    farleft = analogRead(MLS) / 512;
//
//    }
//    startMotors();
//      stopMove();
  _delay_ms(200);
    bool isStop=false;

  digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, HIGH);
    analogWrite(ENL, startSpeed); //Left Motor Speed
    analogWrite(ENR, startSpeed); //Right Motor Speed

    _delay_ms(100);
  while(!isStop)
  {
    digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, HIGH);
    analogWrite(ENL, startSpeed); //Left Motor Speed
    analogWrite(ENR, startSpeed); //Right Motor Speed

      leftSensor = analogRead(LS);
    middleSensor = analogRead(MS);
    rightSensor = analogRead(RS);
  
    farleft = analogRead(MLS) / 512;
    farright = analogRead(MRS) / 512;
    isStop = farleft == HIGH && leftSensor>500&& middleSensor < 600  && farright == HIGH;
   }


   stopMove();
   startMotors();
  _delay_ms(200);


  }

  void stopMove()
  {
        analogWrite(ENL, 0); //Left Motor Speed
    analogWrite(ENR, 0); //Right Motor Speed
    }

  void  moveBackward()
    {
     digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW);
    analogWrite(ENL, startSpeed); //Left Motor Speed
    analogWrite(ENR, startSpeed); //Right Motor Speed
      
      }
