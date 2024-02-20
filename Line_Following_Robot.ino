
//*********************************************************************
//Importing Libraries
//*********************************************************************

#include <AccelStepper.h> //Library for the stepper motors

//*********************************************************************
//Defining Hardware Pins
//*********************************************************************

#define M1A 4 //M1A is connected to PWB pin 
#define M1B 5 //M1B is connected to PWB pin 
#define M2A 6 //M2A is connected to PWB pin
#define M2B 7 //M2B is connected to PWB pin

#define IR_sensor_1D 24 //Defining Digital Pin For IR sensor 1
#define IR_sensor_1A A0 //Defining Analog Pin for IR sensor 1

#define IR_sensor_2D 25 //Defining Digital Pin For IR sensor 2
#define IR_sensor_2A A1 //Defining Analog Pin for IR sensor 2

#define ultrasonic_trigger 22 //Defining Ultrasonic trigger pin
#define ultrasonic_echo 23 //Defining Ultrasonic Echo pin

#define encoder1_A 21 //Defining Encoder 1 signal A pin
#define encoder1_B 20 //Defining Encoder 1 signal B pin

#define encoder2_A 19 //Defining Encoder 2 signal A pin
#define encoder2_B 18 //Defining Encoder 2 signal B pin

//defining stepper motor pins
#define stepper1 8 
#define stepper2 9
#define stepper3 10
#define stepper4 11

#define switch1 2 //Limit switch 1
#define switch2 3 //Limit switch 2

//*********************************************************************
//Defining Global Variables
//*********************************************************************

#define motor_go 80 //Defining motor go speed
#define motor_stop 0 //Defining motor stop speed

//IR variables to determine which wheels must go
int IR_data_left = 0; 
int IR_data_right = 0;

int distance; //Declaring variable for distance calculated from ultrasonic sensor

volatile unsigned long encoder1_ACount = 0; //Declaring encoder 1 A signal count
volatile unsigned long encoder1_BCount = 0; //Declaring encoder 1 B signal count
volatile unsigned long encoder2_ACount = 0; //Declaring encoder 2 A signal count
volatile unsigned long encoder2_BCount = 0; //Declaring encoder 2 B signal count

#define encoder_Delay 1000 //Time between encoder speed measurements
float motor1_speed;  //Declaring variable for speed of the motor 1
float motor2_speed; //Declaring variable for speed of motor 2

int x = 0;  //Declaring variable for stepper motor homing
int y = 0;  //Declaring variable for stepper motor homing

int state = 1;  //Declaring variable for the state the machine will be in
//State 1 = Autonomous line following
//State 2 = Autonomous tennis ball into box 
//State 3 = Manual control

int position = 45;  //Position to close pincers

int incomingByte = 0; //Recieved data from hc-05

String direction;

//*********************************************************************
//Objects
//*********************************************************************

AccelStepper myStepper(AccelStepper::FULL4WIRE, stepper1, stepper2, stepper3, stepper4); //Object for stepper motors

//*********************************************************************
//Setup Function
//*********************************************************************

void setup() {

  Serial.begin(9600);

  pinMode(IR_sensor_1D, INPUT); //Setting IR_sensor_1D as input
  pinMode(IR_sensor_1A, INPUT); //Setting IR_sensor_1A as input

  pinMode(IR_sensor_2D, INPUT); //Setting IR_sensor_2D as input
  pinMode(IR_sensor_2A, INPUT); //Setting IR_sensor_2A as input

  pinMode(ultrasonic_trigger, OUTPUT); //Setting ultrasonic trigger pin as an Output
  pinMode(ultrasonic_echo, INPUT);  //Setting ultrasonic echo pin as an Input

  pinMode(encoder1_A, INPUT); //Setting encoder 1 signal A pin as Input
  pinMode(encoder1_B, INPUT); //Setting encoder 1 signal B pin as Input

  pinMode(encoder2_A, INPUT); //Setting encoder 2 signal A pin as Input
  pinMode(encoder2_B, INPUT); //Setting encoder 2 signal B pin as Input

  pinMode(switch1, INPUT_PULLUP); //Setting limit switch 1 as an input with a pull up resistor
  pinMode(switch2, INPUT_PULLUP); //Setting limit switch 2 as an input with a pull up resistor
  
  //*********************************************************************
  //Digital Interrupts
  //*********************************************************************
  
  //attachInterrupt(digitalPinToInterrupt(IR_sensor_1D), IR_data, FALLING); //Digital Interrupt for IR 1
  //attachInterrupt(digitalPinToInterrupt(IR_sensor_2D), IR_data, FALLING); //Digital Interrupt for IR 2  

  attachInterrupt(digitalPinToInterrupt(encoder1_A), Encoder1_ATick, RISING); //Digital interrupt for encoder 1 A signal count
  attachInterrupt(digitalPinToInterrupt(encoder1_B), Encoder1_BTick, RISING); //Digital interrupt for encoder 1 B signal count

  attachInterrupt(digitalPinToInterrupt(encoder2_A), Encoder2_ATick, RISING); //Digital interrupt for encoder 1 A signal count
  attachInterrupt(digitalPinToInterrupt(encoder2_B), Encoder2_BTick, RISING); //Digital interrupt for encoder 1 B signal count

  attachInterrupt(digitalPinToInterrupt(switch1), limit1, RISING);  //Digital interrupt for the limit switch 1
  attachInterrupt(digitalPinToInterrupt(switch2), limit2, RISING);  //Digital interrupt for the limit switch 2

  //Stepper Motor Speed Code
  myStepper.setMaxSpeed(1000.0); //Limits the value of setSpeed()
  myStepper.setSpeed(200.0); //runSpeed() will run the motor at this speed

  //Stepper Motor Homing Code
  while(x<1){
    myStepper.runSpeed(); //This will run the motor until homing is complete 
  }  
  while(y<1){
    myStepper.runSpeed(); //This will run the motor until homing is complete
  }
  if(x>=1 && y>=1){
    myStepper.setCurrentPosition(0);
  }
}

//*********************************************************************
//Loop Function
//*********************************************************************

void loop() {

  if(state == 1){

    IR_data();  //Function to collect IR data    
    robot_direction(); //Function to determine robot direction
    ultrasonic_data(); //Function to collect data from Ultrasonic sensor

  }
  else if(state == 2){
    IR_data();  //Function to collect IR data  

    if(IR_data_left == 1 && IR_data_right == 1){
      found_box();
    }
    else{
      robot_direction(); 
    }
     
  }
  
  bluetooth();  //Checking data recieved from hc-05
  motor_speed(); //Function to calculate motor speeds
  telemetry();  //Displaying telemetry data

}

//*********************************************************************
//Function to Read IR Sensor Data
//*********************************************************************

int IR_data(){
  
  int value_A1 = analogRead(IR_sensor_1A);  //Reading analog value for IR left
  int value_A2 = analogRead(IR_sensor_2A);  //Reading analog value for IR right
  int value_D1 = digitalRead(IR_sensor_1D); //Reading digital value for IR left
  int value_D2 = digitalRead(IR_sensor_2D); //Reading digital value for IR left

  //Serial.println(value_A1);
  //Serial.println(value_A2);

  int IR_ratio = value_A1/value_A2;

  if(value_D1 == 1 || value_D2 == 1){

    if(IR_ratio < 0.9){
      
      IR_data_left = 0;
      IR_data_right = 1;   

    }else if(IR_ratio>1.1){

      IR_data_left = 1;
      IR_data_right = 0; 

    }else{
      IR_data_left = 1;
      IR_data_right = 1;
    }
  }
  else{
    IR_data_left = 0;
    IR_data_right = 0;    
  }
}
//*********************************************************************
//Function to determine direction of Robot
//*********************************************************************

void robot_direction(){

  //****FOR LOOP to determine robot direction****
  if (IR_data_left == 1 && IR_data_right == 1){ //Go straight

    motor_drive(motor_go, motor_stop, motor_go, motor_stop);
    direction = "Straight";
    
  }else if(IR_data_left == 1 && IR_data_right == 0){ //Turn Left

    motor_drive(motor_stop, motor_stop, motor_go, motor_stop);
    direction = "Left";

  }else if(IR_data_left == 0 && IR_data_right == 1){ //Turn Right

    motor_drive(motor_go, motor_stop, motor_stop, motor_stop);
    direction = "Right";    

  }else{ //Stop

    motor_drive(motor_go, motor_stop, motor_go, motor_stop);  //Drive a tiny bit forward to ensure its not just a gap
    delay(10);
    motor_drive(motor_stop, motor_stop, motor_stop, motor_stop); //Stop
    direction = "Stopped";
    
  }//****END OF FOR LOOP**

}

//*********************************************************************
//Function to Drive Motors
//*********************************************************************

void motor_drive(int speed_M1A, int speed_M1B, int speed_M2A, int speed_M2B){

  //***Speed of left Motor***
  analogWrite(M1A, speed_M1A);
  analogWrite(M1B, speed_M1B);

  //***SPeed of Right Motor***
  analogWrite(M2A, speed_M2A);
  analogWrite(M2B, speed_M2B);
  
}

//*********************************************************************
//Function to Read Ultrasonic Sensor Data
//*********************************************************************

int ultrasonic_data(){

  //Sending a pulse through the trigger pin of the ultrasonic sensor
  digitalWrite(ultrasonic_trigger, LOW);
  delayMicroseconds(5);
  digitalWrite(ultrasonic_trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonic_trigger, LOW);

  //Collecting data from ultrasonic echo pin
  int echo_value = pulseIn(ultrasonic_echo, HIGH);  

  //Conversion to cm
  distance = (echo_value/2)/29.1; 

  if(distance<=3){
    motor_drive(motor_go, motor_stop, motor_go, motor_stop);
    delay(25);
    motor_drive(motor_stop, motor_stop, motor_stop, motor_stop);
    pincers(1);
  }
    
}

//*********************************************************************
//Encoder 1 A signal Count
//*********************************************************************

void Encoder1_ATick(){
  encoder1_ACount++; //Increment encoder A signal Count
}

//*********************************************************************
//Encoder 1 B signal Count
//*********************************************************************

void Encoder1_BTick(){
  encoder1_BCount++; //Increment encoder B signal Count
}

//*********************************************************************
//Encoder 2 A signal Count
//*********************************************************************

void Encoder2_ATick(){
  encoder2_ACount++; //Increment encoder A signal Count
}

//*********************************************************************
//Encoder 2 B signal Count
//*********************************************************************

void Encoder2_BTick(){
  encoder2_BCount++; //Increment encoder B signal Count
}

//*********************************************************************
//Motor Speed Function
//*********************************************************************

int motor_speed(){

  //Serial.print("A: " + String(encoder1_ACount));
  //Serial.print(" B: " + String(encoder2_ACount) + "\n");
  
  //Calculating the individual speed for each motor
  motor1_speed = (((encoder1_ACount)*(3.141592654)*(0.065))/1920)/0.01;
  motor2_speed = (((encoder2_ACount)*(3.141592654)*(0.065))/1920)/0.01;
  
  //Serial.print(" Speed: " +String(motor1_speed) + " mm/s\n");
  //Serial.print(" Speed: " +String(motor2_speed) + " mm/s\n");
  delay(encoder_Delay); //Delay to keep serial monitor readable

  //Resetting count variables
  encoder1_ACount = 0;
  encoder1_BCount = 0;

  encoder2_ACount = 0;
  encoder2_BCount = 0;
  
}

//*********************************************************************
//Function for limit switch 1 (Homing)
//*********************************************************************

void limit1(){

  x=1;  //Setting x=1 so to show Homing complete
  delay(1000);

}

//*********************************************************************
//Function for limit switch 2 (Homing)
//*********************************************************************

void limit2(){

  y=1;  //Setting y=1 to show Homing complete
  delay(1000);
  
}

//*********************************************************************
//Function for Pincer Movement
//*********************************************************************

void pincers(int sign){
  /*for(int i = 0; i<1; i++){
    myStepper.runSpeed(); //This will run the motor until homing is complete    
    delay(1000);
    
  }  */
  
  myStepper.moveTo(sign*position); 
}

//*********************************************************************
//Function for When tennis ball is found
//*********************************************************************

void ball_found(){

  motor_drive(motor_go, motor_stop, motor_go, motor_stop);
  delay(25);
  motor_drive(motor_stop, motor_stop, motor_stop, motor_stop);
  pincers(1);

  state = 2;

}

//*********************************************************************
//Function for found box
//*********************************************************************

void found_box(){
  
  direction = "Reversing";
  pincers(-1);
  motor_drive(motor_stop, motor_go, motor_stop, motor_go);
  delay(100);
  motor_drive(motor_stop, motor_stop, motor_stop, motor_stop);
  
}

//*********************************************************************
//Bluetooth Function
//*********************************************************************

void bluetooth(){

  if (Serial.available() > 0) {

    incomingByte = Serial.read();

  }

  switch(incomingByte)

  {
    case '3':
      //Manual State
      state = 3;
    break;
    
    case '2':
      //Black Box Searching State
      state = 2;
    break;

    case '1':
      //Follow Line State 
      state = 1;
    break;

    if(state == 3){
      
      case 's':
        //Reverse
        motor_drive(motor_stop, motor_go, motor_stop, motor_go);
        incomingByte='*';
      break;
      
      case 'd':
        //Turn Right
        motor_drive(motor_go, motor_stop, motor_stop, motor_stop);
        incomingByte='*';
      break;

      case 'a':
        //Turn Left
        motor_drive(motor_stop, motor_stop, motor_go, motor_stop);
        incomingByte='*';
      break;

      case 'w':
        //Forward
        motor_drive(motor_go, motor_stop, motor_go, motor_stop);
        incomingByte='*';
      break;
        
    }
    delay(5000);

  }
  
}


//*********************************************************************
//Function to Display Telemetry Data Through Bluetooth
//*********************************************************************

void telemetry(){

  Serial.println("************************************");
  Serial.println("Speed :            Acceleration:    ");
  Serial.println("Direction: " + direction + "        Tennis ball:     ");
  Serial.println("Pincers:          State: " + String(state));
  Serial.println("************************************");
  
}



















