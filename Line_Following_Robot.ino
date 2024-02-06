//*********************************************************************
//Defining Hardware Pins
//*********************************************************************

#define M1A 3 //M1A is connected to PWB pin 3
#define M1B 5 //M1B is connected to PWB pin 5
#define M2A 4 //M2A is connected to PWB pin 4
#define M2B 2 //M2B is connected to PWB pin 2

#define IR_sensor_1D 18 //Defining Digital Pin For IR sensor 1
#define IR_sensor_1A A0 //Defining Analog Pin for IR sensor 1

#define IR_sensor_2D 19 //Defining Digital Pin For IR sensor 2
#define IR_sensor_2A A1 //Defining Analog Pin for IR sensor 2

//*********************************************************************
//Defining Global Variables
//*********************************************************************

#define motor_go 80 //Defining motor go speed
#define motor_stop 0 //Defining motor stop speed

int IR_data_left = 0;
int IR_data_right = 0;

//*********************************************************************
//Setup Function
//*********************************************************************

void setup() {

  pinMode(IR_sensor_1D, INPUT); //Setting IR_sensor_1D as input
  pinMode(IR_sensor_1A, INPUT); //Setting IR_sensor_1A as input

  pinMode(IR_sensor_2D, INPUT); //Setting IR_sensor_2D as input
  pinMode(IR_sensor_2A, INPUT); //Setting IR_sensor_2A as input

  attachInterrupt(digitalPinToInterrupt(IR_sensor_1D), IR_data, FALLING); //Digital Interrupt for IR 1
  attachInterrupt(digitalPinToInterrupt(IR_sensor_2D), IR_data, FALLING); //Digital Interrupt for IR 2  

  Serial.begin(9600);

}

//*********************************************************************
//Loop Function
//*********************************************************************

void loop() {

  delay(1000);
  
  //****FOR LOOP to determine robot direction****
  if (IR_data_left = 1 && IR_data_right = 1){ //Go straight

    motor_drive(motor_go, motor_stop, motor_go, motor_stop);
    
  }else if(IR_data_left = 1 && IR_data_right = 0){ //Turn Left

    motor_drive(motor_stop, motor_stop, motor_go, motor_stop);

  }else if(IR_data_left = 0 && IR_data_right = 1){ //Turn Right

    motor_drive(motor_go, motor_stop, motor_stop, motor_stop);

  }else{ //Stop

    motor_drive(motor_stop, motor_stop, motor_stop, motor_stop);
    
  }//****END OF FOR LOOP***

}

//*********************************************************************
//Function to Read IR Sensor Data
//*********************************************************************

int IR_data(int sensor){
  
  int value_A1 = analogRead(IR_sensor_1A);  //Reading analog value for IR left
  int value_A2 = analogRead(IR_sensor_2A);  //Reading analog value for IR right
  int value_D1 = digitalRead(IR_sensor_1D); //Reading digital value for IR left
  int value_D2 = digitalRead(IR_sensor_2D); //Reading digital value for IR left

  int IR_ratio = value_A1/value_A2;

  if(value_2D == 1 || value_2D == 1){

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



















