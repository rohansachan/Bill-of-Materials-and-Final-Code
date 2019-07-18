//motor A connected between A01 and A02

#define VIN A5 // define the Arduino pin A0 as voltage input (V in)

int state = 1;

const int green = 5;
const int red = 2;

int greenbutton = 0;
int redbutton = 0;

int STBY = 10; //standby

//Motor A
int PWMA = 3; //Speed control
int AIN1 = 9; //Direction
int AIN2 = 8; //Direction
int analogPin = A5; // potentiometer wiper (middle terminal) connected to analog pin 3
                    // outside leads to ground and +5V

const float VCC   = 5.0;// supply voltage is from 4.5 to 5.5V. Normally 5V.

float current = 0;

const float QOV =   0.5 * VCC;// set quiescent Output voltage of 0.5V
float voltage;// internal variable for voltage
const int model = 0;   // enter the model number (see below)

float sensitivity[] ={
          0.185,// for ACS712ELCTR-05B-T
          0.100,// for ACS712ELCTR-20A-T
          0.066// for ACS712ELCTR-30A-T
     
         }; 

void setup(){
pinMode(STBY, OUTPUT);

pinMode(PWMA, OUTPUT);
pinMode(AIN1, OUTPUT);
pinMode(AIN2, OUTPUT);
pinMode(analogPin, INPUT);

Serial.begin(9600);// initialize serial monitor

Serial.println("Robojax Tutorial");
Serial.println("ACS712 Current Sensor");

  pinMode (green, INPUT);
  pinMode (red, INPUT);
 
}

void loop(){

greenbutton = digitalRead (green);
redbutton = digitalRead (red);

if (state == 1){
  stop();
  Serial.println(state);
  if(greenbutton == LOW) {
  state = 2;
  }

while (redbutton == HIGH){
  move(1, 255, 1);
  redbutton = digitalRead (red);

}
  
}

  if (state == 2){
    Serial.println(state);
    move(1, 255, 0); //motor 1, full speed, left
    if (current>13)
    state=3;
    if (redbutton == HIGH) {
    state=1;
    }
  }
  
  if (state == 3){
    Serial.println(state);
    stop();
    if (redbutton == HIGH)
    state=1;
  }


  float voltage_raw =   (5.0 / 1023.0)* analogRead(VIN);// Read the voltage from sensor
  Serial.println(voltage_raw);
  voltage =  voltage_raw - QOV + 0.012 ;// 0.000 is a value to make voltage zero when there is no current
  Serial.println(voltage);
  float current = voltage / sensitivity[model];
  Serial.println(current);
    //if(abs(current) > cutOffLimit ){
    Serial.print("V: ");
    Serial.print(voltage,3);// print voltage with 3 decimal places
    Serial.print("V, I: ");
    Serial.print(current,2); // print the current with 2 decimal places
    Serial.println("A");

}


void move(int motor, int speed, int direction){
//Move specific motor at speed and direction
//motor: 0 for B 1 for A
//speed: 0 is off, and 255 is full speed
//direction: 0 clockwise, 1 counter-clockwise

digitalWrite(STBY, HIGH); //disable standby

boolean inPin1 = LOW;
boolean inPin2 = HIGH;

if(direction == 1){
inPin1 = HIGH;
inPin2 = LOW;
}

if(motor == 1){
digitalWrite(AIN1, inPin1);
digitalWrite(AIN2, inPin2);
analogWrite(PWMA, speed);
//}else{
//digitalWrite(BIN1, inPin1);
//digitalWrite(BIN2, inPin2);
//analogWrite(PWMB, speed);
}
}

void stop(){
//enable standby
digitalWrite(STBY, LOW);
}
