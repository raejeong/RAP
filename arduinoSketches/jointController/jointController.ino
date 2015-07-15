/*
 * Joint Controller
 * Runs on every actuator
 * Rae Jeong :: raychanjeong@gmail.com :: MakeLab
 */

#include <Encoder.h> 
#include <SoftwareSerial.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

const int limitSwitchPin = 7; // Assigning pin 7 for the limit switch. Note that pin 7 has the internal pull up
const int encPinA = 2;
const int encPinB = 3;
const int mySerialPinRX = 4;
const int mySerialPinTX = 5;

Encoder myEnc(encPinA, encPinB); // Reading Encoder data from pin 2 and 3. Note that we use 2 and 3 on an UNO because these are the interupt pins 

SoftwareSerial mySerial(mySerialPinRX, mySerialPinTX); // Software Serial for communicating with the Syren 25A motor controller at baud rate of 9600, using pin 4 and 5.

double setPoint;
double input;
double output;

PID myPID(&input, &output, &setPoint, 0.5, 1, 0.7, DIRECT);

boolean limitSwitchLastReading; // Last reading of the limit switch 
uint32_t limitSwitchLastDebounceTime = 0; // Starting the debounce time at 0
const uint32_t limitSwitchDebounceDelay = 50; // Setting the debounce delay time in milliseconds
boolean limitSwitchState = true; // State of the limit switch for debouncing
boolean limitSwitchReading; // Current reading of the limit switch
uint8_t limitSwitchCount = 0;

std_msgs::String test_msg;
//std_msgs::String test_msg_cb;
ros::Publisher pub_test_msg("test", &test_msg);
ros::NodeHandle nh;


/*
 * motorControllerCommand
 * Gobal variable to store the value sent to the motor controller.
 * maxForward: 255 maxReverse: 0 motorStop: 127
 * maxForward, maxReverse and motorStop variables are used in the calibration
 * Direction was chosen based on the encoder direction
 */
const uint8_t motorControllerCommand = 255;
const uint8_t maxForward = 255;
const uint8_t maxReverse = 0;
const uint8_t motorStop = 127; 

/*
void test_cb(const std_msgs::UInt16& cmd_cb) {
  String test_cb_string = String((int)cmd_cb.data);
  const char *test_cb_char = test_cb_string.c_str();
  test_msg_cb.data = test_cb_char;
  pub_test_msg.publish(&test_msg_cb);
}

ros::Subscriber<std_msgs::UInt16> sub("test_cb", test_cb);
*/

void setup()
{
  Serial.begin(9600); // Serial communication start.
  mySerial.begin(9600); // Serial commuication to motor controller start.

  pinMode(limitSwitchPin, INPUT); 
  
  digitalWrite(limitSwitchPin, HIGH); // Enabling the internal pull up
  
  limitSwitchLastReading = digitalRead(limitSwitchPin); // Reading the limit switch data in

  nh.initNode();
  nh.advertise(pub_test_msg);
  //  nh.subscribe(sub);

  input = encoderRead();
  setPoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(maxReverse-motorStop, maxForward-motorStop);
  
  calibration(); // Running the calibration code on every start up

}

void loop()
{
  input = encoderRead();
  
  String test_string = String("output: " + String((int)output+127, DEC));
  test_string = String(test_string + " input: ");
  test_string = String(test_string + String((int)input, DEC));
  test_string = String(test_string + " setPoint: ");
  test_string = String(test_string + String((int)setPoint, DEC));
  const char *test_char = test_string.c_str();
  
  test_msg.data = test_char;

  pub_test_msg.publish(&test_msg);

  myPID.Compute();
  mySerial.write(output+127);

  nh.spinOnce();

  delay(100);
}




/*
 * When called, limitSwitch() returns true when the limit switch is pressed 
 * and false when the limit switch is not pressed.
 */
boolean limitSwitch()
{
  limitSwitchReading = digitalRead(limitSwitchPin);
  
  if(limitSwitchLastReading!=limitSwitchReading) {
    limitSwitchLastDebounceTime = millis();
    limitSwitchState = false;
  }
  
  limitSwitchLastReading = limitSwitchReading;
  
  if(!limitSwitchState && (millis() - limitSwitchLastDebounceTime) > limitSwitchDebounceDelay) {
    limitSwitchState = true;
  }
  return !limitSwitchReading;
  
}


/*
 * calibration() fucntion that is called every time the system starts up
 * Is responsible for hitting the limit switch and resetting the encoder to zero
 */
void calibration()
{
  // If the motor starts on the limitSwitch, go reverse until the limit switch is no longer triggered.
  if(limitSwitch()) { 
    while(limitSwitch()) {
      mySerial.write(maxReverse);
    }
  }
  
  // Go forward until the motor triggers the limit switch.
  while(!limitSwitch()) {
    mySerial.write(maxForward);
  }
  
  // Stop the motor
  mySerial.write(motorStop);

  // Reset the encoder. The delay is there becasue the motor does not stop instantaneously.
  delay(500);
  myEnc.write(0);
}


/*
 * mapFloat is meant to execute mapping for floats
 */
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/*
 * encoderRead() function is used to read the encoder value in degrees
 */
float encoderRead() {
  return mapFloat(myEnc.read(), 0, 5260, 0, 360);
}


