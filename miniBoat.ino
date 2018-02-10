/* two channels RC boat 
 *  
 *  Servo is rudder 
 *  Motor is, well, motor
 *  
 *  Both receive RC controller input and use a getRCChannel to get the values with limits and smoothing
 *  
*/

int servoPin = 9; 
int motorPin = 11; 

#include <Servo.h>
#define in1 6
#define in2 7

Servo myservo;  // create servo object to control a servo


int rud = 0;
int acc = 0;
int raw; 
int min1, c1, max1, min2, c2, max2;  

int pos = 0;    // variable to store the servo position

void setup() {
  pinMode(motorPin, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  Serial.begin(9600);
  myservo.attach(servoPin);  // attaches the servo on pin 9 to the servo object
  myservo.write(90);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(motorPin, 0);

  pinMode(12, INPUT); // Set our input pins as such 
  pinMode(13, INPUT); // Set our input pins as such 


  min1 = 1400; max1 = 1500; 
  min2 = 1400; max2 = 1500; 
  
  Serial.begin(9600); // Pour a bowl of Serial
  rud = getRCChannel(12, &min1, &max1, 30, 150, rud, 1, 0, &raw); 
  acc = getRCChannel(13, &min2, &max2, -255, 255, acc, 1, 0, &raw); 

  Serial.println("callibrating");

  c1 = getChannelAvg(12, 1000);
  c2 = getChannelAvg(13, 3000); 

  Serial.println(c1);
  Serial.println(c2);
  
//calibrateChannel(12, &min1, &c1, &max1); 

  
  Serial.println("start");
  delay(3000);
}

void throtlleForward(){  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);};
void throtlleBackward(){  digitalWrite(in2, LOW); digitalWrite(in1, HIGH);};
void motorSpeed(int s) {  digitalWrite(motorPin, abs(s)); }
void rudder(int deg) {myservo.write(deg);}

int r1; 
void loop() {
  rud = getRCChannel(12, &min1, &max1, 30, 150, rud, 0.7, 1, &r1); 
  
  delay(15);
  acc = getRCChannel(13, &min2, &max2, -255, 255, acc, 0.7, 2, &raw); 
  if (raw > c2) { 
     throtlleForward();
  } else {
    throtlleBackward();
    acc = map(raw, min2, max2, 0, 100); 
  }
  if (abs(raw-c2) < ((max2-min2)/10)) {acc = 0;}
  
  Serial.println(rud);
  Serial.println(acc);
  motorSpeed(acc);
  rudder(rud);
    
  delay(15);
   
}

int getChannelAvg(int ch, int timeout) {
   int value = pulseIn(ch, HIGH, 25000); 
   int oldValue = value;
   float alpha = 0.7;
   long now = millis();
  while (millis() - now <= timeout) {
      value = pulseIn(ch, HIGH, 25000); 
      value = alpha * value + (1-alpha) * oldValue;
      delay(50);
  }
  return value; 

}
void calibrateChannel(int ch, int*mi, int*c, int*ma){
  int timeout = 5000; 
  float alpha = 0.7;
  Serial.println("Calibration: channel " + String(ch)); 
  
  Serial.println("keep control centered"); 
  delay(5000);
  long now = millis(); 
  long sec = millis(); 
  int value = pulseIn(ch, HIGH, 25000); 
  int oldValue = value;
  Serial.println("start"); 
  while (millis() - now <= timeout) {
      value = pulseIn(ch, HIGH, 25000); 
      value = alpha * value + (1-alpha) * oldValue;
      if (millis() - sec > 1000) {
        Serial.println(timeout - (millis()-now));
        sec = millis();
      }
  }
  Serial.println("done");
  Serial.println ("Center: " + String(value)); 
  *c = value;

  
  Serial.println("keep control at MINIMUM"); 
  delay(5000);
  now = millis(); 
  sec = millis(); 
  value = pulseIn(ch, HIGH, 25000); 
  oldValue = value;
  Serial.println("start"); 
  while (millis() - now <= timeout) {
      value = pulseIn(ch, HIGH, 25000); 
      value = alpha * value + (1-alpha) * oldValue;
      if (millis() - sec > 1000) {
        Serial.println(timeout - (millis()-now));
        sec = millis();
      }
  }
  Serial.println("done");
  Serial.println ("Min: " + String(value));
  *mi = value; 

  Serial.println("keep control at MAXIMUM"); 
  delay(5000);
  now = millis(); 
  sec = millis(); 
  value = pulseIn(ch, HIGH, 25000); 
  oldValue = value;
  Serial.println("start"); 
  while (millis() - now <= timeout) {
      value = pulseIn(ch, HIGH, 25000); 
      value = alpha * value + (1-alpha) * oldValue;
      if (millis() - sec > 1000) {
        Serial.println(timeout - (millis()-now));
        sec = millis();
      }
  }
  Serial.println("done");
  Serial.println ("Max: " + String(value)); 
  *ma = value;
}


int getRCChannel(byte chPin, int *inMin, int *inMax, int outMin, int outMax, int lastVal, float alpha, byte diff, int *raw){
/*
 * chPin - where is the RC signal pin connected to 
 * inMin, inMax - input limits, coming from the remote. may vary between systems and configurations. Can be measured with function calibrateChannel 
 * outMin, outMax - output limits to which the input will be mapped. 
 * lastVal - the last OUTPUT value (used for smoothing) 
 * alpha - float used for mixing read value with previous value. Alpha is the weight of current value 
 *         (1 = ignore past value, 0 = ignore current value [which doesn't make any sense of course] 
 * diff - a minimal change to consider a value different from previous. In absolut numbers. To clean noisy signal (i.e. cheap potentiometer) 
 * raw  - just a pointer to give the requesting code raw values too, just for lolz.
 */
  
   int ch = pulseIn(chPin, HIGH, 25000); 
   *raw = ch; 
   int lastIn = map(lastVal, outMin, outMax, *inMin, *inMax); 
  
   if ((ch < *inMin) && ((*inMin / ch) <= 1.2)) *inMin = ch; 
   if ((ch > *inMax) && ((ch / *inMax) <= 1.2)) {*inMax = ch;} 
   
     ch = alpha * ch + (1-alpha) * lastIn;
   if (ch < *inMin || ch > *inMax) return lastVal; 
   ch = map(ch, *inMin, *inMax, outMin, outMax);    
   
   if (abs(ch - lastVal) <= diff) return lastVal; 
   return ch;
}

