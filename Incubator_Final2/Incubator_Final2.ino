#include <math.h>

int interruptPin1 = 2;
double temperature = 0;
volatile bool prevState = HIGH;  // previous state of light bulb
volatile bool currState = HIGH;  // current state of light bulb
float startCount = 0;
float endCount = 0;
double threshold = 32.0;

float SC;
float EC;
float duration;
float flip;

int fan = 0;

double FindTemperature(int rawADC) {
    double temp;  
    temp = log(10000.0*((1024.0/rawADC-1)));
    temp = 1 / (0.001129148 + (0.000234125 * temp) + (0.0000000876741 * temp * temp * temp));
    temp = temp - 273.15; 
    return temp;
}

void setup() {
  noInterrupts();  // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 65000;           
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B = ( _BV(CS02) | _BV(CS00));
  
  TIMSK1 |= (1 << OCIE1A);
 
  interrupts();  
  Serial.begin(9600);
  
    //Setup Channel A
  pinMode(12, OUTPUT);  // initiates Motor Channel A pin
  pinMode(9, OUTPUT);  // initiates Brake Channel A pin
  pinMode(10, OUTPUT);  // light bulb
  pinMode(5, OUTPUT);  // pin connected to pin 2 (interrupt pin)

  pinMode(13, OUTPUT); //Initiates Motor Channel B pin
  pinMode(8, OUTPUT);  //Initiates Brake Channel B pin
  
  pinMode(6, OUTPUT);  // green LED
  pinMode(7, OUTPUT);  // red LED
    
  digitalWrite(10, HIGH);  // turn light bulb on
  digitalWrite(5,HIGH);
  digitalWrite(6, HIGH);  // turn green LED on
  digitalWrite(7,LOW);  // turn red LED off
    
  attachInterrupt(digitalPinToInterrupt(interruptPin1),handler,FALLING);
}
void loop() {
    SC = millis();
    startCount = 0;
    endCount = 0;
    
    temperature = FindTemperature(analogRead(2));  // calculate temperature in degrees Celsius
    Serial.print("Celsius: ");
    Serial.println(temperature);
    
    if(temperature >= threshold && prevState == currState) {
      digitalWrite(5, LOW);  // pin 5 is connected to pin 2 --> an interrupt is generated
      digitalWrite(10, currState);  // turn light bulb off
      digitalWrite(7, !currState);  // turn red LED on
      digitalWrite(6, currState);  // turn green LED off
      digitalWrite(12, !currState); // establishes forward direction of Channel A
      digitalWrite(9, currState);   // disengages the Brake for Channel A
      analogWrite(3, 255);   // spins the motor on Channel A at full speed
      
      startCount = millis();
      endCount = millis();
      
      while(endCount - startCount < 7500) {
        endCount = millis();
      }
      
    }
    else if(prevState != currState && temperature >= threshold) {
      // do nothing
    }
    else if(prevState != currState && temperature < threshold) {
      digitalWrite(10, HIGH); // turn light bulb on
      digitalWrite(5, HIGH);
      digitalWrite(6, HIGH);  // turn green LED on
      digitalWrite(7, LOW);  // turn red LED off
      
      currState = HIGH;
      prevState = currState;
      
      startCount = millis();
      endCount = millis();
      
      while(endCount - startCount < 7500) {
        endCount = millis();
      }
    }
    
    EC = millis();
    duration = EC - SC + duration;
    flip = (float)duration/1000;
    //Serial.println(flip);
    if(flip > (float)40){
      digitalWrite(13, HIGH);  //Establishes backward direction of Channel B
      digitalWrite(8, LOW);   //Disengage the Brake for Channel B
      analogWrite(11, 70);    //Spins the motor on Channel B at half speed

      startCount = millis();
      endCount = millis();
      
      while(endCount - startCount < 1000) {
        endCount = millis();
      }
      
      digitalWrite(8, HIGH);
      SC = (float)0;
      EC = (float)0;
      duration = (float)0;
      flip = (float)0;
    }
}

// interrupt handler function 
void handler() {
    prevState = HIGH;
    currState = LOW;
}

ISR(TIMER1_COMPA_vect){
  if(currState == HIGH) {
    fan++;
    if(fan == 1){
      digitalWrite(12, HIGH); // establishes forward direction of Channel A
      digitalWrite(9, LOW);   // disengages the Brake for Channel A
      analogWrite(3, 225);   // spins the motor on Channel A at full speed*/
    }
    if(fan == 2){
      digitalWrite(9, HIGH); //Engage the Brake for Channel A
      fan = 0;
    }
  }
}
