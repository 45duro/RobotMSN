
#include <Arduino.h>
#include "BasicStepperDriver.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 64
#define RPM 400

// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 1

// All the wires needed for full functionality

#define DIR 3
#define STEP 6

//Uncomment line to use enable/disable functionality
//#define SLEEP 13

// 2-wire basic config, microstepping is hardwired on the driver
BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP);

//Uncomment line to use enable/disable functionality
//BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP, SLEEP);

void setup() {
    Serial.begin(9600);
    pinMode(8,OUTPUT);
    stepper.begin(RPM, MICROSTEPS);
    // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next line
    // stepper.setEnableActiveState(LOW);
    digitalWrite(8,1);  
}

char dato = 0;

void loop() {

    if(Serial.available()>0){
      dato = Serial.read();
      Serial.println(dato);
    }

    if(dato == 'a'){
      Serial.println("entro");
      digitalWrite(8,0);
      // energize coils - the motor will hold position
      // stepper.enable();
    
      /*
       * Moving motor one full revolution using the degree notation
       */
      stepper.move(convertirGrados(45/2)); 
      //stepper.move(convertirGrados(60));
      //stepper.rotate(180);
  
      /*
       * Moving motor to original position using steps
       */
      //stepper.move(-254.75);
      //stepper.rotate(-180);
      //stepper.move(-MOTOR_STEPS*MICROSTEPS);
  
      // pause and allow the motor to be moved by hand
      // stepper.disable();
  
      digitalWrite(8,1);  
    }

    if(dato == 'b'){
      Serial.println("entro");
      digitalWrite(8,0);
      // energize coils - the motor will hold position
      // stepper.enable();
    
      /*
       * Moving motor one full revolution using the degree notation
       */
      //stepper.move(convertirGrados(-60));
      stepper.move(convertirGrados(-45/2)); 
      digitalWrite(8,1);
    }
}

int convertirGrados(short degree){
  int result = 22.6444*degree;
  return result;
}
