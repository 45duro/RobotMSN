#include <EEPROM.h>

#include <Arduino.h>
#include "BasicStepperDriver.h" // generic
#include "MultiDriver.h"
#include "SyncDriver.h"
#include "GFButton.h"

#define DEBUG 0

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 64
#define MICROSTEPS 1
#define MOTOR_X_RPM 400
#define MOTOR_Y_RPM 400
#define MOTOR_Z_RPM 400

// X motor
#define DIR_X 2
#define STEP_X 5
// Y motor
#define DIR_Y 3
#define STEP_Y 6


// Z motor
#define DIR_Z 4
#define STEP_Z 7


#define STOPPER_PIN_Y 10
#define STOPPER_PIN_Z 11
#define STOPPER_PIN_X 9

#define limPos 50

#define sizeBrazo 80
#define sizeAnteBrazo 80
#define phi 3.141593
#define SubidaVertical 25
/*************************************
  DECLARACION DE OBJETOS PRINCIPALES
*************************************/
//Declaracion de motores
BasicStepperDriver stepperX(MOTOR_STEPS, DIR_X, STEP_X);
BasicStepperDriver stepperY(MOTOR_STEPS, DIR_Y, STEP_Y);
BasicStepperDriver stepperZ(MOTOR_STEPS, DIR_Z, STEP_Z);
SyncDriver controller(stepperX, stepperY, stepperZ);

void ActivarMotores(boolean);
void goToHome();
boolean goToHome_X();



/*************************************
*************************************/

/*************************************
  VARIABLES GLOBALES
*************************************/
char dato = 0;
float grados[3] = {0, 0, 90};
float posiciones[limPos][3];
int gradosDecorador[3];
byte contador = 0;

short LecturaBotonGuardar; const byte BotonGuardar = A2;
boolean LecturaBotonGuardarEEPROM; const byte BotonGuardarEEPROM = A3;

// ****************************
//                            *
//          Joystick          *
//                            *
// ****************************
short Joystick1 [3] = {A1,A0, A7};
GFButton btn = GFButton(BotonGuardar, E_GFBUTTON_PULLUP_INTERNAL);

// ****************************
//                            *
//          EEPROM            *
//                            *
// ****************************
struct ObjetoPosiciones{
  int contadorDeDatos = 0;
  float MatrizPosiciones [limPos] [3];
};

// ****************************
//                            *
//        CINEMATICA          *
//                            *
// ****************************

//90 es el valor de inicio y 30 valor que viene de la matriz
// asi mismo en c puesto 90 valor de inicio y  - 20 valor de la matriz
float Bpuesto= 0, Cpuesto= 0; 
float outCinematicoB=0, outCinematicoC=0;
float radioC;
float x, y, teta =0, r;


float convertirGrados(short degree,  byte relacion = 1);
short movimientoJoyStick(short joy, byte pos, byte limCentral = 112);
//void movimientoJoyStick(short joy, BasicStepperDriver Motor, byte pos, byte limCentral = 112, byte relacion = 1);
void setup() {

  //Declaracion de Sensores y pulsadores
  pinMode(STOPPER_PIN_Y,INPUT_PULLUP);
  pinMode(STOPPER_PIN_Z,INPUT_PULLUP);
  pinMode(STOPPER_PIN_X,INPUT_PULLUP);

  pinMode(BotonGuardar, INPUT_PULLUP);
  pinMode(BotonGuardarEEPROM, INPUT_PULLUP);
  
  pinMode(8,OUTPUT);
  
  Serial.begin(115200);
  
  stepperX.begin(MOTOR_X_RPM, MICROSTEPS);
  stepperY.begin(MOTOR_Y_RPM, MICROSTEPS);
  stepperZ.begin(MOTOR_Z_RPM, MICROSTEPS);

  ActivarMotores(false);

  // Attach callbacks to the button object
  //btn.setPressHandler(button_on_press);
  btn.setHoldHandler(button_on_hold);
  //btn.setReleaseHandler(button_on_release);
  btn.setClicksHandler(button_on_click);

}

void loop() {
  if(Serial.available()>0){
    dato = Serial.read();
    Serial.println(dato);
  }

  switch(dato){
    case 'h':
      //Se limpian posiciones para evitar daños
      limpiarPosiciones();
      goToHome();
      dato=0;
      break;

   case 'b':
      //button.check();
      break;

    case 'o':
      ActivarMotores(0);
      break;
    
    case 'g':
      Serial.println(grados[0]);
      break;

    case 'p':
      ActivarMotores(1);
      break;

    case 'i':
      Serial.print(grados[0]); Serial.print("\t\t"); Serial.print(grados[1]); Serial.print("\t\t");  Serial.println(grados[2]);
      dato = 0; 
      break;
      
    case 'v':
      ActivarMotores(1);
      stepperY.move(convertirGrados(42)); 
      dato = 'o';
      break;

    case 'e':
      LecturaDeEEPROM();
      dato=0;
      break;
    
    case 'r':
      ActivarMotores(1);
      rutinaGeneral();
      detenerMotores();
      dato='h';
      break;


    case 'c':
      //ActivarMotores(1);
      cinematicaInv(38,83);
      dato = 0;
      break;

  }

  if(dato == 'j'){
    
      
      ActivarMotores(1);

      short JoyIzqX = analogRead(Joystick1[0]);
      short JoyIzqY = analogRead(Joystick1[1]);
      short JoyIzqZ = analogRead(Joystick1[2]);

      JoyIzqX = movimientoJoyStick(JoyIzqX, 0);
      JoyIzqY = movimientoJoyStick(JoyIzqY, 1);
      JoyIzqZ = movimientoJoyStick(JoyIzqZ, 2);
      
      
      //Movimientos de X Y Z sincronizados
       if(JoyIzqX != 0 || JoyIzqY != 0 || JoyIzqZ != 0 ){
         //controller.move(0, convertirGrados(auxJoy1[1]),convertirGrados(auxJoy1[2]));
         controller.move(convertirGrados(JoyIzqX, 2), convertirGrados(JoyIzqY), convertirGrados(JoyIzqZ));
         
          for(byte i = 0; i < 3; i++){
            Serial.print(grados[i]); Serial.print("   ");
          }
          Serial.println();
       }
       else{
          stepperX.stop();
          stepperY.stop();
          stepperZ.stop();
       }

      
      
      LecturaBotones();

      //Chequeando el boton 1 o joystick principal
      btn.process();

      //Viene con tres estados si es 2, guarde un punto con cinematica de subida
      //Si es un 1 guarde un punto de paso sin cinematica
      //si es -1 no haga nada
      if(LecturaBotonGuardar == 2 ){
        delay(300);
        if(contador == 0){
           Serial.println("Guardando Posiciones"); 
        }  

        for(byte i = 0; i < 3; i++)
          posiciones[contador][i]= grados[i];

        //Agregando la cinematica
        cinematicaInv(grados[1], grados[2]);
        posiciones[contador+1][0]= grados[0];
        posiciones[contador+1][1]= outCinematicoB;
        posiciones[contador+1][2]= outCinematicoC;
          

        for(byte i = 0; i < 3; i++){
          Serial.print(posiciones[contador][i]); Serial.print("\t\t");
        }
        Serial.println();
        contador >= limPos -1 ? contador = 0:contador+=2;
        //Romper el ciclo ya que desde la misma clase no se deja
        LecturaBotonGuardar =0;
      }

      //Doble CLICK
      else if(LecturaBotonGuardar == 1){
        delay(300);
        if(contador == 0){
           Serial.println("Guardando Posiciones"); 
        } 
        for(byte i = 0; i < 3; i++)
          posiciones[contador][i]= grados[i];

        for(byte i = 0; i < 3; i++){
          Serial.print(posiciones[contador][i]); Serial.print("\t\t");
        }
        Serial.println();
        contador >= limPos -1 ? contador = 0:contador+=1;

        //Romper el ciclo ya que desde la misma clase no se deja
        LecturaBotonGuardar = 0;
      }

      if(!LecturaBotonGuardarEEPROM){
        delay(300);
        GuardarEnEEPROM();
        dato=0;
      }
      
      

  }
  /*
  else if(dato == 'l'){
      static short matriz2 [limPos][3] ={
        {  -13 , 8 , 78  },
        { -16 , 10 , 68  },
        { -16 , 10 , 64  },
        { -16 , 10 , 64  },
        { -16 , 10 , 73  },
        { 17  , 20  , 73  },
        { 17  , 20  , 69  },
        { 17  , 20  , 69  },
        { 17  , 20  , 69  },
        { 17  , 20  , 69  },
        { 17  , 14  , 73  },
        { 17  , 14  , 73  },
        { 4 , 16  , 76  },
        { 4 , 19  , 78  },
        { 8 , 22  , 78  },
        { 8 , 22  , 71  },
        { 8 , 22  , 71  },
        { 8 , 22  , 78  },
        { 8 , 22  , 78  },
        { 2 , 16  , 71  },
        { 2 , 16  , 68  },
        { 2 , 16  , 75  },
        { 2 , 16  , 75  },
        { 27  , 16  , 75  },
        { 27  , 16  , 80  },
        { 27  , 29  , 81  },
        { 27  , 29  , 76  },
        { 27  , 29  , 76  },
        { 27  , 29  , 84  },
        { 27  , 29  , 88  },
        { -11 , 14  , 88  },
        { -38 , 1 , 67  },
        { -38 , -4  , 63  },
        { -38 , 3 , 60  },
        { -38 , -3  , 60  },
        { -38 , -5  , 64  },
        { -38 , -5  , 70  },
        { -38 , 7 , 70  },
        { -13 , 12  , 75  },
        { -13 , 14  , 80  },
        { -11 , 27  , 75  },
        { -24 , 27  , 75  },
        { -24 , 27  , 75  },
        { -24 , 27  , 82  },
        { -24 , -1  , 86  },
        { 103 , -1  , 86  },
        { 0 , 0 , 0 },
        { 0 , 0 , 0 },
        { 0 , 0 , 0 },
        { 0 , 0 , 0 }

      };

      for(byte fil = 0; fil < limPos; fil++){
        for(byte col = 0; col < 3; col++){
           posiciones[fil][col] = float(matriz2[fil][col]);
        }
      } 

      GuardarEnEEPROM();
      dato = 0;
  }*/
}

void detenerMotores(){
  stepperX.stop();
  stepperY.stop();
  stepperZ.stop();
}

void rutinaGeneral(){
  
  int xAnterior =0, yAnterior =0, zAnterior = 90;

  for(byte i = 0; i < limPos; i++){

    int x = posiciones[i][0];
    int y = posiciones[i][1];
    int z = posiciones[i][2];

    Serial.print(F("G")); Serial.print(i);
    Serial.print(F("\tx: ")); Serial.print(x);
    Serial.print(F("\ty: ")); Serial.print(y);
    Serial.print(F("\tz: ")); Serial.println(z);

    if(((y-yAnterior) < -50 || (y-yAnterior) > 50 || (z-zAnterior) > 50 || (z-zAnterior) < -50) && ( i != 0 && i != limPos-1)){
      Serial.print(F("Error por desbordamiento en linea: ")); 
      Serial.print(i) ; 
      Serial.println(" se puede estrellar") ; 
    }
    else{
      stepperX.move(convertirGrados(x-xAnterior, 2));  
      stepperY.move(convertirGrados(y-yAnterior));  
      stepperZ.move(convertirGrados(z-zAnterior));  
      
      //Solucionar el tema de los angulos cuando se pasan y que de en recta
      delay(100);
      /*
      controller.move(0, 
                        convertirGrados(y-yAnterior),
                        convertirGrados(z-zAnterior));
      */

    }
    xAnterior = x; yAnterior = y; zAnterior = z;
   

    if(!xAnterior && !yAnterior && !zAnterior)
      delay(0);
    else
      delay(500);
  }
  
}



void LecturaDeEEPROM(){
  Serial.println("Obteniendo datos de EPROM");
  ObjetoPosiciones MiObjetoResultado;
  EEPROM.get(0, MiObjetoResultado);
  
  Serial.println(MiObjetoResultado.contadorDeDatos);
  for(int Filas = 0; Filas < MiObjetoResultado.contadorDeDatos; Filas++){
    for(int Columnas = 0; Columnas < 3; Columnas++){
      //Desempaquetar en posiciones
      posiciones[Filas][Columnas]=MiObjetoResultado.MatrizPosiciones [Filas][Columnas];
      Serial.print(MiObjetoResultado.MatrizPosiciones [Filas][Columnas]);
      Serial.print("\t\t");
      
    }
    Serial.println();
  }
}


void GuardarEnEEPROM(){
    Serial.println("Guardando en EEPROM");
    ObjetoPosiciones MiObjeto;
    MiObjeto.contadorDeDatos = limPos;
    for(byte Filas = 0; Filas < MiObjeto.contadorDeDatos; Filas++){
      for(int Columnas = 0; Columnas < 3; Columnas++){
        MiObjeto.MatrizPosiciones [Filas][Columnas] =  posiciones [Filas] [Columnas];
        Serial.print(MiObjeto.MatrizPosiciones [Filas][Columnas]);
        Serial.print("\t\t");
      }
      Serial.println();
    }
  
    EEPROM.put(0,MiObjeto);
    Serial.print("Tamaño final del objeto = ");
    Serial.println(sizeof(MiObjeto));
}


//Funcion para hablitar y desabilitar motores de una manera entendible
void ActivarMotores(bool Activar){

   pinMode(8,OUTPUT);
   // Simbolo ! para negar 
   digitalWrite(8,!Activar);
}



//Relacion de 1:2 = 2, Relacion de 1 a 1 = 1
float convertirGrados(short degree,  byte relacion = 1){
  float result;
  switch (relacion){
    case 1:
      result = 22.6444*degree;
      break;
    case 2:
      result = 11.3222*degree;
      break;
    
  }
  return result;
}

boolean goToHome_X(){
  ActivarMotores(true);
  if (digitalRead(STOPPER_PIN_X) == 0){
        //Serial.println("STOPPER REACHED");
        //stepperY.startBrake();
        stepperX.stop();
        return true;
  }else{

    unsigned wait_time_micros = stepperX.nextAction();
    return false;
  }

}

boolean goToHome_Y(){
  ActivarMotores(true);
  if (digitalRead(STOPPER_PIN_Y) == 1){
        //Serial.println("STOPPER REACHED");
        //stepperY.startBrake();
        stepperY.stop();
        return true;
  }else{

    unsigned wait_time_micros = stepperY.nextAction();
    return false;
  }
}

boolean goToHome_Z(){
  ActivarMotores(true);
  if (digitalRead(STOPPER_PIN_Z) == 1){
        //Serial.println("STOPPER REACHED");
        //stepperY.startBrake();
        stepperZ.stop();
        return true;
  }else{

    unsigned wait_time_micros = stepperZ.nextAction();
    return false;
  }
}

void goToHome(){
  ActivarMotores(1);
  boolean flag = false;

  //Setear Y
  stepperY.startMove(-100 * MOTOR_STEPS * MICROSTEPS);
  do{
    flag = goToHome_Y();
    if(flag){
      //Moverse ciertos grados hacia adelante
      stepperY.move(convertirGrados(15));
    }
  }while(!flag);

  

  //Setear Z
  stepperZ.startMove(-100 * MOTOR_STEPS * MICROSTEPS);
  do{
    flag = goToHome_Z();
    if(flag){
      //Moverse ciertos grados hacia adelante
      stepperZ.move(convertirGrados(30));
    }
  }while(!flag);

  

  //Setear X
  stepperX.startMove(100 * MOTOR_STEPS * MICROSTEPS);
  do{
    flag = goToHome_X();
    if(flag){
      //Moverse ciertos grados hacia adelante
      stepperX.move(convertirGrados(-115, 2));
    }
  }while(!flag);

  flag = false;

  ActivarMotores(0);

  
}

void limpiarPosiciones(){
  for(byte i=0; i < 3; i++){
    i == 2? grados[i]= 90 : grados[i]=0;
  }
}
