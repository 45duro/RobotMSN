#include <EEPROM.h>

#include <Arduino.h>
#include "BasicStepperDriver.h" // generic
#include "MultiDriver.h"
#include "SyncDriver.h"


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
#define SubidaVertical 20
/*************************************
  DECLARACION DE OBJETOS PRINCIPALES
*************************************/
//Declaracion de motores
BasicStepperDriver stepperX(MOTOR_STEPS, DIR_X, STEP_X);
BasicStepperDriver stepperY(MOTOR_STEPS, DIR_Y, STEP_Y);
BasicStepperDriver stepperZ(MOTOR_STEPS, DIR_Z, STEP_Z);
SyncDriver controller(stepperX, stepperY, stepperZ);

void movimientoJoyStick(short, BasicStepperDriver, byte, byte);
void ActivarMotores(boolean);
void goToHome();
boolean goToHome_X();



/*************************************
*************************************/

/*************************************
  VARIABLES GLOBALES
*************************************/
char dato = 0;
int grados[3] = {0, 0, 0};
int posiciones[limPos][3];
int gradosDecorador[3];
byte contador = 0;

boolean LecturaBotonGuardar; byte BotonGuardar = A2;
boolean LecturaBotonGuardarEEPROM; byte BotonGuardarEEPROM = A3;

// ****************************
//                            *
//          Joystick          *
//                            *
// ****************************
short Joystick1 [3] = {A1,A0, A7};

// ****************************
//                            *
//          EEPROM            *
//                            *
// ****************************
struct ObjetoPosiciones{
  int contadorDeDatos = 0;
  int MatrizPosiciones [limPos] [3];
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
void movimientoJoyStick(short joy, BasicStepperDriver Motor, byte pos, byte limCentral = 112, byte relacion = 1);
void setup() {

  //Declaracion de Sensores y pulsadores
  pinMode(STOPPER_PIN_Y,INPUT_PULLUP);
  pinMode(STOPPER_PIN_Z,INPUT_PULLUP);
  pinMode(STOPPER_PIN_X,INPUT_PULLUP);

  pinMode(BotonGuardar, INPUT_PULLUP);
  pinMode(BotonGuardarEEPROM, INPUT_PULLUP);
  
  pinMode(8,OUTPUT);
  
  Serial.begin(9600);
  
  stepperX.begin(MOTOR_X_RPM, MICROSTEPS);
  stepperY.begin(MOTOR_Y_RPM, MICROSTEPS);
  stepperZ.begin(MOTOR_Z_RPM, MICROSTEPS);

  ActivarMotores(false);
  

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
      ActivarMotores(1);
      controller.move(convertirGrados(0,2), convertirGrados(90), convertirGrados(0));
      cinematicaInv(90, 0);
      
      delay(5000);
      
      controller.move(convertirGrados(0,2), 
      convertirGrados(outCinematicoB), 
      convertirGrados(outCinematicoC));
      ActivarMotores(0);
      delay(200);
      
      dato = 0;
      break;
  

  }

  if(dato == 'j'){
    
      
      ActivarMotores(1);

      short JoyIzqX = analogRead(Joystick1[0]);
      short JoyIzqY = analogRead(Joystick1[1]);
      short JoyIzqZ = analogRead(Joystick1[2]);

      
      movimientoJoyStick(JoyIzqX, stepperX, 0, 100, 2);
      movimientoJoyStick(JoyIzqY, stepperY, 1, 100, 1);
      movimientoJoyStick(JoyIzqZ, stepperZ, 2, 100, 1);
      
      
      for(byte i = 0; i < 3; i++){
        Serial.print(grados[i]); Serial.print("   ");}
      

      Serial.println();
      
      
      LecturaBotones();
      if(LecturaBotonGuardar == 0){
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
      }

      if(!LecturaBotonGuardarEEPROM){
        delay(300);
        GuardarEnEEPROM();
        dato=0;
      }
      
      

  }
}

void detenerMotores(){
  stepperX.stop();
  stepperY.stop();
  stepperZ.stop();
}

void rutinaGeneral(){
  
  int xAnterior =0, yAnterior =0, zAnterior = 0;

  for(byte i = 0; i < limPos; i++){
    int x = posiciones[i][0];
    int y = posiciones[i][1];
    int z = posiciones[i][2];
    
//    stepperX.move(convertirGrados(x-xAnterior));  
//    stepperY.move(convertirGrados(y-yAnterior));  
//    stepperZ.move(convertirGrados(z-zAnterior));  
    
    
    controller.move(convertirGrados(x-xAnterior, 2), 
                    convertirGrados(y-yAnterior),
                    convertirGrados(z-zAnterior));
    
    xAnterior = x; yAnterior = y; zAnterior = z;

    
   

    if(!xAnterior && !yAnterior && !zAnterior)
      delay(0);
    else
      //Serial.println("entro");
      //Motores sincronizados
      //cinematicaInv(y, z);
      //controller.move(convertirGrados(x-xAnterior, 2), 
      //              convertirGrados(y-outCinematicoB),
      //             convertirGrados(z-outCinematicoC));
      //controller.move(convertirGrados(0,2), convertirGrados(outCinematicoB), convertirGrados(outCinematicoC));
      delay(1000);
  }
  delay(200);
  
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

void LecturaBotones(){
 
  LecturaBotonGuardar = digitalRead(BotonGuardar);
  LecturaBotonGuardarEEPROM = digitalRead(BotonGuardarEEPROM);
  
}


//movimientoJoyStick(valor A Pasar, Motor_para_accionar, vector donde se Guarda, Valor o rango de sensibilidad)
void movimientoJoyStick(short joy, BasicStepperDriver Motor, byte pos, byte limCentral = 112, byte relacion = 1){
    
    //Valores del joystick para evitar que se mueva en el centro o rango de NO movimiento cuando se suelta
    //El valor no entra linealizado es decir va de 0 a 1023
    if (joy < (512-limCentral) || joy > (512+limCentral)){

      //linealizo de -10 a 10 para la suavidad
      joy = map(joy,0,1023, -1, 1);
      //guardar en variable global
      grados[pos] += joy;

//      #if debug
//        Serial.print(joy); Serial.print("\t\t");  Serial.print(grados[pos]);
//      #endif
      Motor.move(convertirGrados(joy, relacion));
    }
    
    else{
      Motor.stop();
    }
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
  stepperZ.startMove(100 * MOTOR_STEPS * MICROSTEPS);
  do{
    flag = goToHome_Z();
    if(flag){
      //Moverse ciertos grados hacia adelante
      stepperZ.move(convertirGrados(-35));
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
    grados[i]=0;
  }
}
