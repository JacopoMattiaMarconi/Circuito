//  ***     ***** ****  ***** *     *   *   *
//  *   *   *   * *   * *   * *     *  * *  *
//  *   *   *   * *   * *   * *     * *   * *
//  * *     *   * * **  *   * *     * ***** *
//  *  *    *   * *   * *   *  *   *  *   * *
//  *   *   *   * *   * *   *   * *   *   * *
//  *    *  ***** ****  *****    *    *   * *****


/*Nella gara del 26 e 27 maggio
 * Primo giorno   2°
 * Secondo giorno 2° <--Definitivo
 */



//Dati principali


//Da modificare
long long tempo_s=30;//I secondi che ci deve mettere
int ostacoli=18;

//Altro
#define MAXSPEED 90  //di default 100 max speed of the robot 95
#define MINSPEED 20  //30
int BASESPEED=70;    //80
float corr=10;       //10
float rapporto= 9/10;
int cntlet=0;
int LETCORR= 500;
long marginei= 1000;//Margine di errore accettato iniziale
long marginef= 200; //Margine di errore accettato finale
int motorSpeed;
long long TEMPO = tempo_s*1000;
//Inclusione
#include <QTRSensors.h>

//Definizione motori
#define out_STBY    7
#define out_B_PWM   10 //B motore destro
#define out_A_PWM   5  //A motore sinistro
#define out_A_IN2   6
#define out_A_IN1   4
#define out_B_IN1   8
#define out_B_IN2   9
#define left_motor  0
#define right_motor 1

#define leftFar          0
#define leftCenter       1
#define leftNear         2
#define rightNear        3
#define rightCenter      4
#define rightFar         5
#define NUM_SENSORS      6
#define TIMEOUT       2500  
#define EMITTER_PIN   QTR_NO_EMITTER_PIN

#define DEBFULL 0

//Definizione della soglia di errore
#define SOGLIA 180

//Definizioni per margine in cui accellerare o rallentare
#define PERINCR 350 //350
#define PERDECR 750

#define SOG_ST  10
#define ST_N    0
#define ST_CD   1
#define ST_CS   2
#define ST_T    3
#define ST_V    4


int leftCenterReading;
int leftNearReading;
int leftFarReading;
int rightCenterReading;
int rightNearReading;
int rightFarReading;
int lastDebugTime = 0;
int lastError = 0;

int das=0;        //deve girare a sinistra

int time=0;
int rightMotorSpeed;
int leftMotorSpeed;
int s1;
int s2;

QTRSensorsRC qtrrc((unsigned char[]) {
  A0, A1, A2, A3, A4, A5}
,NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

void setup(){
  pinMode(out_STBY,OUTPUT);
  pinMode(out_A_PWM,OUTPUT);
  pinMode(out_A_IN1,OUTPUT);
  pinMode(out_A_IN2,OUTPUT);
  pinMode(out_B_PWM,OUTPUT);
  pinMode(out_B_IN1,OUTPUT);
  pinMode(out_B_IN2,OUTPUT);
  for (int i = 0; i < 400; i++){
    qtrrc.calibrate();
  }
  Serial.begin(9600);
  #ifdef Sperimentale
    //Serial.print("finito");
  #endif
  motor_standby(false);
}
// a spanne fa 300 rilevazioni al secondo
// a spanne fa 10 cm al sec

///////////////////////////////////
// valori modificabili al volo
// per modificare al volo
// sulla console si spedisce un carattere, il valore e *
// con S si ottiene lo stato corrente
// con R si resertta ad un valore iniziale
// i valore float si spediscono moltiplicati per 1000


// numero ultime misure per valutare la sterzata
//#define NMIS 15
int NMIS=15;  //
int PRCT =80;
float KP=0.05;
float KD=2.0;

char uc;  // ultimo carattere letto
int upl;  // ultimo numero letto
int pl;

  boolean sen[8];
  int v,vp;
  #define soglia 180



//ultime letture del sensore in binario convertite in dec
int um[100];
// indice
int im=0;


// 3 bit a dx
#define MSKDX 7
// 3 bit a sx
#define MSKSX 56


int cl=-2;
long  ts,tc;



void loop(){


  unsigned int sensors[6];

  int aposition = qtrrc.readLine(sensors); // legge i 6 sensori(sensors) e restituisce un unico valore da 0 a 5000
  //Serial.println(aposition);
  int error = aposition - 2500;


  vp=v;
  v=0;
  for(int i= 0; i<6; i++){ //leggere la linea
    sen[i]= sensors[i]>soglia;
    v= v*2+sen[i];
  }
  //Serial.println(v);
  if(v!=vp && v==63){
    cl++;

    
    if (cl==0)
      ts=millis();
    
    if (cl >0){
      tc=millis();
      //diff=(tc-ts)-(cl*(TEMPO/ostacoli)); //Regolazione intelligente Correzione


      if (tc-ts < (cl*(TEMPO-marginei)/ostacoli) && BASESPEED>MINSPEED){ // rallenta
        BASESPEED-=corr;
        if (corr==0)
          corr=1;
      }

      //Serial.println(tc-ts);
      if (tc-ts > (cl*(TEMPO+marginei)/ostacoli) && BASESPEED<MAXSPEED){ // accellera
        BASESPEED+=corr; //+0
        if (corr==0)
          corr=1;
      }
      //Ricalcolo margine & correzione
      marginei=marginei-((marginei-marginef)/ostacoli);
      corr=corr*rapporto;
    } 
/*
    Serial.println("Linea rilevata");
    Serial.print("BASESPEED= ");
    Serial.println(BASESPEED);
    Serial.print("Tempo impiegato= ");
    Serial.println(tc-ts);
    Serial.print("Correzione= ");
    Serial.println(corr);
    Serial.print("Margine= ");
    Serial.println(marginei);
    Serial.println("");
    Serial.println("");
    Serial.println("");*/
  }else{
  
    motorSpeed = (int)(KP * error + KD * (error - lastError)); //Questa è il cuore del programma
    lastError = error;

  }

  rightMotorSpeed = BASESPEED - motorSpeed;
  leftMotorSpeed = BASESPEED + motorSpeed;

  if (rightMotorSpeed > MAXSPEED )
    rightMotorSpeed = MAXSPEED;
  if (leftMotorSpeed > MAXSPEED )
    leftMotorSpeed = MAXSPEED;
  if (rightMotorSpeed < -MAXSPEED)
    rightMotorSpeed = -MAXSPEED;
  if (leftMotorSpeed < -MAXSPEED)
    leftMotorSpeed = -MAXSPEED;

  motors(leftMotorSpeed,rightMotorSpeed);
  //Serial.print(error);//Serial.print(" ");//Serial.println(motorSpeed);

}

void motors(int l, int r){
  set_motor(right_motor,r);
  set_motor(left_motor,l);
}

void set_motor(boolean motor, char speed) { // imposta la velocitÃ  tra -100 (indietro) e +100 (avanti)
  byte PWMvalue=0;
  PWMvalue = map(abs(speed),0,100,50,255);
  if (speed > 0)
    motor_speed(motor,0,PWMvalue);
  else if (speed < 0)
    motor_speed(motor,1,PWMvalue);
  else {
    motor_coast(motor);
  }
}

void motor_speed(boolean motor, boolean direction, byte speed) { // imposta la velocitÃ  tra 0 e 255
  if (motor == left_motor) {
    if (direction == 0) {
      digitalWrite(out_A_IN1,HIGH);
      digitalWrite(out_A_IN2,LOW);
    }
    else {
      digitalWrite(out_A_IN1,LOW);
      digitalWrite(out_A_IN2,HIGH);
    }
    analogWrite(out_A_PWM,speed);
  }
  else {
    if (direction == 0) {
      digitalWrite(out_B_IN1,HIGH);
      digitalWrite(out_B_IN2,LOW);
    }
    else {
      digitalWrite(out_B_IN1,LOW);
      digitalWrite(out_B_IN2,HIGH);
    }
    analogWrite(out_B_PWM,speed);
  }
}

void motor_standby(boolean state) { // abilita/disabilita i motori
  if (state == true)
    digitalWrite(out_STBY,LOW);
  else
    digitalWrite(out_STBY,HIGH);
}

void motor_coast(boolean motor) { // motore in folle
  if (motor == left_motor) {
    digitalWrite(out_A_IN1,LOW);
    digitalWrite(out_A_IN2,LOW);
    digitalWrite(out_A_PWM,HIGH);
  }
  else {
    digitalWrite(out_B_IN1,LOW);
    digitalWrite(out_B_IN2,LOW);
    digitalWrite(out_B_PWM,HIGH);
  }
}

void motor_brake(boolean motor) { // freno motore
  if (motor == left_motor) {
    digitalWrite(out_A_IN1,HIGH);
    digitalWrite(out_A_IN2,HIGH);
  }
  else {
    digitalWrite(out_B_IN1,HIGH);
    digitalWrite(out_B_IN2,HIGH);
  }
}

void dx(){
    //maxSpeed=30;
    motors(-20,-20);
    delay(80);
    leftMotorSpeed = MAXSPEED*80/100;
    rightMotorSpeed = -MAXSPEED*40/100;
    motors(leftMotorSpeed,rightMotorSpeed);
    delay(400);
}

void sx(){
    //maxSpeed=30;
    motors(-20,-20);
    delay(80);
    rightMotorSpeed = MAXSPEED*80/100;
    leftMotorSpeed = -MAXSPEED*40/100;
    motors(leftMotorSpeed,rightMotorSpeed);
    delay(400);
}
