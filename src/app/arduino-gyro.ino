#include <stdio.h> 

void setup()
{
  int pwm = 2;
  Serial.begin(9600);  

  pinMode(pwm,OUTPUT); 
  analogWrite(pwm, 1);
}

void loop()
{
  int SCL = analogRead(A0);
  int SDA = analogRead(A1);
  int EDA = analogRead(A2);
  int ECL = analogRead(A3);
  int ADO = analogRead(A4);
  int INT = analogRead(A5);
  int NCS = analogRead(A6);
  int FSYNC = analogRead(A7); 

  char SCL_FORMAT[15];
  char SDA_FORMAT[15];
  char EDA_FORMAT[15];
  char ECL_FORMAT[15];
  char ADO_FORMAT[15];
  char INT_FORMAT[15];
  char NCS_FORMAT[15];
  char FSYNC_FORMAT[15];

  sprintf(SCL_FORMAT, "SCL:\t%04d\t", SCL);
  sprintf(SDA_FORMAT, "SDA:\t%04d\t", SDA);
  sprintf(EDA_FORMAT, "EDA:\t%04d\t", EDA);
  sprintf(ECL_FORMAT, "ECL:\t%04d\t", ECL);
  sprintf(ADO_FORMAT, "ADO:\t%04d\t", ADO);
  sprintf(INT_FORMAT, "INT:\t%04d\t", INT);
  sprintf(NCS_FORMAT, "NCS:\t%04d\t", NCS);
  sprintf(FSYNC_FORMAT, "FSYNC:\t%04d", FSYNC);

  Serial.print(SCL_FORMAT);
  Serial.print(SDA_FORMAT);
  Serial.print(EDA_FORMAT);
  Serial.print(ECL_FORMAT);
  Serial.print(ADO_FORMAT);
  Serial.print(INT_FORMAT);
  Serial.print(NCS_FORMAT);
  Serial.println(FSYNC_FORMAT);
   
  delay(500);
}
