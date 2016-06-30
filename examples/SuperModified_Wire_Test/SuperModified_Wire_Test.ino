#include <ZerooneSupermodified.h>
#include <Wire.h>

#define id 4

ZerooneSupermodified motor(ZO_HW_WIRE);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  int c;
  
  c = Serial.read();
  
  if( c == '1' )
  {
    Serial.write('1');
    motor.start(id);
  }
  
  if( c == '2' )
  {
    Serial.write('2');
    motor.stop(id);
  }
  
  if( c == '3' )
  {
    Serial.write('3');
    motor.moveWithVelocity(id,5000);
  }

  if( c == '4' )
  {
    Serial.write('4');
    motor.moveWithVelocity(id,-5000);
  }

  if( c == '5' )
  {
    Serial.write('5');
    motor.resetErrors(id);
  }  
  
  if( c == '6' )
  {
    Serial.write('6');
    motor.broadcastStart();
  } 
  
  if( c == '7' )
  {
    Serial.write('7');
    motor.broadcastStop();
  }
  
    if( c == '8' )
  {
    Serial.println();
    Serial.print("Velocity: ");
    Serial.print(motor.getVelocity(id));
  }

  if( c == '9' )
  {
    Serial.println();
    Serial.print("Position: ");
    Serial.print((double)motor.getPosition(id));
  }

  if( !motor.getCommunicationSuccess() )
  {
    Serial.println();
    Serial.print("Communication Warning :");
    Serial.print(motor.getWarning());
  } 
  
}
