#include <ZerooneSupermodified.h>
#include <Wire.h>

ZerooneSupermodified motor(ZO_HW_SERIAL);

void setup()
{
  Serial.begin(57600);
}

void loop()
{
  int c;
  
  c = Serial.read();
  
  if( c == '1' )
  {
    Serial.write('1');
    motor.start(4);
  }
  
  if( c == '2' )
  {
    Serial.write('2');
    motor.stop(4);
  }
  
  if( c == '3' )
  {
    Serial.write('3');
    motor.moveWithVelocity(4,5000);
  }

  if( c == '4' )
  {
    Serial.write('4');
    motor.moveWithVelocity(4,-5000);
  }

  if( c == '5' )
  {
    Serial.write('5');
    motor.resetErrors(4);
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
    Serial.print(motor.getVelocity(4));
  }

  if( c == '9' )
  {
    Serial.println();
    Serial.print("Position: ");
    Serial.print((double)motor.getPosition(4));
  }

  if( !motor.getCommunicationSuccess() )
  {
    Serial.println();
    Serial.print("Communication Warning :");
    Serial.print(motor.getWarning());
  } 
  
}
