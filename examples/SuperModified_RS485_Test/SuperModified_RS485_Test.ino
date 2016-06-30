#include <ZerooneSupermodified.h>
#include <Wire.h>

ZerooneSupermodified motor(ZO_HW_SERIAL,13);

void setup()
{
  Serial.begin(57600);
}

void loop()
{
  int c;
  
  if( Serial.available())
  {
    c = Serial.read();
    
    if( c == '1' )
      motor.start(4);
    
    if( c == '2' )
      motor.stop(4);
    
    if( c == '3' )
      motor.moveWithVelocity(4,5000);
  
    if( c == '4' )
      motor.moveWithVelocity(4,-5000);
  
  
    if( c == '5' )
      motor.resetErrors(4); 
    
    if( c == '6' )
      motor.broadcastStart();
    
    if( c == '7' )
      motor.broadcastStop();
    
  
    if( !motor.getCommunicationSuccess() )
    {
      Serial.println();
      Serial.print("Communication Warning :");
      Serial.print(motor.getWarning());
    } 
    else
    {
      Serial.println();
      Serial.print("Communication Success.");
    }
  }
  
}
