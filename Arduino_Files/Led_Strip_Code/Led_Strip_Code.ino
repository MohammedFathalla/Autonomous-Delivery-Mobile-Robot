/* 
 *  Code for controlling Led strips in the robot
 *  
 */

#define led 3  // PWM led pin

void setup()
{
 pinMode(led,OUTPUT);  // Set led PWM pin as output
}

void loop() 
{

// Two "for loops" where lighting from 0 to max light then reversing by going from max to zero light again

for ( int i=0 ; i<255 ; i++) 
{
  analogWrite(led,i);
  delay(10);
}

for ( int i=255 ; i>0 ; i--) 
{
  analogWrite(led,i);
  delay(10);
}

}
