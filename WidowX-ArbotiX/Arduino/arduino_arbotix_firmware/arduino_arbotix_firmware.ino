/*---------------------------------------------------
 * NATIONAL UNIVERSITY OF SINGAPORE - NUS
 * SINGAPORE INSTITUTE FOR NEUROTECHNOLOGY - SINAPSE
 * Singapore
 *---------------------------------------------------
 * Author: Andrei Nakagawa-Silva
 * Contact: nakagawa.andrei@gmail.com
 * URL: www.sinapseinstitute.org
 * Last update: 08/03/2018
 *---------------------------------------------------
 * IMPORTANT:
 * This program should be uploaded to the WidowX robot
 * using Arduino IDE version 1.0.6. The ArbotiX libraries
 * should be proper installed in the Arduino folder. Also,
 * in the Arduino IDE it is necessary to select ArbotiX
 * in Tools -> Board.
 * According to the documentation, seen in March, 2018,
 * ArbotiX is not working for newer versions of the 
 * Arduino IDE.
 * More details in:
 *  learn.trossenrobotics.com/arbotix/arbotix-getting-started/
 *---------------------------------------------------
 * Description: This arduino sketch should be uploaded
 * to a WidowX robot (ArbotiX platform) so the robot
 * can be controlled from any software through serial
 * communication interface
 * Therefore, this sketch serves like a wrapper that
 * that allows other softwares to be implemented
 * at a higher level, using the low-level functions
 * contained in the libraries for Arduino. 
 * Any device can send commands or receive data to/from
 * ArbotiX via serial communication according to the
 * protocol established here.
 *---------------------------------------------------
 * Details of the serial commnication protocol
 * Input packages format:
 * [header][which action][which servo][positionMSB][positionLSB][end]
 * Output packages format:
 * [header][which servo][positionMSB][positionLSB][end]
 *---------------------------------------------------
 * TO-DO LIST:
 * - Should implement a handshake routine to avoid
 * communication errors and possible delay in the first
 * commands that are sent to ArbotiX
 *---------------------------------------------------
*/
//---------------------------------------------------
//LIBRARIES
//import ax12 library to send DYNAMIXEL commands
#include <ax12.h>
//---------------------------------------------------
//DEFINES
#define baud 9600 //baudrate
#define AX_LED 0 //LED pin id: LED can be used to check something
#define AX_SETPOSITION 0 //Action = Set servo position
#define AX_GETPOSITION 1 //Action = Get servo position
#define AX_TORQUE_ON 2 //Action = Activate torque
#define AX_RELAX 3 //Action = Deactivate torque
#define AX_LED_ON 4 //Action = Turns the LED on
#define AX_LED_OFF 5 //Action = Turns the LED off
#define inPackageSize 6   //size (in bytes) of the input package
#define outPackageSize 5  //size (in bytes) of the output package
#define ST '$' //Header
#define ET '!' //End of package
#define PKG_INACTION 1 //Position in the package that corresponds to action
#define PKG_INSERVO 2 //Position in the package that corresponds to action
#define PKG_INMSB 3 //Position in the incoming package that corresponds to the MSB byte
#define PKG_INLSB 4 //Position in the incoming package that corresponds to the LSB byte
//---------------------------------------------------
//GLOBAL VARIABLES
uint8_t* inPackage;
uint8_t action = 0;
uint8_t servoId = 0;
uint8_t posMSB = 0;
uint8_t posLSB = 0;
uint16_t fullPos = 0;
int pos = 0;
//---------------------------------------------------
//---------------------------------------------------
void setup()
{
  //Initializes the serial communication
  Serial.begin(baud);  
  //LED pin as output
  pinMode(AX_LED, OUTPUT);
  //Relax all servos
  for(int k=0; k<7; k++)
  {
     Relax(k); 
  }
}
//---------------------------------------------------
void loop()
{
  //Routinely checks if a new package has arrived
  //when it does, proceed by deciding which action
  //should be taken
  inPackage = receivePackage();
  if(inPackage != NULL)
  {
    //for debugging purposes    
    //Serial.println("PACKAGE OK!");
    //Serial.println(String(inPackage[0]) + " " + String(inPackage[1]) + " " + String(inPackage[2]) + " " + String(inPackage[3]) + " " + String(inPackage[4]) + " " + String(inPackage[5]));
    
    //Depending on the desired action
    switch(inPackage[PKG_INACTION])
    {
      //Set position of a given servomotor
      case AX_SETPOSITION:        
        fullPos = inPackage[PKG_INMSB];
        fullPos = fullPos<<8;      
        fullPos = fullPos + (inPackage[PKG_INLSB]);
        //Serial.println(String(inPackage[2]) + " " + String(fullPos));
        SetPosition(inPackage[PKG_INSERVO],fullPos);
        break;
     
     //Returns the position (0 - 1023) of the chosen servo
     case AX_GETPOSITION:
       //gets the servo id
       servoId = inPackage[PKG_INSERVO];
       //gets the servo position
       pos = GetPosition(servoId);
       //MSB of the postion
       posMSB = (pos>>8);
       //LSB of the position
       posLSB = pos&0xFF;
       //Writes an output package containing
       //the servo id and its position divided
       //into MSB and LSB bytes
       Serial.write(ST); //header
       Serial.write(servoId); //servo id
       Serial.write(posMSB);//MSB position
       Serial.write(posLSB); //LSB position
       Serial.write(ET); //End of package
       break;
     
     //Turn the LED on
     case AX_LED_ON:
       digitalWrite(AX_LED,HIGH); //high level at Pin 0
       break;
       
     //Turn the LED off
     case AX_LED_OFF:
       digitalWrite(AX_LED,LOW); //low level at Pin 0
       break;
    }
  }
  else
  {
    //for debugging purposes or exceptions
    //Serial.println("Package error!");
  }
}
//---------------------------------------------------
uint8_t* receivePackage()
{
  uint8_t package[6] = {0,0,0,0,0,0};
  uint8_t incByte = 0;
  if(Serial.available())
  {
    //reads one byte from the serial buffer
    incByte = Serial.read();
    //if the byte corresponds to the header
    if(incByte == ST)
    {
      package[0] = ST;
      
      //for debugging
      //Serial.println("READ THE HEADER!");
      
      //waits until a complete package in the serial buffer is available
      while(Serial.available() < 5)
      {
        //for debugging purposes
        //Serial.println("waiting...");
      }      
      
      //once the amount of bytes are available, read all of them
      //byte by byte and store them in the package buffer
      for(int i=1; i<inPackageSize; i++)
      {
        package[i] = Serial.read();//Serial.read();
      }
    }
  }
  //Checks if the last byte corresponds to the end of the buffer
  //if it is, then the package can be validated
  //otherwise, it returns a NULL pointer
  if(package[5] == ET)
  {
    //for debugging purposes
    //Serial.println("PACKAGE OK!");  
    //Serial.println(String(package[0]) + " " + String(package[1]) + " " + String(package[2]) + " " + String(package[3]) + " " + String(package[4]) + " " + String(package[5]));
    return package;
  }
  else
    return NULL;
}
