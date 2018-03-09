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
 * Description: This Arduino sketch serves as DAQ 
 * system to be used for experiments using the
 * force-feedback proportional controller
 *---------------------------------------------------
 *---------------------------------------------------
*/
//---------------------------------------------------
//DEFINES
#define PKG_ST '$'
#define PKG_ET '!'
//---------------------------------------------------
//VARIABLES
uint16_t adSample = 0; //adc value
uint8_t adMSB = 0; //adc msb
uint8_t adLSB = 0; //adc lsb
uint8_t packet[4] = {0,0,0,0}; //serial packet
//---------------------------------------------------
void setup() {  
  Serial.begin(9600); //initializes serial
}

void loop() {  
  adSample = analogRead(A0); //reads the ADC channel
  adMSB = adSample>>8; //retrieves msb
  adLSB = adSample&0xFF; //retrieves lsb
  //mounts the serial packet
  packet[0] = PKG_ST; //header
  packet[1] = adMSB; //data msb
  packet[2] = adLSB; //data lsb
  packet[3] = PKG_ET; //end of packet
  Serial.write(packet,4); //sends the packet via serial
  delay(10); //sampling period: 10ms = 100 Hz
}
