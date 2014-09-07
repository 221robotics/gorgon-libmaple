// Written by Nick Gammon
// February 2011


#include <SPI.h>

bool is_on = true;

void setup (void)
{

  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);  // ensure SS stays high for now

  // Put SCK, MOSI, SS pins into output mode
  // also put SCK, MOSI into LOW state, and SS into HIGH state.
  // Then put SPI hardware into Master mode and turn SPI on
  SPI.begin();

  // Slow down the master a bit
  //SPI.setClockDivider(SPI_CLOCK_DIV16);
  
  // this worked with the maple mini
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  
  //Serial.begin(9600);
  
}  // end of setup


void loop (void)
{
  // enable Slave Select
  digitalWrite(9, LOW);
  
  // set PWM
  SPI.transfer(0x04);
  
  // PWM channel 0
  SPI.transfer(0);

  // 90 degrees
  SPI.transfer(127);

  // disable Slave Select
  digitalWrite(9, HIGH);

  delay (1000);  // 1 seconds delay 
}  // end of loop
