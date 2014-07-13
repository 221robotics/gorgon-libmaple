// Written by Nick Gammon
// February 2011


#include <SPI.h>

bool is_on = true;

void setup (void)
{

  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);  // ensure SS stays high for now

  // Put SCK, MOSI, SS pins into output mode
  // also put SCK, MOSI into LOW state, and SS into HIGH state.
  // Then put SPI hardware into Master mode and turn SPI on
  SPI.begin();

  // Slow down the master a bit
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  
  Serial.begin(9600);
  
}  // end of setup


void loop (void)
{
  // enable Slave Select
  digitalWrite(2, LOW);
  
  byte recv;
  
  if (is_on) {
    recv = SPI.transfer('0');
    is_on = false;
  } else {
    recv = SPI.transfer('1');
    is_on = true;
  }
  
  Serial.println(recv);

  // disable Slave Select
  digitalWrite(2, HIGH);

  delay (1000);  // 1 seconds delay 
}  // end of loop
