/*
 *  Program to control an ADNS-3080 optical flow sensor using an Arduino Uno.
 *  
 *  Created: 11/04/2019
 *  Author:  Wilson Woods
 */


#include "SPI.h"

//uno pin designations

#define FLOW_SENSOR_RESET                             9
#define FLOW_SENSOR_SS                               10
#define FLOW_SENSOR_MOSI                             11
#define FLOW_SENSOR_MISO                             12
#define FLOW_SENSOR_CLOCK                            13 


#define ADNS3080_PIXELS_X                            30
#define ADNS3080_PIXELS_Y                            30

#define ADNS3080_PRODUCT_ID                        0x00
#define ADNS3080_REVISION_ID                       0x01
//0x02, 0x03, 0x04 should be read sequentially
#define FLOW_SENSOR_MOTION                         0x02   // motion since last read, 0 = no motion,  1 = data available in dy/dx
#define DELTA_X                                    0x03   // motion in x and y directions, +/- 128 R/L or U/D
#define DELTA_Y                                    0x04   
#define SQUAL                                      0x05   // (surface quality) 1/4 of the # of valid surface features in current frame
                                                          // # of features in frame = SQUAL reg val * 4
#define ADNS3080_PIXEL_SUM                         0x06   // avg pixel val (avg pixel brightness)
                                                          // avg pixel = reg val * 256 / 900 = reg val / 3.51
#define ADNS3080_MAXIMUM_PIXEL                     0x07   // max pixel val in current frame (max pixel brightness)
                                                          // 6 bit, 0 - 63
#define CONFIG_BITS                                0x0a   // bit 7 must be 0, bit 6 (LED shutter mode) 0 = shutter off/LED always on
                                                          // 1 = shutter on/LED on as needed, bit 5 = sys test
                                                          // bit 4 = res, 0 = 400, 1 = 1600
#define EXTENDED_CONFIG                            0x0b   // disable pull-up current sources, set AGC, auto/fixed frame rate
#define ADNS3080_DATA_OUT_LOWER                    0x0c   // output from sys self test/SROM CRC test
#define ADNS3080_DATA_OUT_UPPER                    0x0d   // read upper first, lower second
#define ADNS3080_SHUTTER_LOWER                     0x0e   // auto adjusts shutter value to keep avg/max pixel vals in normal range
#define ADNS3080_SHUTTER_UPPER                     0x0f
#define FRAME_PERIOD_LOWER                         0x10   // read upper first, lower second, determine frame period and calculate frame rate
#define FRAME_PERIOD_UPPER                         0x11   // frame rate = clock frequency / register value
#define MOTION_CLEAR                               0x12   // reset motion counters (dx, dy) by writing anything here
#define ADNS3080_FRAME_CAPTURE                     0x13   // write here to store next 1.67 frames (1536 pixels) to SROM RAM
                                                          // read 1536 times in a row for same data as pixel_burst
#define ADNS3080_SROM_ENABLE                       0x14   // start SROM download or SROM CRC test
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER      0x19   // MAX frame PERIOD = MIN frame RATE and vice versa
                                                          // frame rate = clock freq / reg val
                                                          // READ upper first, lower second, WRITE lower first, upper second
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER      0x1a   
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER      0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER      0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER           0x1e   // max/min shutter val in auto mode
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER           0x1e
#define ADNS3080_INVERSE_PRODUCT_ID                0x3f   // to test SPI port
#define ADNS3080_PIXEL_BURST                       0x40   // high speed access to pixel data from last 1.67 frames
#define ADNS3080_MOTION_BURST                      0x50   // high speed access to Motion, dx, dy, SQUAL, shutter upper/lower, and max pixel regs
#define ADNS3080_SROM_LOAD                         0x60   // high speed program adns3080 from external SROM or microcontroller
#define ADNS3080_PRODUCT_ID_VAL                    0x00


struct SensorData
{
  int detect;
  byte dX;
  byte dY;
};

void resetFlowSensor()
{
  digitalWrite( FLOW_SENSOR_RESET, HIGH );
  delayMicroseconds( 50 );                                // reset pulse >10us
  digitalWrite( FLOW_SENSOR_RESET,  LOW );
  delay( 35 );                                            // 35ms from reset to functional
}

//initalize sensor
int initFlowSensor()
{
  pinMode( FLOW_SENSOR_RESET, OUTPUT );
  pinMode( FLOW_SENSOR_SS,    OUTPUT );
  
  digitalWrite( FLOW_SENSOR_SS, HIGH );
  
  resetFlowSensor();
  
  int productID = readFlowSensor( ADNS3080_PRODUCT_ID );
  if( productID != ADNS3080_PRODUCT_ID_VAL )
    return -1;

  // turn on sensitive mode
  writeFlowSensor( CONFIG_BITS,     0x09 );
  writeFlowSensor( EXTENDED_CONFIG, 0x03 );

  return 0;
}

void writeFlowSensor( int reg, int val )
{
  digitalWrite( FLOW_SENSOR_SS,  LOW );
  SPI.transfer( reg | 0x80 );
  SPI.transfer( val );
  digitalWrite( FLOW_SENSOR_SS, HIGH );
  delayMicroseconds( 50 );
}

int readFlowSensor( int reg )
{
  digitalWrite( FLOW_SENSOR_SS,  LOW );
  SPI.transfer( reg );
  delayMicroseconds( 75 );
  int regVal = SPI.transfer( 0xff );                      // write while reading
  digitalWrite( FLOW_SENSOR_SS, HIGH ); 
  delayMicroseconds( 1 );
  return regVal;
}




//use FRAME_PERIOD_UPPER or FRAME_PERIOD_LOWER
int getFrameRate( int reg )
{
  int period = readFlowSensor( reg );
  
}


int read_dxdy( int reg )
{
  digitalWrite( FLOW_SENSOR_SS,  LOW );
  SPI.transfer( reg );
  delayMicroseconds( 75 );
  int regVal = SPI.transfer( 0xff );
  digitalWrite( FLOW_SENSOR_SS, HIGH ); 
  delayMicroseconds( 1 );
  return regVal;
}

int check_squal()
{
  digitalWrite( FLOW_SENSOR_SS,  LOW );
  SPI.transfer( SQUAL );
  delayMicroseconds( 75 );
  int squalVal = SPI.transfer( 0xff );
  digitalWrite( FLOW_SENSOR_SS, HIGH );
  delayMicroseconds( 1 );
  return squalVal;
}

int detectMotion()
{
  digitalWrite( FLOW_SENSOR_SS, LOW );
  SPI.transfer( FLOW_SENSOR_MOTION );
  delayMicroseconds( 75 );
  int regVal = SPI.transfer( 0xff );                      // write while reading
  digitalWrite( FLOW_SENSOR_SS, HIGH ); 
  delayMicroseconds( 1 );
  return regVal;
}



void setup() 
{
  pinMode( FLOW_SENSOR_SS,    OUTPUT );                   // pin 10
  pinMode( FLOW_SENSOR_MISO,   INPUT );                   // pin 12
  pinMode( FLOW_SENSOR_MOSI,  OUTPUT );                   // pin 11
  pinMode( FLOW_SENSOR_CLOCK, OUTPUT );                   // pin 13
  
  SPI.begin();
  SPI.setClockDivider( SPI_CLOCK_DIV32 );
  SPI.setDataMode( SPI_MODE3 );
  SPI.setBitOrder( MSBFIRST );
  
  
  
  Serial.begin( 38400 );

  if( initFlowSensor()==-1 )
  {
    Serial.println( "Optical flow sensor failed to initialize" );
    while( 1 );
  }  

  byte extconfig = readFlowSensor( EXTENDED_CONFIG );
  Serial.print( "extended config reg: " ); Serial.println( extconfig );
  byte configbits = readFlowSensor( CONFIG_BITS );
  Serial.print( "config_bits reg: " ); Serial.println( configbits );
}


void loop() 
{
  Serial.print( "Surface quality: " ); Serial.println( check_squal() );
  if( detectMotion() != 0 )
  {
    Serial.print( "DX: " ); Serial.print( read_dxdy( DELTA_X ) ); Serial.print(" ");
    Serial.print( "DY: " ); Serial.println( read_dxdy( DELTA_Y ) );
  }
  else
  {
    Serial.println( "No motion detected" );
  }
  //writeFlowSensor(MOTION_CLEAR, 0xFF);
  delayMicroseconds( 200 );
}
