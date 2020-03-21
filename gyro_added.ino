#include <ft_config.h>
#include <ft.h>
#include <Pozyx_definitions.h>
#include <Pozyx.h>
#include <Wire.h>

//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------

// Distance between anchors
uint32_t const ANCHOR_DISPLACEMENT = 1600;

// POZYX IDs
uint16_t const LEFT_ANCHOR_ID = 0x6719;
uint16_t const RIGHT_ANCHOR_ID = 0x6E21;
uint16_t const DEVICE_ID = 0x6741;
uint16_t const REMOTE_ID = 0x6751;

// Ring Buffer Settings
uint32_t const MAX_BUFFER_SIZE = 30;

// conversions
double const RADIANS_TO_DEGREES = 57.2957795131;

//timing:
unsigned long startMillis; 
unsigned long currentMillis;
unsigned long lastFTMillis;
const unsigned long period = 500; //one second

//GYRO CODE
#define Samples 50
#define SCALING_GYRO .9//0.0151
int16_t gyro_raw[3];
long offsetG_Y;
int gyroYDPS;
int highG_y = -5000,lowG_y = 0;
long lastMillis;
double gyroOffset = 0;
bool flag = 1;
unsigned long previousMillis = 0;
unsigned long interval = 50;
double gyroOffSetHeading = 0; //this is the result of adding gyroOffset to gyroOffset
int count = 0; //used for the Pozyx reset

//------------------------------------------------------------------------------
// HELPER CLASSES
//------------------------------------------------------------------------------

class AverageBuffer
{
public:
  AverageBuffer ()
  {
    index = 0;
    size = 0;
    window_start = 0;
    window_end = 0;
  }

  void push (uint32_t value)
  {
    // update circle buffer
    values[index] = value;
    increment(index);
    if (size < MAX_BUFFER_SIZE)
    {
      ++size;
      increment(window_end);
    }
    else
    {
      increment(window_start);
      increment(window_end);
    }
  }

  uint32_t average ()
  {
    if (size == 0)
      return 0;
    uint32_t sum = 0;
    for (uint32_t i = window_start; i != window_end; increment(i))
      sum += values[i];
    return sum / size;
  }

private:
  void increment (uint32_t & index)
  {
    ++index;
    if (index >= size)
      index = 0;
  }

  uint32_t window_start;
  uint32_t window_end;
  uint32_t index;
  uint32_t size;
  uint32_t values [MAX_BUFFER_SIZE];
};

class PozyxLink
{
public:
  PozyxLink ()
  {
    this->anchor_id = 0;
    this->robot_id = 0;
  }
  
  PozyxLink (uint16_t anchor_id, uint16_t robot_id)
  {
    this->anchor_id = anchor_id;
    this->robot_id = robot_id;
  }

  void operator= (PozyxLink const & link)
  {
    this->anchor_id = link.anchor_id;
    this->robot_id = link.robot_id;
  }

  void take_sample ()
  {
    if (this->robot_id == DEVICE_ID)
    {
      Pozyx.doRanging(this->anchor_id, &pozyx_range);
    }
    else
    {
      Pozyx.doRemoteRanging(this->robot_id, this->anchor_id, &pozyx_range);  
    }
    buffer.push(pozyx_range.distance);
  }

  uint16_t average_distance ()
  {
    return buffer.average();
  }

private:
  uint16_t anchor_id;
  uint16_t robot_id;
  device_range_t pozyx_range;
  AverageBuffer buffer;
};

//------------------------------------------------------------------------------
// state
//------------------------------------------------------------------------------

PozyxLink links [4];

double device_x = 0;
double device_y = 0;
double remote_x = 0;
double remote_y = 0;

FT_t ft_handle;

//------------------------------------------------------------------------------
// helper functions
//------------------------------------------------------------------------------

double law_of_cosine (uint32_t a, uint32_t b, uint32_t c)
{
  /*                c
   *       A *-----------* B      Cos(C) = (a^2 + b^2 -c^2)/(2*a*b)
   *          \         /         Cos(A) = (b^2 + c^2 -a^2)/(2*b*c)
   *           \       /          Cos(B) = (c^2 + a^2 -b^2)/(2*c*a)
   *          b \     /  a
   *             \   /
   *              \ /
   *               C
  */           
  long num = (a*a) + (b*b) - (c*c);
  long den = (2*a*b);
  return acos((double)num/(double)den);
}

void sample_ranges ()
{
  int const NUM_SAMPLES = 10;
  for (int i = 0; i < NUM_SAMPLES; ++i)
    for (int j = 0; j < 4; ++j)
      links[j].take_sample();
}

void update_coordinates ()
{
  // get distances from anchors
  uint32_t left_to_device = links[0].average_distance();
  uint32_t right_to_device = links[1].average_distance();
  uint32_t left_to_remote = links[2].average_distance();
  uint32_t right_to_remote = links[3].average_distance();

  // get angles from origin
  double device_angle = law_of_cosine(left_to_device, ANCHOR_DISPLACEMENT, right_to_device);
  double remote_angle = law_of_cosine(left_to_remote, ANCHOR_DISPLACEMENT, right_to_remote);

  // calculate coordinates
  remote_x = (double)left_to_remote * cos(remote_angle);
  remote_y = (double)left_to_remote * sin(remote_angle);
  device_x = (double)left_to_device * cos(device_angle);
  device_y = (double)left_to_device * sin(device_angle);
}

void ft_put (uint8_t byte)
{
  // send byte via UART
  Serial.write(byte);
}

uint8_t ft_get ()
{
  // read a byte from receive buffer
  uint8_t byte = (uint8_t)Serial.read();
}

bool ft_empty ()
{
  int next = Serial.peek();

  // peeked char is -1 if buffer is empty
  return next == -1;
}

//caller function for void loop, makes it easier to use in regards to integrating for sampling interval 
//that will be called in between gyro offset corrections
void updateHeading(){
  sample_ranges();
  update_coordinates();

  //----------------------------------------------------------------------------
  // calculating POZYX outputs
  //----------------------------------------------------------------------------

  // calculate average position (of device and remote)
  double midpoint_x = (device_x + remote_x) / 2;
  double midpoint_y = (device_y + remote_y) / 2;

  // calculate approx. heading in degrees (relative to left anchor)
  double x_component = remote_x - device_x;
  double y_component = remote_y - device_y;
  double magnitude = sqrt(pow(x_component, 2) + pow(y_component, 2));
  double unit_x_component = x_component / magnitude;
  double unit_y_component = y_component / magnitude;
  double const VEC_ANGLE = 270 / RADIANS_TO_DEGREES;
  double heading_unit_vector_x = cos(VEC_ANGLE) * unit_x_component - sin(VEC_ANGLE) * unit_y_component;
  double heading_unit_vector_y = sin(VEC_ANGLE) * unit_x_component + cos(VEC_ANGLE) * unit_y_component;
  double heading_angle = atan2(heading_unit_vector_y, heading_unit_vector_x) * RADIANS_TO_DEGREES + 180;
}

//void function for all gyro calculations
void gyroCorrections(){

//----------------------------------------------------------------------------
// calculating GYRO offset 
//----------------------------------------------------------------------------

      //update the pozyx heading every set interval (a defined constant)
      //unsigned long currentMillis = millis();
      if (abs(millis() - previousMillis) > interval) 
      {
        //update pozyx angle
        updateHeading();
        //update the heading;
        if(isnan(heading_angle))
        {
          flag = 0; //bad heading value, flag is 0
        }
        else
        {
          flag = 1;
        }
        //reset gyro offset
        if(flag)
        {
          gyroOffset = 0;
        }
          previousMillis = millis();  //used for next iteration of loop
      } 

      //obtain the gyro offset
      if((abs(lastMillis - millis())) > 10)
      {
        Pozyx.regRead(POZYX_GYRO_X, (uint8_t*)&gyro_raw, 3*sizeof(int16_t));
        gyroYDPS =  gyro_raw[1]- offsetG_Y;
        if(isWithinFloat(gyroYDPS,lowG_y*1.1,highG_y*1.1))
           gyroYDPS=0;  
        gyroOffset += ((double)gyroYDPS*SCALING_GYRO * ((millis()-lastMillis)/1000.0)/16);
          
        lastMillis = millis();
      } 
      //add the gyro offset to the current heading
      if(flag)
      {
        gyroOffSetHeading = heading_angle + gyroOffset;
      }
      else
      {
        gyroOffSetHeading = gyroOffSetHeading + gyroOffset; //update the heading with only the last good value
#ifdef DEBUG
         Serial.println("gyroOffSetHeading UPDATED");
         Serial.println(gyroOffSetHeading);
         Serial.println("gyroOffset");
         Serial.println(gyroOffset);
#endif
      }
#ifdef DEBUG
      Serial.println("flag");
      Serial.println(flag);
      Serial.print("Gyro Offset Heading: ");
      Serial.println(gyroOffSetHeading);
#endif
}

//------------------------------------------------------------------------------
// initialization
//------------------------------------------------------------------------------

void setup ()
{
  // initialize serial comms
  links[0] = PozyxLink(LEFT_ANCHOR_ID,  DEVICE_ID);
  links[1] = PozyxLink(RIGHT_ANCHOR_ID, DEVICE_ID);
  links[2] = PozyxLink(LEFT_ANCHOR_ID,  REMOTE_ID);
  links[3] = PozyxLink(RIGHT_ANCHOR_ID, REMOTE_ID);

  // initialize POZYX
  if (Pozyx.begin() == POZYX_FAILURE)
  {
    delay(100);
    abort();
  }

  Pozyx.setRangingProtocol(POZYX_RANGE_PROTOCOL_PRECISION, REMOTE_ID);
  Pozyx.setSensorMode(0, REMOTE_ID);
  Pozyx.regWrite(POZYX_POS_FILTER, FILTER_TYPE_MOVINGMEDIAN | B11110000, 8);

  // initialize fast transfer
  Serial.begin(115200);
  FT_Init(&ft_handle, 0x10, ft_put, ft_get, ft_empty);
}

//------------------------------------------------------------------------------
// main loop
//------------------------------------------------------------------------------

void loop ()
{
  //----------------------------------------------------------------------------
  // determine the heading to send via FastTransfer (based on gyro offset AND pozyx calculated heading)
  // calculated heading from pozyx should update less frequently, s.t. gyro gives us an accurate heading 
  // every small interval and pozyx gives us another heading every larger interval, giving us a way to correct
  // the actual heading 
  //----------------------------------------------------------------------------
#ifdef DEBUG

  Serial.println("---------------------------------");

  Serial.print("Device X: ");
  Serial.println(device_x);

  Serial.print("Device Y: ");
  Serial.println(device_y);

  Serial.print("Remote X: ");
  Serial.println(remote_x);

  Serial.print("Remote Y: ");
  Serial.println(remote_y);

  Serial.print("Midpoint X: ");
  Serial.println(midpoint_x);
  Serial.print("Midpoint Y: ");
  Serial.println(midpoint_y);

  Serial.print("Heading angle [deg]: ");
  Serial.println(heading_angle);
  
#else
   
  // send data
  FT_ToSend(&ft_handle, 0x00, (int16_t)midpoint_x);
  FT_ToSend(&ft_handle, 0x01, (int16_t)midpoint_y);
  FT_ToSend(&ft_handle, 0x02, (int16_t)heading_angle);
  FT_Send(&ft_handle, 0x09);

#endif // DEBUG
 
 //this IF block is for periodic system resets, something that we might need to do for 
  //if(count > 30000) //reset every 30 seconds
  //{
      //Serial.println("SYSTEM RESET******************");
      //Pozyx.resetSystem(LEFT_ANCHOR_ID); //keep?
      //Pozyx.resetSystem(RIGHT_ANCHOR_ID);
      //Pozyx.resetSystem(REMOTE_ID);
      //Pozyx.resetSytem(DEVICE_ID);
      //count = 0;   
  //}
      //count++;
}
