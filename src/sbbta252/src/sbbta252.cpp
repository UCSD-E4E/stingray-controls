/*------------------------------------------------------------------------------
 *  Title:    sbBTA252.cpp
 *  Description:  Sending and receiving data with the Seabotix BTA252 thrusters.
 *----------------------------------------------------------------------------*/

/*
 *
 *    Copyright (c) 2010 <iBotics -- www.sdibotics.org>
 *    All rights reserved.
 *
 *    Redistribution and use in source and binary forms, with or without
 *    modification, are permitted provided that the following conditions are
 *    met:
 *
 *    * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 *    * Neither the name of the Stingray, iBotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *    A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *    OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SE// Set up the thruster.RVICES; LOSS OF USE,
 *    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "sbbta252.h"

/*------------------------------------------------------------------------------
 * SBBTA252()
 * Constructor.
 *----------------------------------------------------------------------------*/

SBBTA252::SBBTA252(string _portname, int _baud, int _init_time) : Serial::Serial(_portname, _baud)
{
  // Initialize variables.
  baud      = _baud;
  init_time = _init_time;
  portname  = _portname;

  // Start a new timer.
  timer = new Timing(init_time);

  // Set up the thruster.
  //setup();
} // end SBBTA252()


/*------------------------------------------------------------------------------
 * ~SBBTA252()
 * Destructor.
 *----------------------------------------------------------------------------*/

SBBTA252::~SBBTA252()
{  
  // Close the open file descriptors.
  if (fd > 0)
  {
    close(fd);
  }
  if (Serial::fd > 0)
  {
    close(Serial::fd);
  }
  delete timer;
} // end ~SBBTA252()


/*------------------------------------------------------------------------------
 * void setup()
 * Initializes communications with the Seabotix BTA252 Thruster. Sets up a file
 * descriptor for further communications.
 *----------------------------------------------------------------------------*/

void SBBTA252::setup()
{
  // Declare variables.
  b_thruster_initialized = false;

  // Check if the thruster serial port was opened correctly.
  if (Serial::fd > 0)
  {
  
  	/*
  	ROS_INFO("START SENDING");
  
  	for (int k = 0; k < 256; k++)
  		sendSpeedCommand(k, 50);
  		
  	ROS_INFO("END SENDING");
  	*/
  	
    // Setup each thruster.
    for (int i = 0; i < 5; i++)
    {
      // Initialize the timer
      timer->set();

      // Pick the thruster to consider
      int addr = 1;
      if (i == 0)
      {
        addr = t_left_yaw;
        ROS_INFO("Initializing Left Yaw Thruster (%d)...", addr);
      }
      else if (i == 1)
      {
        addr = t_right_yaw;
        ROS_INFO("Initializing Right Yaw Thruster (%d)...", addr);
      }
      else if (i == 2)
      {
        addr = t_left_roll;
        ROS_INFO("Initializing Left Roll Thruster (%d)...", addr);
      }
      else if (i == 3)
      {
        addr = t_right_roll;
        ROS_INFO("Initializing Right Roll Thruster (%d)...", addr);
      }
      else if (i == 4)
      {
        addr = t_pitch;
        ROS_INFO("Initializing Pitch Thruster (%d)...", addr);
      }

      // Check for valid thruster data until the timer expires or valid data is found.
      while (!timer->checkExpired())
      {
        // Try to initialize the thruster to make sure that we are actually getting valid data from it.
        init(addr);
        if (b_thruster_initialized)
        {
          ROS_INFO("...thruster initialized.");        
          break;
        }
      }
    
      if (!b_thruster_initialized)
      {
        ROS_INFO("...thruster could not be initialized.");
      }
    }
  }

  // Set the file descriptor.
  fd = Serial::fd;
} // end Setup()


/*------------------------------------------------------------------------------
 * void init()
 * Looks for valid status data from the thruster.
 *----------------------------------------------------------------------------*/

void SBBTA252::init(int thruster)
{
  // Initialize variables.
  b_thruster_initialized = false;
  
  // Send the Read Command
  sendReadCommand(thruster);
    
  // Get Status
  b_thruster_initialized = getStatus();
} // end init()


/*------------------------------------------------------------------------------
 * PublishThrusterData()
 * Publish custom compass message.
 *----------------------------------------------------------------------------*/

void SBBTA252::publishMessage(ros::Publisher *pub_thruster_data)
{
  sbbta252::sbbta252data msg;

  msg.speed           = atof(speed.c_str());
  msg.current         = atof(current.c_str());
  msg.motor_temp      = atof(motor_temp.c_str());
  msg.controller_temp = atof(controller_temp.c_str());
  msg.controller_volt = atof(controller_volt.c_str());
  msg.water           = atof(water.c_str());
  
  pub_thruster_data->publish(msg);
} // end publishMessage()


/*------------------------------------------------------------------------------
 * configCallback()
 * Callback function for dynamic reconfigure server.
 *----------------------------------------------------------------------------*/

void SBBTA252::configCallback(sbbta252::sbbta252ParamsConfig& config, uint32_t level)
{
  // Set class variables to new values.
  roll_freq = config.roll_freq;
  pitch_freq = config.pitch_freq;
  yaw_freq = config.yaw_freq;
  depth_freq = config.depth_freq;
  surge_freq = config.surge_freq;
  portname     = config.port.c_str();
  baud         = config.baud;
  t_left_yaw   = config.t_left_yaw;
  t_right_yaw  = config.t_right_yaw;
  t_left_roll  = config.t_left_roll;
  t_right_roll = config.t_right_roll;
  t_pitch      = config.t_pitch;
  init_time    = config.init_time;
  manual       = config.manual;
  thrust_addr  = config.thrust_addr;
  change_addr  = config.change_addr;
  new_addr     = config.new_addr;
  speed_perc   = config.speed_perc;
  get_status   = config.get_status;
  


  // Check to see if we should attempt to reconnect to the thrusters.
  if (config.reconnect)
  {  
    // Use the new thruster settings to reconnect.
    setup();
    ROS_INFO("Using new settings to reconnect to thrusters. Got fd = %d", fd);

    // Reset the reconnect variable.
    config.reconnect = false;
  }
} // end configCallback()


/*------------------------------------------------------------------------------
 * controlsCallback()
 * Callback to receive navigation controls message.
 *----------------------------------------------------------------------------*/

void SBBTA252::controlsCallback(const nav::ControlInputs::ConstPtr& msg)
{
  u_roll  = msg->u_roll;
  u_pitch = msg->u_pitch;
  u_yaw   = msg->u_yaw;
  u_depth = msg->u_depth;
  u_surge = msg->u_surge;
  
  ROS_DEBUG("uR=%lf, uP=%lf, uY=%lf, uD=%lf, uS=%lf", u_roll, u_pitch, u_yaw, u_depth, u_surge);
} // end controlsCallback()


/*------------------------------------------------------------------------------
 * void getStatus()
 * Get status data from a thruster.
 *----------------------------------------------------------------------------*/

bool SBBTA252::getStatus()
{
  // Declare variables.
  int bytes_to_discard = 0;

  // Get the number of bytes available on the serial port.
  getBytesAvailable();

  // Make sure we don't read too many bytes and overrun the buffer.
  if (bytes_available >= SERIAL_MAX_DATA)
  {
    bytes_available = SERIAL_MAX_DATA - 1;
  }
  if (bytes_available + strlen(buf_recv) >= SERIAL_MAX_DATA)
  {
    bytes_to_discard = bytes_available + strlen(buf_recv) - SERIAL_MAX_DATA - 1;
    memmove(buf_recv, &buf_recv[bytes_to_discard], bytes_to_discard);
  }

  // Read data off the serial port.
  if (bytes_available > 0)
  {
    recv();
    
    // Look for entire message.
    if (findMsg())
    {
      // Parse the status.
      return parseStatus();
    }
  }
  
  return false;
} // end getStatus()


/*------------------------------------------------------------------------------
 * sendSpeedCommand()
 * Sends the command for speed of the thruster. The variable speed is a number
 * between -100 and 100.
 *----------------------------------------------------------------------------*/

void SBBTA252::sendSpeedCommand(int thruster, int speed)
{
  // Declare variables.
  char cmd[10];
  Serial::buf_send = cmd;

  // Send command.
  if (Serial::fd > 0)
  {
    cmd[0] = SBBTA252_SC;
    cmd[1] = getAddress(1, thruster);
    cmd[2] = getAddress(2, thruster);
    cmd[3] = getSpeed(1, speed);
    cmd[4] = getSpeed(2, speed);
    cmd[5] = '0';
    cmd[6] = '0';
    cmd[7] = getChecksum(1, cmd, 10);
    cmd[8] = getChecksum(2, cmd, 10);
    cmd[9] = SBBTA252_EC;
    Serial::length_send = 10;
    send();
    usleep(SBBTA252_SERIAL_DELAY);
    
    ROS_DEBUG("Send Speed Command (%d): %s.", thruster, cmd);
  }
} // end sendSpeedCommand()
  

/*------------------------------------------------------------------------------
 * sendReadCommand()
 * Sends the command for to send status.
 *----------------------------------------------------------------------------*/

void SBBTA252::sendReadCommand(int thruster)
{
  // Declare variables.
  char cmd[6];
  Serial::buf_send = cmd;

  // Send command.
  if (Serial::fd > 0)
  {
    cmd[0] = SBBTA252_SC;
    cmd[1] = getAddress(1, thruster);
    cmd[2] = getAddress(2, thruster);
    cmd[3] = getChecksum(1, cmd, 6);
    cmd[4] = getChecksum(2, cmd, 6);
    cmd[5] = SBBTA252_EC;
    Serial::length_send = 6;
    send();
    usleep(SBBTA252_SERIAL_DELAY);
    
    ROS_DEBUG("Send Read Command (%d): %s.", thruster, cmd);
  }
} // end sendReadCommand()


/*------------------------------------------------------------------------------
 * sendChangeAddressCommand()
 * Changes the address of the specified thruster to the hex value specified by
 * new_address.
 *----------------------------------------------------------------------------*/

void SBBTA252::sendChangeAddressCommand(int thruster, int new_address)
{
  // Declare variables
  char cmd[10];
  Serial::buf_send = cmd;

  if (new_address < 1 || new_address > 255)
  {
    ROS_WARN("New address %d is not in the range from 1-255", new_address);
    return;
  }

  // Send command
  if (Serial::fd > 0)
  {
    cmd[0] = SBBTA252_SC;
    cmd[1] = getAddress(1, thruster);
    cmd[2] = getAddress(2, thruster);
    cmd[3] = '0'; // Command specific char
    cmd[4] = 'B'; // Command specific char
    cmd[5] = dec2hex(new_address / 16);
    cmd[6] = dec2hex(new_address % 16);
    cmd[7] = getChecksum(1, cmd, 10);
    cmd[8] = getChecksum(2, cmd, 10);
    cmd[9] = SBBTA252_EC;
    Serial::length_send = 10;
    send();
    usleep(SBBTA252_SERIAL_DELAY);
    
    ROS_DEBUG("Changing address from %d to %d.", thruster, new_address);
  }
} // end SendChangeAddressCommand()


/*------------------------------------------------------------------------------
 * main()
 * Main function to set up ROS node, get compass data and use callbacks to
 * publish compass data.
 *----------------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "thruster_node");
  ros::NodeHandle n;
  ros::NodeHandle private_node_handle_("~");
  ros::Publisher pub_thruster_data = n.advertise<sbbta252::sbbta252data>("thrusterData", 1000);

  // Declare variables.
  string portname;
  int baud;
  int init_time;
  bool reconnect;
  SBBTA252 *thruster;

  // Initialize node parameters.
  private_node_handle_.param("port", portname, string("/dev/ttyS0"));
  private_node_handle_.param("baud", baud, int(115200));
  private_node_handle_.param("init_time", init_time, int(3));
  private_node_handle_.param("reconnect", reconnect, bool(false));

  // Create a new SBBTA252 thruster object.
  thruster = new SBBTA252(portname, baud, init_time);

  // Tell ROS to run this node at the rate that the compass is sending messages to us.
  ros::Rate r(100);

  // Set up a dynamic reconfigure server.
  dynamic_reconfigure::Server<sbbta252::sbbta252ParamsConfig> gain_srv;
  dynamic_reconfigure::Server<sbbta252::sbbta252ParamsConfig>::CallbackType f;
  f = boost::bind(&SBBTA252::configCallback, thruster, _1, _2);
  gain_srv.setCallback(f);
  
  // Create a subscriber.
  // Name the topic, message queue, callback function with class name, and object containing callback function.
  ros::Subscriber sub_message = n.subscribe("controlInputs", 1000, &SBBTA252::controlsCallback, thruster);
  
  // Set up the thruster.
  thruster->setup();

  // Connect to the Seabotix BTA252
  if (thruster->fd < 0)
  {
    ROS_ERROR("Could not connect to thrusters on port %s at %d baud. You can try changing the parameters using the dynamic reconfigure gui.", portname.c_str(), baud);
  }

 //Perry: create a hold for the thruster commands 
  long loopCount = 0;
  float savedRoll = thruster->u_roll;
  float savedPitch = thruster->u_depth;
  float savedYaw = thruster->u_yaw;
  float savedDepth = thruster->u_depth;
  float savedSurge = thruster->u_surge;

  // GABE: scale radians to motor speeds
  float yawScale = 1; // 80 / (2 * M_PI);
  //float rollPitchScale = 1; //80 / M_PI;
  
  // Main loop.thruster
  while (n.ok())
  {
    if (thruster->fd > 0)
    {
      // If we are in manual mode then use the reconfigure commands
      if (thruster->manual)
      {
        ROS_ERROR("Manual mode.");
        if (thruster->change_addr)
        {
          thruster->sendChangeAddressCommand(thruster->thrust_addr, thruster->new_addr);
        }
        
        thruster->sendSpeedCommand(thruster->thrust_addr, thruster->speed_perc);
        /*
        thruster->sendSpeedCommand(2, thruster->speed_perc);
        thruster->sendSpeedCommand(3, thruster->speed_perc);
        thruster->sendSpeedCommand(4, thruster->speed_perc);
        thruster->sendSpeedCommand(5, thruster->speed_perc);
        */
        if (thruster->get_status)
        {
          thruster->sendReadCommand(thruster->thrust_addr);
          thruster->getStatus();
        }
      }
      else
      {
        // Not doing manual commands so use the planner thrust 
        
        if ( loopCount % thruster->roll_freq == 0)
        {

          // GS: save current depth
          savedRoll = thruster->u_roll;
        }
        
if ( loopCount  % thruster->pitch_freq == 0)
        {
          // GS: save current depth
          savedPitch = thruster->u_pitch;
        }
if ( loopCount % thruster->yaw_freq == 0)
        {

          savedYaw = thruster->u_yaw;
        }
if ( loopCount % thruster->depth_freq == 0)
        {

          // GS: save current depth
          savedDepth = thruster->u_depth;
        }

if ( loopCount % thruster->surge_freq == 0)
        {

          savedSurge = thruster->u_surge;
        }


        float negSavedDepth = savedDepth * -1;
        
        // Send the Pitch command
        thruster->sendSpeedCommand(thruster->t_pitch, (thruster->u_pitch * -1) + (negSavedDepth / 2));


        // Send the Roll commands
        thruster->sendSpeedCommand(thruster->t_left_roll, (thruster->u_roll ) + negSavedDepth);
        thruster->sendSpeedCommand(thruster->t_right_roll, (thruster->u_roll*-1) + negSavedDepth);
        loopCount++;
        
        // Send the Yaw commands
        thruster->sendSpeedCommand(thruster->t_left_yaw, thruster->u_yaw * yawScale);
        thruster->sendSpeedCommand(thruster->t_right_yaw, thruster->u_yaw * -1 * yawScale);
      }
    }

    ros::spinOnce();
    r.sleep();
  }

  return 0;
} // end main()


/// !!!!!!!!!!!!!!!!!!!!!!!!!!
/// !!!!!!!!!!!!!!!!!!!!!!!!!!
///               !!!
/// BEGIN HELPER FUNCTIONS !!!
///               !!!
/// !!!!!!!!!!!!!!!!!!!!!!!!!!
/// !!!!!!!!!!!!!!!!!!!!!!!!!!

/*------------------------------------------------------------------------------
 * void GetAddress()
 * Converts the thruster number to hex address.
 *----------------------------------------------------------------------------*/
 
char SBBTA252::getAddress(int bit, int thruster)
{
  char ret;
  
  // Check validity of thruster address
  if (thruster < 0 || thruster > 255)
  {
    ROS_WARN("Thruster address %d is out of range.", thruster);
    return -1;
  }
  
  if (bit == 1)
  {
    ret = dec2hex(thruster / 16);
  }
  else if (bit == 2)
  {
    ret = dec2hex(thruster % 16);
  }
  else
  {
    ROS_WARN("Address bit %d is out of range.", bit);
    ret = -1;
  }
  
  // Return the address bit
  return ret;
} // end getAddress()


/*------------------------------------------------------------------------------
 * void dec2hex()
 * Converst the decimal (0-15) to hex.
 *----------------------------------------------------------------------------*/

char SBBTA252::dec2hex(int dec)
{
  char ret;
  
  if (dec < 0 || dec > 15)
  {
    ROS_WARN("Decimal %d is out of range.", dec);
    return -1;
  }

  if (dec < 10)
  {
    ret = 48 + dec; // 48 is the ASCII value for '0'
  }
  else
  {
    ret = 55 + dec; // 65 is the ASCII value for 'A'
                    // + 9 < dec < 16 ====> 64 < 55 + dec < 71
  }

  return ret;
} // end dec2hex()


/*------------------------------------------------------------------------------
 * void hex2dec()
 * Converst the hex (0x00 - 0xFF) to decimal (0 - 255)
 *----------------------------------------------------------------------------*/

int SBBTA252::hex2dec(char hex[2])
{
  int ret;
  int dec[2];
  
  for (int i=0; i<2; i++)
  {
    /*hex[i] -= 48; // if hex[0] is '0' to '9', then this sets its value to the correct number
    if (0 <= hex[i] && hex[i] <= 10)
    {
      dec[i] = hex[i];
    }
    else if (17 <= hex[i] && hex[i] <= 22)
    {
      dec[i] = hex[i] - 7;
    }*/

    if (hex[i] == '0')
    {
      dec[i] = 0;
    }
    else if (hex[i] == '1')
    {
      dec[i] = 1;
    }
    else if (hex[i] == '2')
    {
      dec[i] = 2;
    }
    else if (hex[i] == '3')
    {
      dec[i] = 3;
    }
    else if (hex[i] == '4')
    {
      dec[i] = 4;
    }
    else if (hex[i] == '5')
    {
      dec[i] = 5;
    }
    else if (hex[i] == '6')
    {
      dec[i] = 6;
    }
    else if (hex[i] == '7')
    {
      dec[i] = 7;
    }
    else if (hex[i] == '8')
    {
      dec[i] = 8;
    }
    else if (hex[i] == '9')
    {
      dec[i] = 9;
    }
    else if (hex[i] == 'A')
    {
      dec[i] = 10;
    }
    else if (hex[i] == 'B')
    {
      dec[i] = 11;
    }
    else if (hex[i] == 'C')
    {
      dec[i] = 12;
    }
    else if (hex[i] == 'D')
    {
      dec[i] = 13;
    }
    else if (hex[i] == 'E')
    {
      dec[i] = 14;
    }
    else if (hex[i] == 'F')
    {
      dec[i] = 15;
    }
  }
  
  ret = (dec[0]*16) + dec[1];
  
  return ret;
} // end hex2dec()


/*------------------------------------------------------------------------------
 * void getChecksum()
 * Caculates the checksum in hex based on the command.
 *----------------------------------------------------------------------------*/

char SBBTA252::getChecksum(int bit, char *cmd, int cmdLength)
{
  char ret;
  int dec = 0;
  int i = 1;
  
  while (i < cmdLength-3)
  {
    dec += hex2dec(&cmd[i]);
    i += 2;
  }
  dec = dec % 256;
  
  if (bit == 1)
  {
    ret = dec2hex(dec / 16);
  }
  else if (bit == 2)
  {
    ret = dec2hex(dec % 16);
  }
  else
  {
    ROS_WARN("Checksum bit %d is out of range.", bit);
    ret = -1;
  }
  
  // Return the checksum bit
  return ret;
} // end getChecksum()


/*------------------------------------------------------------------------------
 * void getDecimal()
 * Get the integer value encoded in str from start to end, ignoring spaces
 *----------------------------------------------------------------------------*/

int SBBTA252::getDecimal(char * str, int start, int end)
{
  while (str[start] == ' ')
  {
    start++;
  }

  if (start > end)
  {
    ROS_WARN("You cannot have a string that starts after it ends");
    return 0; // 0 is a legitimate return value, so maybe change this
  }

  char substring[end - start + 2];
  for (int i = start; i <= end; i++)
  {
    substring[i - start] = str[i];
  }
  substring[end - start + 1] = 0; // null terminates the string

  return atoi(substring); 
} // end getDecimal()


/*------------------------------------------------------------------------------
 * void getSpeed()
 * Get the speed in hex based on speed in percent.
 *----------------------------------------------------------------------------*/

char SBBTA252::getSpeed(int bit, int speed)
{
  char ret;
  
  // Check validity of thruster address
  if (speed < -100 || speed > 100)
  {
    ROS_WARN("Speed %d is out of range. Sending +/- 100.", speed);
    speed = ((speed>0)-(speed<0))*100;
  }
  
  int dec;
  if (speed == 0)
  {
    dec = 128;
  }
  else if (speed > 0)
  {
    dec = speed + 128;
  }
  else if (speed < 0)
  {
    dec = speed + 127;
  }
  
  if (bit == 1)
  {
    ret = dec2hex(dec / 16);
  }
  else if (bit == 2)
  {
    ret = dec2hex(dec % 16);
  }
  else
  {
    ROS_WARN("Address bit %d is out of range.", bit);
    ret = -1;
  }
  
  // Return the address bit
  return ret;
} // end getSpeed()
 

/*------------------------------------------------------------------------------
 * void findMsg()
 * Searches a buffer looking for the start and end sequences.
 *----------------------------------------------------------------------------*/

bool SBBTA252::findMsg()
{
  // Declare variables.
  bool b_found_start = false;
  int i = 0;
  int msg_start_location = 0;
  
  // Look for start character.
  for (i = 0; i < Serial::bytes_recv; i++)
  {
    if (b_found_start)
    {
      if (Serial::buf_recv[i] == '!')
      {
        // Place the last complete message at the beginning of the buffer and throw away any data before that.
        memmove(Serial::buf_recv, Serial::buf_recv + msg_start_location, Serial::bytes_recv - msg_start_location);
        return true;
      }
    }
    // Look for start sequence. Don't assume that there is an end response before another start response.
    if (Serial::buf_recv[i] == '$')
    {
      b_found_start = true;
      msg_start_location = i;
    }
  }
  
  return false;
} // end findMsg()


/*------------------------------------------------------------------------------
 * void parseStatus()
 * Parses a message for thruster Status.
 *----------------------------------------------------------------------------*/

bool SBBTA252::parseStatus()
{
  // Declare variables.
  char *msg = Serial::buf_recv;
  char address[2];
  int rpm, // ranges from -4500 to 4500
      current, // Amps * 10
      motor_temp,
      controller_temp,
      voltage,
      water_detect,
      status,
      faults;

  address[0]      = msg[1];
  address[1]      = msg[2];
  rpm             = getDecimal(msg, 3, 8);
  current         = getDecimal(msg, 10, 12);
  motor_temp      = getDecimal(msg, 14, 16);
  controller_temp = getDecimal(msg, 18, 20);
  voltage         = getDecimal(msg, 22, 24);
  water_detect    = getDecimal(msg, 26, 28);
  status          = getDecimal(msg, 30, 32);
  faults          = getDecimal(msg, 34, 36);

  // check for what status and fault messages are here:
  char checksum[2];

  int cs = hex2dec(address) + rpm + current + motor_temp + controller_temp + voltage + water_detect + status + faults;

  cs = cs % 256;  
  checksum[0] = dec2hex(cs / 16);
  checksum[1] = dec2hex(cs % 16);

  if (checksum[0] != msg[38] || checksum[1] != msg[39])
  {
    ROS_ERROR("We received a bad status message. Checksums didn't match.");
    return false;
  }
  else
  {
    /*
    ROS_ERROR("msg=%s.", msg);
    ROS_ERROR("rpm = %d.", rpm);
    ROS_ERROR("current = %d.", current);
    ROS_ERROR("motor_temp = %d.", motor_temp);
    ROS_ERROR("controller_temp = %d.", controller_temp);
    ROS_ERROR("voltage = %d.", voltage);
    ROS_ERROR("water_detect = %d.", water_detect);
    ROS_ERROR("status = %d.", status);
    ROS_ERROR("faults = %d.", faults);
    */
  
    return true;
  }
} // end parseStatus()

