#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>

#include <serial/serial.h>
#include <string>

// serial protocol
#define SERIAL_OUT_ENCODERS 1
#define SERIAL_OUT_PWM 2
#define SERIAL_OUT_LIDAR0 3
#define SERIAL_OUT_LIDAR1 4

#define SERIAL_IN_ENCODERS 1
#define SERIAL_IN_LIDAR 2

#define SERIAL_START_BYTES 13
#define SERIAL_END_BYTE 10

// servo config
#define SERVO_STEP 5
#define SERVO_START 30.0
#define SERVO_END 150.0
#define SERVO_DEGRES 180.0
#define DEG_TO_RAD 0.0174532925

// main data  sizes
#define ENCODER_UNION_MAX 4
#define PWM_UNION_MAX 4
#define LIDAR_UNION_MAX 512

// data trasfer union
union u_tag {
    uint8_t b[4];
    int32_t i;
    float f;
};

// data transfer variables
union u_tag pwm_data[PWM_UNION_MAX];
union u_tag enc_data[ENCODER_UNION_MAX];
union u_tag lidar_data[LIDAR_UNION_MAX];
union u_tag outPacket[4];
uint16_t lidarCmd = 0;
bool newLidarCmd = false;
bool newPwmCmd = false;

void leftPwmCallback(std_msgs::Float32 msg) {
  //ROS_INFO("motorLeftCallback: %f", msg.data);
  pwm_data[0].i = (int32_t)msg.data;
  newPwmCmd = true;
}

void rightPwmCallback(std_msgs::Float32 msg) {
  //ROS_INFO("motorLeftCallback: %f", msg.data);
  pwm_data[1].i = (int32_t)msg.data;
  newPwmCmd = true;
}

void lidarCallback(std_msgs::Int16 msg) {
  //ROS_INFO("lidarCallback: %f", msg.data);
  lidarCmd = (int16_t)msg.data;
  newLidarCmd = true;
}

int main(int argc, char **argv) {

  int i;
  int rate;

  serial::Serial ser;

  std::string port;
  std::string tf_parent_frame_id;
  std::string tf_frame_id;
  std::string frame_id;

  //setup ros
  ros::init(argc, argv, "stm32_base1_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param<std::string>("port", port, "/dev/ttyAMA0");
  pnh.param<std::string>("tf_parent_frame_id", tf_parent_frame_id, "lidar_base");
  pnh.param<std::string>("tf_frame_id", tf_frame_id, "lidar_link");
  pnh.param<std::string>("frame_id", frame_id, "lidar_link");
  pnh.param<int>("rate", rate, 50);

  // setup ros publishers
  ros::Publisher left_encoder_pub = nh.advertise<std_msgs::Int64>("wheel_left", 100);
  ros::Publisher right_encoder_pub = nh.advertise<std_msgs::Int64>("wheel_right", 100);
  ros::Publisher laser_scan_pub = nh.advertise<sensor_msgs::LaserScan>("laser_scan", 50);

  // setup ros subscribers
  ros::Subscriber left_pwm_sub = nh.subscribe("motor_left", 100, leftPwmCallback);
  ros::Subscriber right_pwm_sub = nh.subscribe("motor_right", 100, rightPwmCallback);
  ros::Subscriber lidar_sub = nh.subscribe("lidar", 100, lidarCallback);

  // setup ros variables
  ros::Rate loop_rate(rate);
  std_msgs::Int64 msgEnc;
  sensor_msgs::LaserScan msgLaser;

  // assure that main data variables are zero
  for(i = 0; i < PWM_UNION_MAX; i++) pwm_data[i].i = 0;
  for(i = 0; i < ENCODER_UNION_MAX; i++) enc_data[i].i = 0;
  for(i = 0; i < LIDAR_UNION_MAX; i++) lidar_data[i].i = 0;

  // pwm velocity
  pwm_data[0].i = 0;
  pwm_data[1].i = 0;

  // serial
  std::string input;
  std::string read;

  bool serialMsgStart = false;
  int serialMsgLen = 0;
  int serialMsgLenExpected = 0;
  int serialMsgStartCounter = 0;
  int serialMsgStartIndex = 0;
  int serialMsgCmd = 0;

  int laserScanTime = 0;
  int num_readings = 25;
  // set constant LaserRange fields
  msgLaser.header.frame_id = frame_id;
  msgLaser.angle_min = -((SERVO_DEGRES / 2 - SERVO_START) * DEG_TO_RAD);
  msgLaser.angle_max = ((SERVO_END - SERVO_START)/2) * DEG_TO_RAD;
  msgLaser.angle_increment = SERVO_STEP * DEG_TO_RAD;

  //msgLaser.time_increment = (float)(1.0 / laser_frequency) / (float)num_readings;
  //msgLaser.scan_time = 1 / laser_frequency; //(millis()-scan_start)/1000.0;

  msgLaser.range_min = 0.04;
  msgLaser.range_max = 1.7;

  /*
      if(ser.isOpen()) {
        ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized and opened.");

        for(i = 0; i < 3; i++) {
          // stop motors
          outPacket[0].b[0] = SERIAL_START_BYTES;
          outPacket[0].b[1] = SERIAL_START_BYTES;
          outPacket[0].b[2] = 0;
          outPacket[0].b[3] = SERIAL_OUT_PWM;
          outPacket[1].i = pwm_data[0].i;
          outPacket[2].i = pwm_data[1].i;
          outPacket[3].b[0] = 0;
          outPacket[3].b[1] = 0;
          outPacket[3].b[2] = 0;
          outPacket[3].b[3] = SERIAL_END_BYTE;
          ser.write((uint8_t*)outPacket, 16);
        }

        for(i = 0; i < 3; i++) {
          // reset encoders
          outPacket[0].b[0] = SERIAL_START_BYTES;
          outPacket[0].b[1] = SERIAL_START_BYTES;
          outPacket[0].b[2] = 0;
          outPacket[0].b[3] = SERIAL_OUT_ENCODERS;
          outPacket[1].i = 0;
          outPacket[2].i = 0;
          outPacket[3].b[0] = 0;
          outPacket[3].b[1] = 0;
          outPacket[3].b[2] = 0;
          outPacket[3].b[3] = SERIAL_END_BYTE;
          ser.write((uint8_t*)outPacket, 16);
        }

        for(i = 0; i < 3; i++) {
          // stop lidar
          outPacket[0].b[0] = SERIAL_START_BYTES;
          outPacket[0].b[1] = SERIAL_START_BYTES;
          outPacket[0].b[2] = 0;
          outPacket[0].b[3] = SERIAL_OUT_LIDAR0;
          outPacket[1].i = 0;
          outPacket[2].i = 0;
          outPacket[3].b[0] = 0;
          outPacket[3].b[1] = 0;
          outPacket[3].b[2] = 0;
          outPacket[3].b[3] = SERIAL_END_BYTE;
          ser.write((uint8_t*)outPacket, 16);
        }
      }
  */

  ROS_INFO("setup end");

  while (ros::ok()) {

   //ROS_INFO("ML");

    try {
      if (ser.isOpen()){
        //ROS_INFO("SO");
        // read string from serial device
        if(ser.available()) {
          //ROS_INFO("SR");
          read = ser.read(ser.available());
          //ROS_INFO("read %i new chars from serial, adding to %i characters of old input.", (int)read.size(), (int)input.size());
          input += read;
	  // check for a msg header
          if(!serialMsgStart) {
            //printf("seek start\n"); // dbg
            // need to match 3 bytes for a possible start
            for(int f = 0; f < (int)input.size(); f++) {
              switch(serialMsgStartCounter) {
                case 0:
                  if(input[f] == SERIAL_START_BYTES) {
                    serialMsgStartCounter++;
                  }
                  break;
                case 1:
                  if(input[f] == SERIAL_START_BYTES) {
                    serialMsgStartCounter++;
                  } else {
                    serialMsgStartCounter = 0;
                  }
                  break;
                case 2:
                  if(input[f] == 0) {
                    serialMsgStartCounter++;
                  } else {
                    serialMsgStartCounter = 0;
                  }
                  break;
                case 3:
                  if(input[f] == SERIAL_IN_ENCODERS) {
                    serialMsgStart = true;
                    serialMsgLenExpected = 15;
                    serialMsgStartIndex = f-3;
                    serialMsgStartCounter = 0;
                  } else if(input[f] == SERIAL_IN_LIDAR) {
                    serialMsgStart = true;
                    serialMsgLenExpected = 111;
                    serialMsgStartIndex = f-3;
                    serialMsgStartCounter = 0;
                  } else {
                    serialMsgStartCounter = 0;
                  }
                  break;
              }
              //printf("%d ", input[f]); // dbg
              if(serialMsgStart) break;
            }
            //printf("\n"); // dbg
          }

          // erase input buffer before start
          if(serialMsgStartIndex > 0 ) {
             //printf("erase beguin\n"); // dbg
             input.erase(0, serialMsgStartIndex);
             serialMsgStartIndex = 0;
          }

          // debug
          /*
          printf("input \n");
          for(int f = 0; f < (int)input.size(); f++) {
            printf("%d ", input[f]);
          }
          printf("\n");
          */

          // check for the end of message
          if(serialMsgStart) {
            //printf("seek end\n");
            for(int f = 0; f < (int)input.size(); f++) {
              //printf("%d %d ", f, input[f]); // dbg
              //printf("%d ", input[f]); // dbg
              if(input[f] == SERIAL_END_BYTE ) {
                if(f == serialMsgLenExpected) {
                  serialMsgCmd = input[3];
                  //printf("end found %d %d \n", f, serialMsgCmd); // dbg
                  break;
                }
              }
            }
            //printf("\n");
          }

          // process available cmd
          if(serialMsgCmd) {
	    switch(serialMsgCmd) {
              case SERIAL_IN_ENCODERS:
                enc_data[0].b[0] = input[4];
                enc_data[0].b[1] = input[5];
                enc_data[0].b[2] = input[6];
                enc_data[0].b[3] = input[7];

                enc_data[1].b[0] = input[8];
                enc_data[1].b[1] = input[9];
                enc_data[1].b[2] = input[10];
                enc_data[1].b[3] = input[11];
                break;
              case SERIAL_IN_LIDAR:

                // get scantime
                lidar_data[0].b[0] = input[4];
                lidar_data[0].b[1] = input[5];
                lidar_data[0].b[2] = input[6];
                lidar_data[0].b[3] = input[7];
                laserScanTime = lidar_data[0].i;

                msgLaser.time_increment = (float)((float)laserScanTime/1000.0/(float)num_readings);
                msgLaser.scan_time = (float)((float)laserScanTime/1000.0);

                //printf("%d %f %f\n", laserScanTime, (float)((float)laserScanTime/1000.0/(float)num_readings), (float)((float)laserScanTime/1000.0));

                for(int f = 8; f < serialMsgLenExpected; f += 4) {
                  lidar_data[f/4-2].b[0] = input[f];
                  lidar_data[f/4-2].b[1] = input[f+1];
                  lidar_data[f/4-2].b[2] = input[f+2];
                  lidar_data[f/2-2].b[3] = input[f+3];
                }

                msgLaser.ranges.resize(num_readings);
                msgLaser.intensities.resize(num_readings);
                for(unsigned int i = 0; i < num_readings; ++i){
                  msgLaser.ranges[i] = (float)((float)lidar_data[i].i/1000.0);
                  //printf("%d ", lidar_data[i].i ); // dbg
                  msgLaser.intensities[i] = 0;
                }
                msgLaser.header.stamp = ros::Time::now();
                //printf("\n"); // dbg

                // publish
                laser_scan_pub.publish(msgLaser);
                break;

            }
            //printf("erase 0 to %d \n", (serialMsgLenExpected + 1));
            // clean last msg & reset serial msg control
            input.erase(0, serialMsgLenExpected + 1);
            serialMsgStart = false;
            serialMsgLenExpected = 0;
            serialMsgStartIndex = 0;
            serialMsgCmd = 0;

            // bof dbg
            /*
            printf("remain\n");
            for(int f = 0; f < (int)input.size(); f++) {
              printf("%d ", input[f]); 
            }
            printf("\n");
            */
            // eof dbg

	  }
          //printf("enc: %d %d\n", enc_data[0].i ,enc_data[1].i);

        }

      } else {

        // try and open the serial port
        try {
          ser.setPort(port);
          ser.setBaudrate(115200);
          serial::Timeout to = serial::Timeout::simpleTimeout(1000);
          ser.setTimeout(to);
          ser.open();
        }
        catch (serial::IOException& e) {
          ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
          ros::Duration(5).sleep();
        }

        if(ser.isOpen()) {
          ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized and opened.");

          //
          // send inital commands to microcontroller
          //

          for(i = 0; i < 3; i++) {
            // stop motors
            outPacket[0].b[0] = SERIAL_START_BYTES;
            outPacket[0].b[1] = SERIAL_START_BYTES;
            outPacket[0].b[2] = 0;
            outPacket[0].b[3] = SERIAL_OUT_PWM;
            outPacket[1].i = pwm_data[0].i;
            outPacket[2].i = pwm_data[1].i;
            outPacket[3].b[0] = 0;
            outPacket[3].b[1] = 0;
            outPacket[3].b[2] = 0;
            outPacket[3].b[3] = SERIAL_END_BYTE;
            ser.write((uint8_t*)outPacket, 16);
          }

          for(i = 0; i < 3; i++) {
            // reset encoders
            outPacket[0].b[0] = SERIAL_START_BYTES;
            outPacket[0].b[1] = SERIAL_START_BYTES;
            outPacket[0].b[2] = 0;
            outPacket[0].b[3] = SERIAL_OUT_ENCODERS;
            outPacket[1].i = 0;
            outPacket[2].i = 0;
            outPacket[3].b[0] = 0;
            outPacket[3].b[1] = 0;
            outPacket[3].b[2] = 0;
            outPacket[3].b[3] = SERIAL_END_BYTE;
            ser.write((uint8_t*)outPacket, 16);
          }

          for(i = 0; i < 3; i++) {
            // stop lidar
            outPacket[0].b[0] = SERIAL_START_BYTES;
            outPacket[0].b[1] = SERIAL_START_BYTES;
            outPacket[0].b[2] = 0;
            outPacket[0].b[3] = SERIAL_OUT_LIDAR0;
            outPacket[1].i = 0;
            outPacket[2].i = 0;
            outPacket[3].b[0] = 0;
            outPacket[3].b[1] = 0;
            outPacket[3].b[2] = 0;
            outPacket[3].b[3] = SERIAL_END_BYTE;
            ser.write((uint8_t*)outPacket, 16);
          }
        }
      }
    }

    catch (serial::IOException& e) {
      ROS_ERROR_STREAM("Error reading from the serial port " << ser.getPort() << ". Closing connection.");
      ser.close();
    }

    if(newPwmCmd) {
      // send PWM power to motors
      outPacket[0].b[0] = SERIAL_START_BYTES;
      outPacket[0].b[1] = SERIAL_START_BYTES;
      outPacket[0].b[2] = 0;
      outPacket[0].b[3] = SERIAL_OUT_PWM;
      outPacket[1].i = pwm_data[0].i;
      outPacket[2].i = pwm_data[1].i;
      outPacket[3].b[0] = 0;
      outPacket[3].b[1] = 0;
      outPacket[3].b[2] = 0;
      outPacket[3].b[3] = SERIAL_END_BYTE;
      ser.write((uint8_t*)outPacket, 16);
      newPwmCmd = false;
    }

    if(newLidarCmd) {
      // prepare cmd packet
      outPacket[0].b[0] = SERIAL_START_BYTES;
      outPacket[0].b[1] = SERIAL_START_BYTES;
      outPacket[0].b[2] = 0;
      outPacket[0].b[3] = 0;
      outPacket[1].i = 0;
      outPacket[2].i = 0;
      outPacket[3].b[0] = 0;
      outPacket[3].b[1] = 0;
      outPacket[3].b[2] = 0;
      outPacket[3].b[3] = SERIAL_END_BYTE;
      switch(lidarCmd) {
       case 0:
         outPacket[0].b[3] = SERIAL_OUT_LIDAR0;
         break;
       case 1:
         outPacket[0].b[3] = SERIAL_OUT_LIDAR1;
         break;
      }
      ser.write((uint8_t*)outPacket, 16);
      newLidarCmd = false;
    }

    // publish left encoder ros topic
    msgEnc.data = enc_data[0].i;
    left_encoder_pub.publish(msgEnc);

    // publish right encoder  ros topic
    msgEnc.data = enc_data[1].i;
    right_encoder_pub.publish(msgEnc);

    ros::spinOnce();
    loop_rate.sleep();

  }

}
