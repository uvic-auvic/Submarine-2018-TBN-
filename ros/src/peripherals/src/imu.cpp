#include <ros/ros.h>
#include <string>
#include <memory>
#include <serial/serial.h>
#include <iostream>

#include "monitor/GetSerialDevice.h"
#include "peripherals/imu.h"
#include "peripherals/orientation.h"
#include "geometry_msgs/Vector3.h"

#define RESPONSE_MAX_SIZE (23)

#define READ_EEPROM_CMD         (0x28)
#define READ_EEPROM_SIZE        (7)
#define MAG_GAIN_SCALE_ADDR     (232)
#define ACCEL_GAIN_SCALE_ADDR   (230)
#define GYRO_GAIN_SCALE_ADDR    (130)

#define TEMP_CMD        (0x07)
#define TEMP_SIZE       (7)

#define EULER_STAB_CMD  (0x0E)
#define EULER_STAB_SIZE (11)

#define MAG_ACCEL_GYRO_STAB_CMD  (0x02)
#define MAG_ACCEL_GYRO_STAB_SIZE (23)

#define MAG_ACCEL_GYRO_CMD      (0x03)
#define MAG_ACCEL_GYRO_SIZE     (23)

using rosserv = ros::ServiceServer;
using imu_msg = peripherals::imu;

class imu{
public:
    imu(const std::string & port, int baud_rate = 38400, int timeout = 3000);
    ~imu();
    bool get_temperature(double &temperature);
    bool get_euler_stable(peripherals::orientation &euler_angles);
    bool get_mag_accel_gyro_stable(geometry_msgs::Vector3 &mag, geometry_msgs::Vector3 &accel, 
            geometry_msgs::Vector3 &gyro, double &time);
    bool get_mag_accel_gyro(geometry_msgs::Vector3 &mag, geometry_msgs::Vector3 &accel, 
            geometry_msgs::Vector3 &gyro, double &time);
private:
    void write(uint8_t command, int response_bytes = 0);
    bool verify_response(int response_bytes);
    std::unique_ptr<serial::Serial> connection = nullptr;
    uint8_t * response_buffer = nullptr;
    double mag_gain_scale = 1;
    double accel_gain_scale = 1;
    double gyro_gain_scale = 1;
};

imu::imu(const std::string & port, int baud_rate, int timeout) {
    ROS_INFO("Connecting to imu on port: %s", port.c_str());
    connection = std::unique_ptr<serial::Serial>(new serial::Serial(port, (u_int32_t) baud_rate, serial::Timeout::simpleTimeout(timeout)));
    response_buffer = new uint8_t[RESPONSE_MAX_SIZE];

    // Need to get calibrated gain scales for magnetometer, gyroscope and accelerometer
    ROS_INFO("Acquiring the magnetometer gain scale.");
    write(READ_EEPROM_CMD);
    write(MAG_GAIN_SCALE_ADDR >> 8);
    write(MAG_GAIN_SCALE_ADDR & 0xFF, READ_EEPROM_SIZE);
    if(verify_response(READ_EEPROM_SIZE)) {     
        mag_gain_scale = ((int16_t) ((response_buffer[1] << 8) | response_buffer[2]));
    }
    else {      
        ROS_INFO("Bad Checksum, failed to get MagGainScale");
    }

    ROS_INFO("Acquiring the accelerometer gain scale.");
    write(READ_EEPROM_CMD);
    write(ACCEL_GAIN_SCALE_ADDR >> 8);
    write(ACCEL_GAIN_SCALE_ADDR & 0xFF, READ_EEPROM_SIZE);
    if(verify_response(READ_EEPROM_SIZE)) {     
        accel_gain_scale = ((int16_t) ((response_buffer[1] << 8) | response_buffer[2]));
    }
    else {      
        ROS_INFO("Bad Checksum, failed to get AccelGainScale");
    }

    ROS_INFO("Acquiring the gyroscope gain scale.");
    write(READ_EEPROM_CMD);
    write(GYRO_GAIN_SCALE_ADDR >> 8);
    write(GYRO_GAIN_SCALE_ADDR & 0xFF, READ_EEPROM_SIZE);
    if(verify_response(READ_EEPROM_SIZE)) {     
        gyro_gain_scale = ((int16_t) ((response_buffer[1] << 8) | response_buffer[2]));
    }
    else {      
        ROS_INFO("Bad Checksum, failed to get GyroGainScale");
    }
    
    ROS_INFO("Gain scales: M:%f, A:%f, G:%f", mag_gain_scale, accel_gain_scale, gyro_gain_scale);
}

imu::~imu() {
    connection->close();
    delete [] response_buffer;
}

void imu::write(uint8_t command, int response_bytes)
{
    connection->write(&command, 1);
    //connection->flushInput();
    
    // Put result into vector if need be
    if (response_bytes) {
        connection->read(response_buffer, (size_t) response_bytes);
    }
}

bool imu::verify_response(int response_bytes) {
    if((response_bytes > 0) && (response_bytes <= RESPONSE_MAX_SIZE)) {
        // Each response contains a checksum (last two bytes)
        uint16_t response_checksum = (response_buffer[response_bytes - 2] << 8) | (response_buffer[response_bytes - 1]);
        
        // Can compute the checksum by summing all bytes preceding the checksum as 16 bit numbers where
        // MSB of header is 0.
        uint16_t computed_checksum = response_buffer[0];
        for( int i = 1; i < (response_bytes - 2); i+=2) {    
            computed_checksum += (response_buffer[i] << 8) | (response_buffer[i+1]);
        }

        // Check if the checksums are the same
        return response_checksum == computed_checksum;
    }
    else {
        return false;
    }
}

bool imu::get_temperature(double &temperature) {
    // Send temperature command
    write(TEMP_CMD, TEMP_SIZE);

    // Verify the checksum of the response
    if(!verify_response(TEMP_SIZE)) {
        ROS_INFO("Bad checksum");
        return false;
    }
    
    // Compute temperature (first 2 non-header bytes of response)
    temperature = (( (double)(((int) response_buffer[1] << 8) | (int) response_buffer[2]) * 5.0 / 65536) - 0.5) * 100.0;

    return true;
}

bool imu::get_euler_stable(peripherals::orientation &euler_angles) {  
    // Send stable euler angles command
    write(EULER_STAB_CMD, EULER_STAB_SIZE);

    // Verify the checksum of the response
    if(!verify_response(EULER_STAB_SIZE)) {      
        ROS_INFO("Bad Checksum");
        return false;
    }

    euler_angles.roll = ((int16_t)((response_buffer[1] << 8) | response_buffer[2])) * 360.0 / 65536.0;
    euler_angles.pitch = ((int16_t)((response_buffer[3] << 8) | response_buffer[4])) * 360.0 / 65536.0;
    euler_angles.yaw = ((int16_t)((response_buffer[5] << 8) | response_buffer[6])) * 360.0 / 65536.0;

    return true;
}

bool imu::get_mag_accel_gyro_stable
(
    geometry_msgs::Vector3 &mag, 
    geometry_msgs::Vector3 &accel, 
    geometry_msgs::Vector3 &gyro, 
    double &time
)
{
    // Send the stable vectors command
    write(MAG_ACCEL_GYRO_STAB_CMD, MAG_ACCEL_GYRO_STAB_SIZE);

    // Verify the checksum of the response
    if(!verify_response(MAG_ACCEL_GYRO_STAB_SIZE)) {      
        ROS_INFO("Bad Checksum");
        return false;
    }

    // Get magnetometer vector (Gauss)
    mag.x = ((int16_t)((response_buffer[1] << 8) | response_buffer[2])) * mag_gain_scale / 32768000.0;
    mag.y = ((int16_t)((response_buffer[3] << 8) | response_buffer[4])) * mag_gain_scale / 32768000.0;
    mag.z = ((int16_t)((response_buffer[5] << 8) | response_buffer[6])) * mag_gain_scale / 32768000.0;

    // Get accelerometer vector (G's)
    accel.x = ((int16_t)((response_buffer[7] << 8) | response_buffer[8])) * accel_gain_scale / 32768000.0;
    accel.y = ((int16_t)((response_buffer[9] << 8) | response_buffer[10])) * accel_gain_scale / 32768000.0;
    accel.z = ((int16_t)((response_buffer[11] << 8) | response_buffer[12])) * accel_gain_scale / 32768000.0;

    // Get gyroscope vector (rad/sec)
    gyro.x = ((int16_t)((response_buffer[13] << 8) | response_buffer[14])) * gyro_gain_scale / 32768000.0;
    gyro.y = ((int16_t)((response_buffer[15] << 8) | response_buffer[16])) * gyro_gain_scale / 32768000.0;
    gyro.z = ((int16_t)((response_buffer[17] << 8) | response_buffer[17])) * gyro_gain_scale / 32768000.0;

    // Get timestamp (ms)
    time = ((uint16_t)((response_buffer[19] << 8) | response_buffer[20])) * 6.5536;

    return true;
}

bool imu::get_mag_accel_gyro
(
    geometry_msgs::Vector3 &mag, 
    geometry_msgs::Vector3 &accel, 
    geometry_msgs::Vector3 &gyro, 
    double &time
) 
{        
    // Send the command to get instantaneous vectors
    write(MAG_ACCEL_GYRO_CMD, MAG_ACCEL_GYRO_SIZE);

    // Verify the checksum of the response
    if(!verify_response(MAG_ACCEL_GYRO_SIZE)) {      
        ROS_INFO("Bad Checksum");
        return false;
    }

    // Get magnetometer vector (Gauss)
    mag.x = ((int16_t)((response_buffer[1] << 8) | response_buffer[2])) * mag_gain_scale / 32768000.0;
    mag.y = ((int16_t)((response_buffer[3] << 8) | response_buffer[4])) * mag_gain_scale / 32768000.0;
    mag.z = ((int16_t)((response_buffer[5] << 8) | response_buffer[6])) * mag_gain_scale / 32768000.0;

    // Get accelerometer vector (G's)
    accel.x = ((int16_t)((response_buffer[7] << 8) | response_buffer[8])) * accel_gain_scale / 32768000.0;
    accel.y = ((int16_t)((response_buffer[9] << 8) | response_buffer[10])) * accel_gain_scale / 32768000.0;
    accel.z = ((int16_t)((response_buffer[11] << 8) | response_buffer[12])) * accel_gain_scale / 32768000.0;

    // Get gyroscope vector (rad/sec)
    gyro.x = ((int16_t)((response_buffer[13] << 8) | response_buffer[14])) * gyro_gain_scale / 32768000.0;
    gyro.y = ((int16_t)((response_buffer[15] << 8) | response_buffer[16])) * gyro_gain_scale / 32768000.0;
    gyro.z = ((int16_t)((response_buffer[17] << 8) | response_buffer[17])) * gyro_gain_scale / 32768000.0;

    // Get timestamp (ms)
    time = ((uint16_t)((response_buffer[19] << 8) | response_buffer[20])) * 6.5536;

    return true;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "imu");
    ros::NodeHandle nh("~");

    monitor::GetSerialDevice srv;
    nh.getParam("device_id", srv.request.device_id);

    // Declare publisher
    ros::Publisher pub = nh.advertise<peripherals::imu>("imu_sensor", 10);

    /* Wait for callbacks */
    imu dev("/dev/ttyS3");
    ros::Rate r(10);
    while(ros::ok()) {
        peripherals::imu msg;
        bool valid_msg = true;

        // Get the Temperature of the IMU
        valid_msg = dev.get_temperature(msg.temperature) && valid_msg;

        // Get the Stabilised Euler Angles
        valid_msg = dev.get_euler_stable(msg.euler_angles) && valid_msg;

        // Get the Stabilised IMU Sensor Vectors
        valid_msg = dev.get_mag_accel_gyro_stable(msg.stabilised_magnetic_field, 
                msg.stabilised_acceleration, msg.compensated_angular_rate, msg.stabilised_vectors_timestamp) && valid_msg;

        // Get the Instantaneous IMU Sensor Vectors
        valid_msg = dev.get_mag_accel_gyro(msg.magnetic_field, msg.acceleration, msg.angular_rate, 
                msg.instantaneous_vectors_timestamp) && valid_msg;

        if(valid_msg) {   
            // Publish message
            pub.publish(msg);

            ROS_INFO("Temperature: %f", msg.temperature);
            ROS_INFO("Euler Angles: P:%f, R:%f, Y:%f", msg.euler_angles.pitch, msg.euler_angles.roll, msg.euler_angles.yaw);
            ROS_INFO("Stabilised Mag: X:%f, Y:%f, Z:%f", msg.stabilised_magnetic_field.x, msg.stabilised_magnetic_field.y, 
                    msg.stabilised_magnetic_field.z);
            ROS_INFO("Stabilised Accel: X:%f, Y:%f, Z:%f", msg.stabilised_acceleration.x, msg.stabilised_acceleration.y, 
                    msg.stabilised_acceleration.z);
            ROS_INFO("Compensated Gyro: X:%f, Y:%f, Z:%f", msg.compensated_angular_rate.x, msg.compensated_angular_rate.y, 
                    msg.compensated_angular_rate.z);
            ROS_INFO("Stabilised Vector Timestamp: %f", msg.stabilised_vectors_timestamp);
            ROS_INFO("Mag: X:%f, Y:%f, Z:%f", msg.magnetic_field.x, msg.magnetic_field.y, 
                    msg.magnetic_field.z);
            ROS_INFO("Accel: X:%f, Y:%f, Z:%f", msg.acceleration.x, msg.acceleration.y, 
                    msg.acceleration.z);
            ROS_INFO("Gyro: X:%f, Y:%f, Z:%f", msg.angular_rate.x, msg.angular_rate.y, 
                    msg.angular_rate.z);
            ROS_INFO("Instantaneous Vector Timestamp: %f\n", msg.instantaneous_vectors_timestamp);
        }
        else {  
            ROS_INFO("Invalid message.\n");
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

