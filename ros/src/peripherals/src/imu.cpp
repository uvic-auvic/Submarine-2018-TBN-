#include <ros/ros.h>
#include <string>
#include <memory>
#include <serial/serial.h>
#include <iostream>

#include "monitor/GetSerialDevice.h"
#include "peripherals/imu.h"

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
    void get_temperature();
    void get_euler_stable();
    void get_mag_accel_gyro_stable();
    void get_mag_accel_gyro();
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

void imu::get_temperature() {
    // Send temperature command
    write(TEMP_CMD, TEMP_SIZE);

    // Verify the checksum of the response
    if(verify_response(TEMP_SIZE)) {
        // Compute temperature (first 2 non-header bytes of response)
        double temperature = (( (double)(((int) response_buffer[1] << 8) | (int) response_buffer[2]) * 5.0 / 65536) - 0.5) * 100.0;
        
        // Print the temperature to console
        ROS_INFO("Temp is : %f", temperature);
    }
    else {      
        ROS_INFO("Bad Checksum");
    }
    
    // Print the individual byte values from the response
    ROS_INFO("Byte values are: ");
    for (int i= 0; i < TEMP_SIZE; i ++) {
        ROS_INFO(" Byte %d : %x ", i+1, response_buffer[i]);
    }
    ROS_INFO("\n");
}

void imu::get_euler_stable() {  
    // Send stable euler angles command
    write(EULER_STAB_CMD, EULER_STAB_SIZE);

    // Verify the checksum of the response
    if(verify_response(EULER_STAB_SIZE)) {      
        double roll = ((int16_t)((response_buffer[1] << 8) | response_buffer[2])) * 360.0 / 65536.0;
        double pitch = ((int16_t)((response_buffer[3] << 8) | response_buffer[4])) * 360.0 / 65536.0;
        double yaw = ((int16_t)((response_buffer[5] << 8) | response_buffer[6])) * 360.0 / 65536.0;

        ROS_INFO("Roll: %f", roll);
        ROS_INFO("Pitch: %f", pitch);
        ROS_INFO("Yaw: %f", yaw);
    }
    else {      
        ROS_INFO("Bad Checksum");
    }

    // Print the individual byte values from the response
    ROS_INFO("Byte values are: ");
    for (int i= 0; i < EULER_STAB_SIZE; i ++) {
        ROS_INFO(" Byte %d : %x ", i+1, response_buffer[i]);
    }
    ROS_INFO("\n");
}

void imu::get_mag_accel_gyro_stable() { 
    // Send the stable vectors command
    write(MAG_ACCEL_GYRO_STAB_CMD, MAG_ACCEL_GYRO_STAB_SIZE);

    // Verify the checksum of the response
    if(verify_response(MAG_ACCEL_GYRO_STAB_SIZE)) {      
        // Get magnetometer vector (Gauss)
        double mag_x = ((int16_t)((response_buffer[1] << 8) | response_buffer[2])) * mag_gain_scale / 32768000.0;
        double mag_y = ((int16_t)((response_buffer[3] << 8) | response_buffer[4])) * mag_gain_scale / 32768000.0;
        double mag_z = ((int16_t)((response_buffer[5] << 8) | response_buffer[6])) * mag_gain_scale / 32768000.0;

        // Get accelerometer vector (G's)
        double accel_x = ((int16_t)((response_buffer[7] << 8) | response_buffer[8])) * accel_gain_scale / 32768000.0;
        double accel_y = ((int16_t)((response_buffer[9] << 8) | response_buffer[10])) * accel_gain_scale / 32768000.0;
        double accel_z = ((int16_t)((response_buffer[11] << 8) | response_buffer[12])) * accel_gain_scale / 32768000.0;

        // Get gyroscope vector (rad/sec)
        double gyro_x = ((int16_t)((response_buffer[13] << 8) | response_buffer[14])) * gyro_gain_scale / 32768000.0;
        double gyro_y = ((int16_t)((response_buffer[15] << 8) | response_buffer[16])) * gyro_gain_scale / 32768000.0;
        double gyro_z = ((int16_t)((response_buffer[17] << 8) | response_buffer[17])) * gyro_gain_scale / 32768000.0;

        // Get timestamp (ms)
        double time = ((int16_t)((response_buffer[19] << 8) | response_buffer[20])) * 6.5536;

        ROS_INFO("Magnetometer (Gauss): X:%f, Y:%f, Z:%f", mag_x, mag_y, mag_z);
        ROS_INFO("Accelerometer (G's): X:%f, Y:%f, Z:%f", accel_x, accel_y, accel_z);
        ROS_INFO("Gyroscope (rad/sec): X:%f, Y:%f, Z:%f", gyro_x, gyro_y, gyro_z);
        ROS_INFO("Timestamp (ms): %f", time);
    }
    else {      
        ROS_INFO("Bad Checksum");
    }

    ROS_INFO("\n");
}

void imu::get_mag_accel_gyro() {        
    // Send the command to get instantaneous vectors
    write(MAG_ACCEL_GYRO_CMD, MAG_ACCEL_GYRO_SIZE);

    // Verify the checksum of the response
    if(verify_response(MAG_ACCEL_GYRO_SIZE)) {      
        // Get magnetometer vector (Gauss)
        double mag_x = ((int16_t)((response_buffer[1] << 8) | response_buffer[2])) * mag_gain_scale / 32768000.0;
        double mag_y = ((int16_t)((response_buffer[3] << 8) | response_buffer[4])) * mag_gain_scale / 32768000.0;
        double mag_z = ((int16_t)((response_buffer[5] << 8) | response_buffer[6])) * mag_gain_scale / 32768000.0;

        // Get accelerometer vector (G's)
        double accel_x = ((int16_t)((response_buffer[7] << 8) | response_buffer[8])) * accel_gain_scale / 32768000.0;
        double accel_y = ((int16_t)((response_buffer[9] << 8) | response_buffer[10])) * accel_gain_scale / 32768000.0;
        double accel_z = ((int16_t)((response_buffer[11] << 8) | response_buffer[12])) * accel_gain_scale / 32768000.0;

        // Get gyroscope vector (rad/sec)
        double gyro_x = ((int16_t)((response_buffer[13] << 8) | response_buffer[14])) * gyro_gain_scale / 32768000.0;
        double gyro_y = ((int16_t)((response_buffer[15] << 8) | response_buffer[16])) * gyro_gain_scale / 32768000.0;
        double gyro_z = ((int16_t)((response_buffer[17] << 8) | response_buffer[17])) * gyro_gain_scale / 32768000.0;

        // Get timestamp (ms)
        double time = ((int16_t)((response_buffer[19] << 8) | response_buffer[20])) * 6.5536;

        ROS_INFO("Magnetometer (Gauss): X:%f, Y:%f, Z:%f", mag_x, mag_y, mag_z);
        ROS_INFO("Accelerometer (G's): X:%f, Y:%f, Z:%f", accel_x, accel_y, accel_z);
        ROS_INFO("Gyroscope (rad/sec): X:%f, Y:%f, Z:%f", gyro_x, gyro_y, gyro_z);
        ROS_INFO("Timestamp (ms): %f", time);
    }
    else {      
        ROS_INFO("Bad Checksum");
    }

    ROS_INFO("\n");
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "imu");
    ros::NodeHandle nh("~");

    monitor::GetSerialDevice srv;
    nh.getParam("device_id", srv.request.device_id);

    /*
    ros::ServiceClient client = nh.serviceClient<monitor::GetSerialDevice>("/serial_manager/GetDevicePort");
    if (!client.call(srv)) {
        ROS_INFO("Couldn't get \"%s\" file descripter. Shutting down", srv.request.device_id.c_str());
        return 1;
    }
    */

    //ROS_INFO("Using imu on fd %s\n", srv.response.device_fd.c_str());

    /* Wait for callbacks */
    //ros::spin();
    imu dev("/dev/ttyS3");
    ros::Rate r(4);
    while(ros::ok()) {
        dev.get_mag_accel_gyro();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

