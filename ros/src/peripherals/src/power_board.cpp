#include <ros/ros.h>
#include <string>
#include <memory>
#include <serial/serial.h>

#include "monitor/GetSerialDevice.h"
#include "peripherals/powerboard.h"

using rosserv = ros::ServiceServer;
using powerboardInfo = peripherals::powerboard;

class power_board{
public:
    power_board(const std::string & port, int baud_rate = 9600, int timeout = 1000);
    ~power_board();
    void get_powerboard_data(powerboardInfo & msg);
private:
    std::unique_ptr<serial::Serial> connection = nullptr;
    std::string write(const std::string & out, bool ignore_response = true, std::string eol = "\n");
};

power_board::power_board(const std::string & port, int baud_rate, int timeout) {
    ROS_INFO("Connecting to power_board on port: %s", port.c_str());
    connection = std::unique_ptr<serial::Serial>(new serial::Serial(port, (u_int32_t) baud_rate, serial::Timeout::simpleTimeout(timeout)));
}

power_board::~power_board() {
    connection->close();
}

std::string power_board::write(const std::string & out, bool ignore_response, std::string eol)
{
    // Flush the output, then the input (order matters, to flush any unwanted responses)
    connection->flushOutput();
    connection->flushInput();

    // Send command 
    connection->write(out + eol);
    ROS_INFO("%s", out.c_str());

    // Used for commands where response does not matter (enabling, etc)
    if (ignore_response) {
        return "";
    }

    // Used for commands to read data
    return connection->readline(65536ul, eol);
}

void power_board::get_powerboard_data(powerboardInfo &msg) {
    // Get data from power board
    std::string currents = this->write("CRA", false);
    std::string voltages = this->write("VTA", false);
    std::string temperature = this->write("TMP", false);
    std::string humidity = this->write("HUM", false);
    std::string water = this->write("WTR", false);
    std::string pressure_internal = this->write("PIN", false);
    std::string pressure_external = this->write("PEX", false);

    // Populate message with current data
    if(currents.length() >= 12) {       
        msg.current_battery_1 = (currents[0] << 16) | (currents[1] << 8) | (currents[2]);
        msg.current_battery_2 = (currents[3] << 16) | (currents[4] << 8) | (currents[5]);
        msg.current_motors = (currents[6] << 16) | (currents[7] << 8) | (currents[8]);
        msg.current_system = (currents[9] << 16) | (currents[10] << 8) | (currents[11]);
    }
    else {      
        ROS_INFO("Current data is invalid. Data:%s\n", currents.c_str());
    }

    // Populate message with voltage data
    if(voltages.length() >= 4) {        
        msg.voltage_battery_1 = (voltages[0] << 8) | (voltages[1]);
        msg.voltage_battery_2 = (voltages[2] << 8) | (voltages[3]);
    }
    else {      
        ROS_INFO("Voltage data is invalid. Data:%s\n", voltages.c_str());
    }

    // Populate message with temperature data
    if(temperature.length() >= 2) {     
        msg.temperature = (temperature[0] << 8) | (temperature[1]);
    }
    else {      
        ROS_INFO("Temperature data is invalid. Data:%s\n", temperature.c_str());
    }

    // Populate message with humidity data
    if(humidity.length() >= 2) {     
        msg.humidity = (humidity[0] << 8) | (humidity[1]);
    }
    else {      
        ROS_INFO("Humidity data is invalid. Data:%s\n", humidity.c_str());
    }

    // Populate message with water sensor data
    if(water.length() >= 2) {     
        msg.water_sensor = (water[0] << 8) | (water[1]);
    }
    else {      
        ROS_INFO("Water data is invalid. Data:%s\n", water.c_str());
    }

    // Populate message with main housing pressure data
    if(pressure_internal.length() >= 2) {     
        msg.internal_pressure = (pressure_internal[0] << 8) | (pressure_internal[1]);
    }
    else {      
        ROS_INFO("Internal housing pressure data is invalid. Data:%s\n", pressure_internal.c_str());
    }

    // Populate message with external water pressure data
    if(pressure_external.length() >= 2) {     
        msg.external_pressure = (pressure_external[0] << 8) | (pressure_external[1]);
    }
    else {      
        ROS_INFO("External water pressure data is invalid. Data:%s\n", pressure_external.c_str());
    }
} 

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "power_board");
    ros::NodeHandle nh("~");

    monitor::GetSerialDevice srv;
    nh.getParam("device_id", srv.request.device_id);

    ros::ServiceClient client = nh.serviceClient<monitor::GetSerialDevice>("/serial_manager/GetDevicePort");
    if (!client.call(srv)) {
        ROS_INFO("Couldn't get \"%s\" file descripter. Shutting down", srv.request.device_id.c_str());
        return 1;
    }

    ROS_INFO("Using Power Board on fd %s\n", srv.response.device_fd.c_str());

    ros::Publisher pub = nh.advertise<peripherals::powerboard>("power_board_data", 10);

    // Main loop
    ros::Rate r(1);
    power_board device(srv.response.device_fd);
    while(ros::ok()) {
        // Publish message to topic 
        peripherals::powerboard msg;
        device.get_powerboard_data(msg);
        pub.publish(msg);

        // End of loop maintenance
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

