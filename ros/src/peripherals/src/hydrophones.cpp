#include <ros/ros.h>
#include <vector>
#include <string>
#include <serial/serial.h>

#include "monitor/GetSerialDevice.h"

#define NUM_HYDROPHONES (4)
#define PKT_HEADER_SIZE (12)


class hydrophones
{       
public:
    hydrophones(const std::string port, int baud_rate = 115200, int timeout = 1000);
    ~hydrophones();
    std::string write(const std::string out, bool ignore_response = true, const std::string eol = "\n");
    void acquire_hydro_data(std::vector<uint16_t> hydro_data[NUM_HYDROPHONES]);
private:
    uint32_t stm32f4_crc32(uint8_t* data, size_t data_len, uint32_t crc = 0xFFFFFFFF);
    ros::NodeHandle nh;
    std::unique_ptr<serial::Serial> connection = nullptr;
};

hydrophones::hydrophones(const std::string port, int baud_rate, int timeout) :
    nh(ros::NodeHandle("~"))
{       
    ROS_INFO("Connecting to hydrophones on port %s", port.c_str());
    connection = std::unique_ptr<serial::Serial>(new serial::Serial(port, (u_int32_t)baud_rate, serial::Timeout::simpleTimeout(timeout)));

    /* Get the packet and data sizes from launch file */
    int packet_size, data_size;
    nh.getParam("packet_size", packet_size);
    nh.getParam("data_size", data_size);

    /* Set the packet size */
    std::string out = "PS";
    out += (packet_size < 10000 ? "0" : "");
    out += (packet_size < 1000 ? "0" : "");
    out += (packet_size < 100 ? "0" : "");
    out += (packet_size < 10 ? "0" : "");
    out += std::to_string(packet_size);
    std::string p_size = write(out, false);
   
    /* Set the data size */
    out = "DS";
    out += (data_size < 10000 ? "0" : "");
    out += (data_size < 1000 ? "0" : "");
    out += (data_size < 100 ? "0" : "");
    out += (data_size < 10 ? "0" : "");
    out += std::to_string(data_size);
    std::string d_size = write(out, false);

    uint16_t packet_size_actual;
    if(p_size.length() >= 4)
    {   
        packet_size_actual = (p_size[1] << 8) | p_size[0];
        ROS_ERROR("Packet Size is set to %u", packet_size_actual);
    }
   
    uint16_t data_size_actual;
    if(d_size.length() >= 4)
    {   
        data_size_actual = (d_size[1] << 8) | d_size[0];
        ROS_ERROR("Data size is set to %u", data_size_actual);
    }
}

hydrophones::~hydrophones()
{       
    connection->close();
}

std::string hydrophones::write(const std::string out, bool ignore_response, const std::string eol)
{   
    connection->flush();
    ROS_ERROR("%s", out.c_str());
    connection->write(out + eol);
    connection->flushOutput();

    if(ignore_response) 
    {   
        return "";
    }

    return connection->readline(65536ul, eol);
}

void hydrophones::acquire_hydro_data(std::vector<uint16_t> hydro_data[NUM_HYDROPHONES])
{   
    uint8_t header[PKT_HEADER_SIZE];
    write("ADCDR");

    uint32_t data_index = 0;
    uint8_t packet_idx = 0;
    uint8_t packet_count = 0;
    do
    {  
        // Get the packet header and relevant information
        uint8_t lines;
        if((lines = connection->read(header, PKT_HEADER_SIZE)) != PKT_HEADER_SIZE)
        {       
            ROS_ERROR("Did not read enough lines! Read only %u lines.", lines);
        }
        uint32_t crc = (header[3] << 24) | (header[2] << 16) | (header[1] << 8) | header[0];
        packet_count = header[6];
        packet_idx = header[7];
        uint16_t packet_size = (header[9] << 8) | header[8];
        uint16_t total_size = (header[11] << 8) | header[10];

        ROS_ERROR("CRC: %u, Packet Index: %u, Packet Count: %u, Packet Size: %u, Total Size: %u", crc, packet_idx, packet_count, packet_size, total_size);

        // Use header to determine how much data to read
        uint8_t* packet_data = new uint8_t[packet_size];
        connection->read(packet_data, packet_size);

        // Check to see if there were any communication errors
        uint32_t host_crc = stm32f4_crc32(packet_data, packet_size);
        if(host_crc != crc)
        {       
            ROS_ERROR("Communication error. Executing packet resend.");
            packet_idx = 0; // Do this just in case this is the last packet (get through the while loop)
            write("RETRY"); // Send the last packet again
            delete[] packet_data;
            continue;
        }
        
        // Signal the beginning of the next packet transfer
        write("NEXT");

        // Append data to output while we wait
        for(int i = 0; i < (packet_size/2); i++, data_index++)
        {   
            hydro_data[data_index % NUM_HYDROPHONES].push_back((packet_data[(i*2)+1] << 8) | packet_data[i*2]); 
        }

        delete[] packet_data;

    } while(packet_idx < (packet_count - 1));

    ROS_ERROR("DONE");

    connection->flush(); // Sent an extra "NEXT"
}

uint32_t hydrophones::stm32f4_crc32(uint8_t* data, size_t data_len, uint32_t crc)
{
    // Make sure data length is a multiple of 4
    if(data_len % 4)
    {   
        ROS_ERROR("Invalid data for STM32F4 CRC. Data length must be a multiple of 4.");
        return 0;
    }

    // Compute the CRC 32 bits at a time
    for(int i = 0; i < (data_len/4); i++)
    {  
        uint32_t uint32_datum = (data[(i*4)+3] << 24) | (data[(i*4)+2] << 16) | (data[(i*4)+1] << 8) | data[(i*4)];
        //uint32_t uint32_datum = (data[i*4] << 24) | (data[(i*4)+1] << 16) | (data[(i*4)+2] << 8) | data[(i*4)+3];
        crc = crc ^ uint32_datum;
        for(int j = 0; j < 32; j++)
        {      
            if(crc & 0x80000000)
            {      
                crc = (crc << 1) ^ 0x04C11DB7;
            }
            else
            {   
                crc = crc << 1;
            }
        }
    }

    return crc;
}

int main(int argc, char** argv)
{       
    ros::init(argc, argv, "hydrophones");
    ros::NodeHandle nh("~");

    monitor::GetSerialDevice srv;
    nh.getParam("device_id", srv.request.device_id);

    /*ros::ServiceClient client = nh.serviceClient<monitor::GetSerialDevice>("/serial_manager/GetDevicePort");
    if(!client.call(srv)) {     
        ROS_INFO("Couldn't get \"%s\" file descriptor. Shutting down", srv.request.device_id.c_str());
        return 1;
    }

    ROS_INFO("Using Hydrophones on fd %s\n", srv.response.device_fd.c_str());*/

    //hydrophones device(srv.response.device_fd.c_str());
    hydrophones device("/dev/ttyS3");

    std::vector<uint16_t> data[4];
    device.acquire_hydro_data(data);
   
    ros::spin();

    return 0;
}
