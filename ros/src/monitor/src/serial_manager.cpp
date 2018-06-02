#include <ros/ros.h>
#include <serial/serial.h>
#include <vector>
#include <map>
#include <algorithm>
#include <string>
#include <iostream>
#include <stdexcept>
#include <cstdint>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "monitor/SerialDevice.h"
#include "monitor/GetSerialDevice.h"
#include "monitor/GetSerialDevices.h"
#include "devices.hpp"

device_manager::device_manager(const std::vector<device_property> & properties) {
    std::vector<serial::PortInfo> ports = serial::list_ports();
    for (auto property = properties.begin(); property != properties.end(); ++property) {
        for(auto port = ports.begin(); port != ports.end();) {
            
            std::string prefix("/dev/ttyUSB");
            if(!std::equal(prefix.begin(), prefix.end(), port->port.begin())) {
                port = ports.erase(port); //iterator takes next position in list
                continue;
            }
            
            serial::Serial connection(port->port, (uint32_t) property->baud , property->timeout);
            
            bool device_found = false;
            if (property->convert_to_bytes) {
                // Get response size as defined in JSON
                uint8_t send_data = std::stoi(property->ack_message);
                connection.write(&send_data, 1);
                uint8_t * response_array = new uint8_t[property->size_of_response];
                connection.read(response_array, property->size_of_response);

                // put bytes together to get our expected resonse 
                uint64_t response = 0;
                for (int i = 0 ; i < property->size_of_response; ++i) {
                    int shift_by = 8 * (property->size_of_response-i-1);
                    response |= ((uint64_t) response_array[i]) << shift_by; // Assuming Big endian
                }
                uint64_t expected_response = std::stoi(property->ack_response);
                device_found = expected_response == response;
                delete [] response_array; 
            } else {
                connection.write(property->ack_message);
                std::string response = connection.readline(65536ul, "\n");
                device_found = response == property->ack_response;
            }

            connection.close();
            
            if (device_found) {
                monitor::SerialDevice dev;
                dev.name = property->name;
                dev.port = port->port;
                devices.insert(MapPair(dev.name, dev));
                port = ports.erase(port);
                ROS_INFO("Found device \"%s\" on %s\n", dev.name.c_str(), dev.port.c_str());
                break;
            }
            
            ++port; 
        }

        // Check if we found the device i.e is it in the map
        SerialDeviceMap::iterator it = devices.find(property->name);
        if (it == devices.end()) {
            throw DeviceNotFoundException(property->name);
        }
        

    }
}

bool device_manager::get_device_by_name(GetSerialReq &req, GetSerialRes &res) {

    SerialDeviceMap::iterator it = devices.find(req.device_id);
    if (it == devices.end()) {
        ROS_INFO("No Device with name \"%s\"\n", req.device_id.c_str());
        return false;
    }

    ROS_INFO("found Device with name \"%s\"\n", req.device_id.c_str());
    res.device_fd = it->second.port;
    return true;
}

bool device_manager::get_all_devices(GetSerialsReq &req, GetSerialsRes &res) {
    //std::vector<monitor::SerialDevice> v;
    std::transform(devices.begin(), devices.end(), std::back_inserter(res.devices), 
        [](const MapPair &p) { 
            ROS_INFO(" %s : %s \n", p.second.name.c_str(), p.second.port.c_str());
            return p.second; });
    return true;
}


void parse_json(std::vector<device_property> & json_properties, std::string json_file_location) {
    ptree pt;
    boost::property_tree::read_json(json_file_location, pt);
    
    for (ptree::const_iterator it = pt.begin(); it != pt.end(); ++it) {
        int baud, timeout;
        std::string msg, rsp;
        bool convert;
        size_t size_of_response = 0; //default to 0
        try {
            baud = it->second.get<int>("baud");
            msg = it->second.get<std::string>("ack_message");
            rsp = it->second.get<std::string>("ack_response");
            timeout = it->second.get<int>("timeout");
            convert = it->second.get<bool>("convert_to_bytes");
            if (convert) {
                size_of_response = it->second.get<size_t>("size_of_response");
            }
        } catch (...) {
            throw std::runtime_error("Failed to parse " + it->first);
        }
        
        device_property new_dev(it->first, msg, rsp, baud, timeout, convert, size_of_response);
        json_properties.push_back(new_dev);
    }
}

                                
int main(int argc, char ** argv) {
    // Setup ROS stuff
    ros::init(argc, argv, "device_manager");
    ros::NodeHandle nh("~");

    // Create ptree structure from JSON file
    std::string json_file_location;
    nh.getParam("devices_json_location", json_file_location);
    std::vector<device_property> json_properties;
    parse_json(json_properties, json_file_location);

    device_manager m(json_properties);
    ros::ServiceServer getDevice = nh.advertiseService("GetDevicePort", &device_manager::get_device_by_name, &m);
    ros::ServiceServer getDevices = nh.advertiseService("GetAllDevices", &device_manager::get_all_devices, &m);
    ros::spin();
    
    return 0;
}