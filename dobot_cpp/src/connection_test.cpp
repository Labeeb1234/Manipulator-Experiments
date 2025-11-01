#include <iostream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>

using namespace std;

// ensure DobotMessages don't have any padding (added while compile to prevent wrong messages being sent to the hardware)
#pragma pack(push,1)
struct DobotMessage{
    uint8_t header[2] = {0xAA, 0xAA}; // array
    uint8_t len = 0x00;
    uint8_t id = 0;
    uint8_t ctrl = 0x00;
    uint8_t params[256] = {0}; // max parameter size 256 8bit resolution controllers ?? // array
    uint8_t check_sum = 0x00;
};
#pragma pack(pop)


uint8_t calculateCheckSum(const DobotMessage& msg){
    uint8_t sum = msg.id+msg.ctrl;
    // sum += msg.id;
    // sum += msg.ctrl;
    for(int i=0; i<msg.len-2; i++){
        sum += msg.params[i];
    }
    return (uint8_t) (256-(sum%256)); // 2s complement of 8 bits of data
}


int main(int argc, char** argv){

    // dobot test connection (get device id)
    // creating a ping message 
    DobotMessage ping_msg;
    uint8_t param_idx = 0; // goes to the end of len field value for specific tasks/cmd
    ping_msg.len = 2+param_idx;
    ping_msg.id = 1;
    ping_msg.ctrl = 0x01;
    ping_msg.check_sum = calculateCheckSum(ping_msg);


    // serializing the ping msg to send to dobot
    vector<uint8_t> send_buffer; // push_bask may have 0s at the start right (Jargon Issue ??)
    send_buffer.push_back(ping_msg.header[0]);
    send_buffer.push_back(ping_msg.header[1]);
    send_buffer.push_back(ping_msg.len);
    send_buffer.push_back(ping_msg.id);
    send_buffer.push_back(ping_msg.ctrl); // params portion is empty for the get device info cmd
    send_buffer.push_back(ping_msg.check_sum);

    printf("Ping packet to be sent (in Hex): ");
    for(uint8_t byte: send_buffer){
        printf("%02X", byte);
    }
    printf("\n");


    // send and recieve response
    // serial connection setup and communication
    try{
        string port_id = "/dev/ttyUSB0";
        // creating io service using boost asio
        boost::asio::io_context io;
        boost::asio::serial_port serial(io);
        serial.open(port_id);

        // serial coms settings
        serial.set_option(boost::asio::serial_port_base::baud_rate(115200));
        serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial.set_option(boost::asio::serial_port_base::character_size(8));
        serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

        printf("Successfully Opened Port: %s\n", port_id.c_str());
        printf("Setting communication baudrate to: %d\n", 115200);

        // send cmd msg packet
        boost::asio::write(serial, boost::asio::buffer(send_buffer.data(), send_buffer.size()));
        printf("Ping message s0ent, Awaiting response....\n");
        // reading recieved data (for now keeping this)
        // --- Robustly Read Response ---
        // A full response message is at least 6 bytes (Header*2, Len, ID, Ctrl, Checksum)
        // Max total message size (2 Header + 1 Len + 1 ID + 1 Ctrl + 256 Params + 1 Checksum) = 262 bytes
        vector<uint8_t> read_buffer(270); // Buffer large enough for max message
        // Read the header (2 bytes)
        size_t bytes_read = boost::asio::read(serial, boost::asio::buffer(read_buffer.data(), 2), boost::asio::transfer_exactly(2));
        if (bytes_read != 2 || read_buffer[0] != 0xAA || read_buffer[1] != 0xAA) {
            cerr << "Invalid header received" << endl;
            serial.close();
            return 1;
        }
        // Read the length (1 byte)
        bytes_read = boost::asio::read(serial, boost::asio::buffer(read_buffer.data() + 2, 1), boost::asio::transfer_exactly(1));
        if (bytes_read != 1) {
            cerr << "Failed to read length" << endl;
            serial.close();
            return 1;
        }

        uint8_t msg_len = read_buffer[2]; // This is the length of (id + ctrl + params)
        // Read the payload (id + ctrl + params) and checksum
        // We need to read 'msg_len' bytes (payload) + 1 byte (checksum)
        size_t bytes_to_read = (size_t)msg_len + 1; 
        bytes_read = boost::asio::read(serial, boost::asio::buffer(read_buffer.data() + 3, bytes_to_read), boost::asio::transfer_exactly(bytes_to_read));
        if (bytes_read != bytes_to_read) {
            cerr << "Failed to read full payload" << endl;
            serial.close();
            return 1;
        }

        // Total bytes in buffer: 2 (header) + 1 (len) + msg_len (payload) + 1 (checksum)
        size_t total_msg_len = 3 + msg_len + 1;
        printf("Received %zu (payload)bytes (in Hex):\n", total_msg_len);
        for(size_t i = 0; i < total_msg_len; ++i) {
            printf("%02X ", read_buffer[i]);
        }
        printf("\n");
        
        // --- Process the Response ---
        // TODO: Verify checksum of received message
        // Check response ID (at index 3)
        uint8_t response_id = read_buffer[3];
        if (response_id == ping_msg.id) { // Should be 0
            cout << "Connection successful! Received response for GetDeviceId (CMD-ID=0)." << endl;
            // For ID=0, params start at index 5 and are 24 bytes (Device SN)
            // msg_len = 1 (id) + 1 (ctrl) + 24 (params) = 26      
            string device_sn;
            // Assign from the params section (starts at index 5, length 24)
            device_sn.assign(read_buffer.begin() + 5, read_buffer.begin() + 5 + 24);
            // Trim trailing null characters from the C-style string
            device_sn.erase(device_sn.find_last_not_of('\0') + 1);
            cout << "Device Serial Number: " << device_sn << endl;

        }
        else{
            cerr << "Received response for unexpected ID: " << (int)response_id << endl;
        }
        serial.close();

    }catch(const boost::system::system_error& e){
        cerr << "Boost.Asio Error: " << e.what() << endl;
        cerr << "Please double check port id" << endl;
    }catch(const exception& e){
        cerr << "Error: " << e.what() << endl;
    }

    return 0;
}
