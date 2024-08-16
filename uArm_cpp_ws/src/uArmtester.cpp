#include <string>
#include <iostream>
#include <cstdio>
#include <chrono>         // For sleep
#include <thread>  

#include "uarm/uarm.h"


using namespace std;

void enumerate_ports()
{

	vector<uarm::PortInfo> devices_found = uarm::list_ports();

	vector<uarm::PortInfo>::iterator iter = devices_found.begin();

	while (iter != devices_found.end())
	{
		uarm::PortInfo device = *iter++;
		cout << "port: " << device.port << ", desc: " << device.description << ", hardware_id: " << device.hardware_id << endl;
	}
}

// an async callback function
void get_callback(int value){
    cout << "callbacks: " << value << endl;
}


void get_info(uarm::Swift* swift){
	// swift->get_power_status(false, 2, get_callback);
	int value = swift->get_power_status();
	cout << "get_power_status(sync): " << value << endl;
}


int main(int argc, char** argv){

    uarm::Swift* swift = new uarm::Swift("/dev/ttyAMC0", 115200);
    
    if(!swift->connected){
        cout << "UARM Swift PRO not connected please try again.." << endl;
        delete swift;
        return -1;
    }

    get_info(swift);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
    swift->disconnect();
    delete swift;

    return 0;
}