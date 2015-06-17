/*
 * LodamModbusProject.cpp
 *
 *  Created on: Feb 14, 2015
 *      Author: imtiazahmed
 */

#include <iostream>
#include "ModbusController.h"
#include "ServerController.h"
#include "Utility.h"
#include <stdlib.h>
#include <unistd.h>
#include <syslog.h>

#include<sys/socket.h>    //socket
#include<arpa/inet.h> //inet_addr
#include <sys/signal.h>

using namespace std;

int device_ids[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
int total_devices_count = 8;

int register_ids[] = { 203, 221, 400, 401, 402, 403, 404, 405, 406, 407, 408,
	409, 1000, 1001, 1002, 1003, 1004 };

int total_register_count = 17;
string serverAddress = "";
int serverPort = 0;
int modbus_interval = 0;
bool show_output = true;
unsigned long long ctr = 0;

Utility utility;

unsigned char* mac_address;




static struct s_timeout_info timeout_info[10];

ModbusController modbusController;
ServerController serverController;

/**
 * Loop and read status from devices using Modbus protocol
 * and send data to server
 */
int work() {


	try {

		//		ModbusController modbusController;
		//		ServerController serverController;

		// 5 for command number and length + (  17 registers * 2 (reg number ) * 2 (reg value))
		unsigned char devices_data[total_devices_count][5 + 68];

		for (int i = 0; i < total_devices_count; i++) {
			utility.clearBytes(devices_data[i], 0, 5 + 68);
		}

		//unsigned char m_communication_message[256];

		int total_bytes = 0;

		int change_counter;

		int timer = 0;

		serverController.setIDForCommunication(mac_address);
		if (!modbusController.establishConnection()) {
			printf(" Can not connect to USB \n");
			return 0;
		}

		bool connected = false;
		int read_length = 0;
		unsigned char* data_from_server;
		unsigned char* data_from_modbus;
		int reply_length;

		do
		{

			show_output = false;

			if (++ctr % 500 == 0) {

				show_output = true;
				cout << "\nREAD COUNTER  = " << ctr;

				if(ctr > 100000) {ctr = 0;}

			}
			connected=serverController.isConnected();

			//if not connected, try to connect to server
			if (connected!=true) {
				connected = serverController.connectWithServer(serverAddress,
						serverPort);

				if (connected) {
					//send welcome message, the server will either return success
					//or will return with the values to be written to the registers
					serverController.sendWelcomeMessage();

					//Check the last reply returned by server after welcome message
					data_from_server = serverController.getLastReplyFromServer(
							read_length);

					//now give the data to modbus controller, it will parse and write on registers
					//of given device, if required.
					//if returned values is true, report it to server.
					reply_length = modbusController.parseCommands(
							data_from_server, read_length,mac_address,&timeout_info[10]);

					if (reply_length > 0) {
						//if there is a data to be written, write it to server
						//the modbusController.parseCommand would have updated the array (as they are passed as ref)
						//so just send the number of bytes, from start, to write to server
						//serverController.sendCommandReply(reply_length);
					}

				} else {
					//connection with server failed, exit the program
					//watch dog will start it again
					cout << "CONNECTION WITH SERVER FAILED...EXITING" << endl;
					return 0;
				}
			}

			//if connected, loop through the devices and send the data to server
			if (connected) {
				//timer++;

				//check if value of any register is changed - if so send the values of all registers for that device only

				for (int device_ctr = 0; device_ctr < total_devices_count;
						device_ctr++) {

					//if device id on given index is 0, move to next one
					if (device_ids[device_ctr] == 0) {
						continue;
					}

					timer = 0;

					do{

						change_counter = 0;

						if (show_output) {
							cout << endl
								<< "==============================================";
							cout << endl << "Device_id = " << device_ids[device_ctr]
								<< endl;
							cout << endl;
						}

						total_bytes = modbusController.readRegistersOfADevice(
								device_ids[device_ctr], register_ids,
								devices_data[device_ctr], total_register_count,
								change_counter, show_output);

						if (show_output) {
							cout << endl << "-------------------------------------"
								<< endl;
							cout << endl << "Change_index = " << change_counter
								<< " Read Length = " << total_bytes << endl;
						}

						//value of none of the sensor changed, must be due to some error
						if (change_counter == 0) {
							timer++;
							total_bytes=0;
						}

						//if no values were found changed for 5 times, try to reconnect the port
						if (timer >= 5) {

							//cout << "Exiting inner loop.. " << endl;
							timer = 0;
							break;
							//modbusController.releaseConnection();
							//sleep(3);
							//modbusController.establishConnection();

							//read data again
							//						//return 1;
						}
					}while(change_counter == 0);

					if (/*total_bytes > 0 && change_counter > 0 yas error*/1) {
						if (show_output) {
							cout << "\nSending data to server ";
						}

						//more than one register changed, send the data to server
						bool status = serverController.sendDevicesStatus(devices_data[device_ctr], total_bytes);

						data_from_server = serverController.getLastReplyFromServer(read_length);

						//now give the data to modbus controller, it will parse and write on registers
						//of given device, if required.
						//if returned values is true, report it to server.
						reply_length = modbusController.parseCommands(data_from_server, read_length,mac_address,timeout_info);

						data_from_modbus= modbusController.getLastReply(reply_length);

						if (reply_length > 0) {
							//if there is a data to be written, write it to server
							//the modbusController.parseCommand would have updated the array (as they are passed as ref)
							//so just send the number of bytes, from start, to write to server
							//							data_from_modbus= modbusController.getLastReply(reply_length);
							//printf("yas %d: Sending replay func=%s, file=%s\n",__LINE__,__FUNCTION__,__FILE__);
							//utility.printBytes(data_from_modbus,0,reply_length);
							serverController.sendCommandReply(data_from_modbus,reply_length);
						}

						if (show_output) {
							cout << endl << "Done with status " << status
								<< endl;
						}

						//if there was a problem sending data to server try to reconnect
						if (!status) {
							//							printf("yas %d: func=%s, file=%s\n",__LINE__,__FUNCTION__,__FILE__);
							//							serverController.disconnect();
							//							connected = false;
							//							break;
						}
					}

					if (show_output) {
						cout << "=============================================="
							<< endl;
					}

				}

				//				serverController.disconnect();	yas update
				//				connected = false;	yas update
			}

			sleep(modbus_interval);
			//			usleep(35 * 500);
			//			sleep(2);
		}

		while (true);
		modbusController.releaseConnection();

	} catch (exception &e) {
		cout << "Standard exception: " << e.what() << endl;
	} catch (...) {

	}

	return 0;

}

void startWatchdog() {

	FILE *fp;
	char path[1035];
	const char *watchdogApp = "Watchdog";

	bool bFound = false;

	fp = popen("/bin/ps | grep Watchdog", "r");
	if (fp == NULL) {
		printf("Failed to run command\n");
		return;
	}

	/* Read the output a line at a time - output it. */
	while (fgets(path, sizeof(path) - 1, fp) != NULL) {
		if (strstr(path, watchdogApp) != NULL
				&& strstr(path, "grep Watchdog ") == NULL) {
			bFound = true;
			break;
		}
	}
	/* close */
	pclose(fp);

	if (bFound == false) {
		printf("\n\rStarting Watchdog..\n\r");
		system("/home/Watchdog &");
	}

	return;

}

void *Timer(void*)
{

	try {
		int timeout = 0;
		int timeout_limit=0;
		bool connected;

		while(1){
			connected=serverController.isConnected();
			//if not connected, try to connect to server
			if (connected==true) {
				if(timeout_info[0].timeout_status==true){
					if(timeout_info[0].timeout_cycle==true)
						timeout=0;
					else{
						timeout++;
						timeout_limit++;
					}

					if(timeout>30){
						timeout=0;
						utility.printBytes(timeout_info[0].timeout_packet,0,timeout_info[0].timeout_len);
						if(timeout_info[0].timeout_len>0)
							serverController.sendCommandReply(timeout_info[0].timeout_packet,timeout_info[0].timeout_len);
					}

					if(timeout_limit>300)
						timeout_info[0].timeout_status=false;

					timeout_info[0].timeout_cycle=false;
				}
				//send welcome message, the server will either return success
				//or will return with the values to be written to the registers
			}

			sleep(1);
		}
	} catch (exception &e) {
		cout << "Standard exception: " << e.what() << endl;
	} catch (...) {

	}


	//	pthread_exit(NULL);
}
int main(int argc, char **argv) {

	pthread_t timer_thread;
	cout << endl;
	cout << endl;
	cout << "****************************************" << endl;
	cout << "Moxa Client Application " << endl;
	cout << "Version 1.1.1 " << endl;
	cout << "****************************************" << endl;
	cout << endl;

	cout << "Moxa client starting....." << endl;

	//Wait for a while to ensure the connections are up before start working
#if DEVICE == 1
#endif

	startWatchdog();

	signal(SIGPIPE, SIG_IGN);
	utility.loadConfiguration();
	utility.loadDevices(device_ids);

	mac_address = utility.getMACAddress();

	if (mac_address[0] == 0 && mac_address[1] == 0) {
		cout << "NO MAC ADDRESS FOUND.. EXITING " << endl;
		return -1;
	}

	total_devices_count = utility.getDevicesCount();

	serverAddress = utility.getServerAddress();
	serverPort = utility.getServerPort();

	cout << endl;
	cout << "Server Address =  " << serverAddress << endl;
	cout << "Server Port    =  " << serverPort << endl;

	printf("MAC Address    =  %02X:%02X:%02X:%02X:%02X:%02X\n", mac_address[0],
			mac_address[1], mac_address[2], mac_address[3], mac_address[4],
			mac_address[5]);

	cout << "Local Devices  =  " << utility.getDevicesIDs() << endl;


	memset(timeout_info,0,sizeof(timeout_info[10]));
	pthread_create(&timer_thread, NULL, Timer, NULL);

	do {
		if (work() == 0) {
			break;
		}
		cout << endl << " " << endl;
	} while (true);

	cout << endl << " Done..";

	return 0;
}

/*
 * //		int length = 31;
//		unsigned char status_data [] = {0x00, 0x1D, 0x02, 0x1F, 0x03, 0x00, 0xCB, 0x09, 0x2F,
//				0x00, 0xDD, 0x00, 0x00, 0x03, 0xEB, 0x00, 0x00, 0x1E, 0x03, 0x00, 0xCB, 0x09, 0x2A,
//				0x00, 0xDD, 0x00, 0x10, 0x03, 0xEB, 0x00, 0x01};
 *
 * */
