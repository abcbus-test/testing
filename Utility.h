/*
 * Utility.h
 *
 *  Created on: Mar 2, 2015
 *      Author: imtiazahmed
 */

#ifndef UTILITY_H_
#define UTILITY_H_

#include <fstream>
#include <string>

using namespace std;

#define DEVICE 1
#define DEBUG  3

const int UPDATE_APPLICATION_DATASIZE= 500;//72;
const int UPDATA_APPLICATION_FRAMESIZE=50;
const int CRCSIZE =2;

const int MAX_MESSAGE_LENGTH = 1100;//315;

struct s_timeout_info {
                 bool timeout_status;
                 unsigned char timeout_packet[50];
		 int timeout_len;
                 bool timeout_cycle;
              };


class Utility {
public:
	Utility();
	virtual ~Utility();


	/**
	 * Return mac address of the device
	 */
	unsigned char* getMACAddress();


	/**
	 * Load the configuration from config file
	 */
	void loadConfiguration();


	/**
	 * read the server address from file and return
	 */
	string getServerAddress() ;


	/**
	 * returns the server port
	 */
	int getServerPort();

	/**
	 * returns the devices ids in string
	 */
	string getDevicesIDs();

	/**
	 * Load the device numbers to given array
	 */
	bool loadDevices(int* devices_array);


	/**
	 * Return the count of devices
	 */
	int getDevicesCount();

	/**
	 * Load integer to a given location in char array
	 */
	void loadIntInArray(int value, unsigned char* array, int start_position);


	/**
	 * Zero out the given array
	 */
	bool clearBytes(unsigned char* array, int start, int end);

	/**
	 * Print out the given array
	 */
	void printBytes(unsigned char* array, int start, int end);


	/**
	 * Get Int from byte array
	 */
	int getIntFromByte(unsigned char* array,
			int start_position);
			
	int findpattren(unsigned char *str, unsigned char *find, int strln,int findln,int str_p);

	void loadLongInArray(unsigned long int longInt, unsigned char* byteArray, int start_position);
	
	long int getLongIntFromByte(unsigned char* byteArray, int start_position);

	
        static bool packet_timeout[10];
        static unsigned char timeout_packet[10][100];

private:

	ifstream m_config_file;

	/**
	 * Server address i.e
	 * datacenter.lodam.com
	 */
	string m_server_address;

	/**
	 * Port number of server. i.e
	 * 5000
	 */
	int m_server_port;

	/**
	 * Indicator if configuration file was read successfully
	 */
	bool m_configuration_read;

	/**
	 * Name of configuration file
	 */
	string m_config_file_name;

	/**
	 * Comma separated device numbers to be read from the configuration file
	 */
	string m_device_numbers;

	/**
	 * Count of devices attached to this moxa box
	 */
	int m_devices_count;

	/**
	 * MAC address of this moxa box
	 */
	unsigned char mac_address[6];


};

#endif /* UTILITY_H_ */
