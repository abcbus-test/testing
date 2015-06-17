/*
 * Utility.cpp
 *
 *  Created on: Mar 2, 2015
 *      Author: imtiazahmed
 */

#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <netinet/in.h>
#include <string.h>
#include <sstream>
#include <iostream>
#include <stdio.h>
#include "Utility.h"
#include <stdlib.h>     /* strtoul */

Utility::Utility() {

	m_server_port = 5000;
#if DEVICE == 1
	m_server_address = "datacenter.lodam.com"; //"datacenter.lodam.com";  "192.168.40.78"   "lodamapp-dev-env.elasticbeanstalk.com"
#else
	m_server_address = "192.168.40.78";
#endif


	m_configuration_read = false;
	m_config_file_name = "config.txt";

#if DEVICE == 1
	m_device_numbers = "28,29,30";
#else
	m_device_numbers = "31";
#endif

	m_devices_count = 0;

}

Utility::~Utility() {

}

void Utility::loadConfiguration() {


	FILE *fp;
	char buf[100];

	string str;


	fp = fopen(m_config_file_name.c_str(), "r");

	   if( fp == NULL )
	   {
	      perror("Error while opening the file.\n");
	      return;
	   }

	   printf("Configuration file found. Reading contents form %s  :\n", m_config_file_name.c_str());

	   while (fgets(buf,1000, fp) != NULL){

			string line (buf);

			if (line.find("server_address=") != string::npos) {
				int start = line.find("=") + 1;
				int end = line.size()  - start - 1;
				str = line.substr(start, end);
				m_server_address = str.substr();
				cout << m_server_address << endl;
			}
			else if (line.find("server_port=") != string::npos) {
				str = line.substr(line.find("=") + 1);
				m_server_port = atoi(str.c_str());
//				cout << m_server_port << endl;
			}
			else if (line.find("device_numbers=") != string::npos) {
				str = line.substr(line.find("=") + 1);
				m_device_numbers = str;
//				cout << m_device_numbers << endl;
			}
	   }


	   fclose(fp);




/**
 * For some reasons following code failed on micro Linux
 */
//	std::ifstream config_file(m_config_file_name.c_str());
//
//
//	if (config_file.is_open()) {
//
//		cout << m_config_file_name <<"FILE FOUND " << endl;
//
//		while (std::getline(config_file, line)) {
//
//			cout << line << endl;
//
//			if (line.find("server_address=") != string::npos) {
//				str = line.substr(line.find("=") + 1);
//				m_server_address = str;
//				cout << m_server_address << endl;
//			}
//			else if (line.find("server_port=") != string::npos) {
//				str = line.substr(line.find("=") + 1);
//				m_server_port = atoi(str.c_str());
//				cout << m_server_port << endl;
//			}
//			else if (line.find("device_numbers=") != string::npos) {
//				str = line.substr(line.find("=") + 1);
//				m_device_numbers = str;
//				cout << m_device_numbers << endl;
//			}
//
//		}
//		config_file.close();
//	} else {
//		cout << "Config.txt file not found using defaults " << endl;
//	}

	m_configuration_read = true;



}

/**
 * Get the address of server to connect to.
 */
string Utility::getServerAddress() {
	return m_server_address;
}

/**
 * Get the address of server to connect to.
 */
int Utility::getServerPort() {
	return m_server_port;
}

/**
 *  returns the devices ids in string
 */
string Utility::getDevicesIDs() {
	return m_device_numbers;
}


int Utility::getDevicesCount() {
	return m_devices_count;
}


/*find string from another string
parameter:
	str: string from whcih pattern needed to be find out
	find: pattern which needed to find out
	strln: length of string
	findln: pattern length
	str_p: startng point from where patter should start to find out
output:
	index in string where pattern has been find out, -1 on unable to find
*/
int Utility::findpattren(unsigned char *str, unsigned char *find, int strln,int findln,int str_p){
        int a=0,b;
	
        //int substr_len=strlen(find);

	if(findln==1){
		for(a=str_p;a<strln;a++){
			if(str[a]==*(find))
				break;
		}
	}
	else{	
		for(a=str_p;a<strln;a++){
			if(str[a]==*(find)){
				for(b=1;b<findln;b++){
					if(str[a+b]!=*(find+b)){
						break;
					}
				}
				if(b==findln)
					break;
			}
		}
	}
        if(a==strln)
                return -1;
        return a;
}

/**
 * Get the address of server to connect to.
 */
bool Utility::loadDevices(int* devices_array) {


	int max_size = 8;
	int current_position = 0;
	int last_position = 0;


	m_devices_count = 0;

	int device_id = 0;

	if(m_device_numbers.size() != 0)
	{

		while(true)
		{

			current_position = m_device_numbers.find(",", current_position);
			device_id = atoi(m_device_numbers.substr(last_position, current_position).c_str());

			if(device_id < 255)
			{
				devices_array[m_devices_count++] = device_id;
//				cout << devices_array[m_devices_count - 1] << endl;
			}

			if(current_position == -1)
			{
				break;
			}


			if(m_devices_count >= max_size)
			{
				break;
			}

			last_position = current_position + 1;
			current_position++;

		}
	}

	return (m_devices_count != 0);
}


unsigned char* Utility::getMACAddress() {

	struct ifreq ifr;
	struct ifconf ifc;
	const int buf_length = 256;
	char buf[buf_length];
	int success = 0;


	//zero out the array
	for(int i = 0; i < 6; i++)
	{
		mac_address[i] = 6;
	}

	int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
	if (sock == -1) { /* handle error*/
		return mac_address;
	};

	ifc.ifc_len = buf_length;
	ifc.ifc_buf = buf;
	if (ioctl(sock, SIOCGIFCONF, &ifc) == -1) { /* handle error */
		return mac_address;
	}

	struct ifreq* it = ifc.ifc_req;
	const struct ifreq* const end = it + (ifc.ifc_len / sizeof(struct ifreq));

	for (; it != end; ++it) {
		strcpy(ifr.ifr_name, it->ifr_name);
		if (ioctl(sock, SIOCGIFFLAGS, &ifr) == 0) {
			if (!(ifr.ifr_flags & IFF_LOOPBACK)) { // don't count loopback
				if (ioctl(sock, SIOCGIFHWADDR, &ifr) == 0) {
					success = 1;
					break;
				}
			}
		} else { /* handle error */
		}
	}


	if (success) {

		memcpy(mac_address, ifr.ifr_hwaddr.sa_data, 6);

	}


	return mac_address;

}


bool Utility::clearBytes(unsigned char* array, int start, int end) {


	if(start > end)
	{
		return false;
	}

	for (int i = start; i < end; i++)
		array[i] = 0;


	return true;
}



void Utility::printBytes(unsigned char* array, int start, int end) {

//#if DEBUG == 1

	if(start > end)
	{
		printf("Wrong start and end values given for printing bytes \n");
	}


	printf("\n Data = ");

	int i;

	for ( i = start; i < end; i++){
		if (i > 0) {
			printf(":");
		}

		printf("%02X", array[i]);

	}

//#endif

}



/**
 * Write given value in big Endian notation
 * @param value  Value to write
 * @param array  Buffer to write to
 * @param start_position   Position to write at
 */
void Utility::loadIntInArray(int value, unsigned char* array, int start_position) {

	for (int i = 0; i < 2; i++){
		array[start_position + 1 - i] = (value >> (i * 8));
		//printf("%d, %02X, \n", start_position + i, array[start_position + i]);
	}

//	printf("\n");
}


/**
 * Read an int value from given array, starting from given position
 */
int Utility::getIntFromByte(unsigned char* array,
		int start_position) {

	return  array[start_position + 1] | array[start_position] << 8;

}


void Utility::loadLongInArray(unsigned long int longInt, unsigned char* byteArray, int start_position){
	// convert from an unsigned long int to a 4-byte array
/*	byteArray[start_position] = (int)((longInt >> 24) & 0xFF) ;
	byteArray[start_position+1] = (int)((longInt >> 16) & 0xFF) ;
	byteArray[start_position+2] = (int)((longInt >> 8) & 0XFF);
	byteArray[start_position+3] = (int)((longInt & 0XFF));*/
	char byte;
	for ( int index = start_position; index < start_position+8; index ++ ) {
        byte = longInt & 0xff;
        byteArray [ index ] = byte;
        longInt = (longInt - byte) / 256 ;
    }
}

long int Utility::getLongIntFromByte(unsigned char* byteArray, int start_position){
	/*unsigned long int anotherLongInt = ( (byteArray[start_position] << 24) 
			+ (byteArray[start_position+1] << 16) 
			+ (byteArray[start_position+2] << 8) 
			+ (byteArray[start_position+3] ) );
	return anotherLongInt;*/

	long int value = 0;
    for ( int i = (start_position+8) - 1; i >= (start_position); i--) {
        value = (value * 256) + byteArray[i];
    }

    return value;
}
