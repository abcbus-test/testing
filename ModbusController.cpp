/*
 * ModbusController.cpp
 *
 *  Created on: Feb 13, 2015
 *      Author: imtiazahmed
 */

#include <sys/select.h>
#include "ModbusController.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitions
#include <time.h>   // time calls
#include <sys/ioctl.h>
#include <iostream>
#include <sstream>
#include <cmath>
#include "Utility.h"
#include <stdlib.h>

using namespace std;

/* Table of CRC values for high-order byte */
static const uint8_t table_crc_hi[] = { 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
	0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
	0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
	0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
	0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
	0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
	0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40 };

/* Table of CRC values for low-order byte */
static const uint8_t table_crc_lo[] = { 0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03,
	0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C,
	0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
	0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE,
	0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17,
	0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30,
	0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35,
	0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
	0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B,
	0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24,
	0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21,
	0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6,
	0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
	0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8,
	0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD,
	0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2,
	0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53,
	0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
	0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59,
	0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E,
	0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 0x44, 0x84, 0x85, 0x45, 0x87, 0x47,
	0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40 };

ModbusController::ModbusController() {
	// TODO Auto-generated constructor stub

#if DEVICE == 0
	m_device = "/dev/ttyUSB1";
#else
	//	m_device = "/dev/ttyUSB0";
	m_device = "/dev/ttyM0";
#endif

	m_fd = -1;
	m_time_out = 2;

#if DEVICE == 1

	int mode;
	int fd;
	fd = open(m_device, O_WRONLY);

	//Get the serial port mode for MOXA box
	ioctl(fd, MOXA_GET_OP_MODE, &mode);

#if DEBUG == 1
	cout << " get  mode = " << mode << std::endl;
#endif

	//if it is not RS485 - 2 Wires, set it
	if (mode != 1) {
		mode = 1;
		ioctl(fd, MOXA_SET_OP_MODE, &mode);
	}

#if DEBUG == 1
	cout << "port mode = " << mode << std::endl;
#endif

	close(fd);

#endif

}

ModbusController::~ModbusController() {

}

/**
 * establish connection
 */
bool ModbusController::establishConnection() {

	// file description for the serial port
	m_fd = open(m_device, O_RDWR | O_NOCTTY | O_NDELAY | O_EXCL);

	if (m_fd == -1) { // if open is unsucessful
#if DEBUG == 1
		cout << ("open_port: Unable to open  . \n");
#endif
		return false;
	} else {
		fcntl(m_fd, F_SETFL, 0);
#if DEBUG == 1
		cout << ("port is open.\n");
#endif
		cout << ("...");
	}

	setTimeout();

	if (configurePort() > 0) {
		return true;
	} else {
		return false;
	}

}

/**
 * Configure port
 */
int ModbusController::configurePort()      // configure the port
{
	struct termios port_settings;     // structure to store the port settings in

	//ZERO out the struct to keep valgrind happy
	for (int i = 0; i < NCCS; i++) {
		port_settings.c_cc[i] = 0;
	}
	port_settings.c_cflag = 0;
	port_settings.c_iflag = 0;
	port_settings.c_ispeed = 0;
	port_settings.c_lflag = 0;
	port_settings.c_line = 0;
	port_settings.c_oflag = 0;
	port_settings.c_ospeed = 0;

	//end of initializing

	cfsetispeed(&port_settings, B19200);    // set baud rates
	cfsetospeed(&port_settings, B19200);

	port_settings.c_cflag |= (CREAD | CLOCAL);

	port_settings.c_cflag &= ~CSIZE;
	port_settings.c_cflag |= CS8;
	port_settings.c_cflag &= ~ CSTOPB;

	port_settings.c_cflag |= PARENB;
	port_settings.c_cflag &= ~ PARODD;
	port_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	port_settings.c_iflag |= INPCK;

	// Raw ouput
	port_settings.c_oflag &= ~ OPOST;
	port_settings.c_cc[VMIN] = 0;
	port_settings.c_cc[VTIME] = 0;

	if (tcsetattr(m_fd, TCSANOW, &port_settings) < 0) {
		close(m_fd);
		m_fd = -1;
	}

	return (m_fd);

} //configure_port

/**
 * close connection and release memory
 */
void ModbusController::releaseConnection() {

	int successful_close=0;
	m_select = false;
	if (m_fd > 0) {
		successful_close=close(m_fd);
		cout<< "Serial release status " << successful_close;
	}

}

/**
 * set the time out
 */
void ModbusController::setTimeout() {

	response_timeout.tv_sec = 0;
	response_timeout.tv_usec = 1000 * 200;

}

/**
 * Send command to given device id
 */
int ModbusController::sendReadCommand(int device_id, int register_address,
		int length_to_read, int command_type, uint16_t* dest) {

	int i;
	for (i = 0; i < length_to_read; i++) {
		dest[i] = 0;
	}

	uint8_t tab_reg[_MODBUS_RTU_READ_REGISTER_LENGTH];

	int req_length = readInputRegister(device_id, command_type,
			register_address, length_to_read, tab_reg);

	int rep_length = _MODBUS_RTU_PRESET_REQ_LENGTH + length_to_read;

	uint8_t raw_reply[rep_length];

	memset(raw_reply,0,rep_length);

	for (i = 0; i < rep_length; i++) {
		raw_reply[i] = 0;
	}

	int bytes_read = sendRawCommand(tab_reg, req_length, raw_reply, rep_length);

	if (bytes_read > 0) {

		int offset = 1;

		for (i = 0; i < length_to_read; i++) {
			/* shift reg hi_byte to temp OR with lo_byte */
			dest[i] = (raw_reply[offset + 2 + (i << 1)] << 8)
				| raw_reply[offset + 3 + (i << 1)];
		}
	}

	return bytes_read;
}

/**
 * modbus_set_slave - set slave number in the context
 */
bool ModbusController::setSlaveId(int slave_id) {

	//TODO:
	return false;
}

/**
 *
 *
 Slave ID			1
 Records Length		1
 Address				2
 Value				2
 Address				2
 Value				2
 Address				2
 Value				2

 *
 */
int ModbusController::readRegistersInByteArray(int* local_devices,
		int* registers, unsigned char* m_communication_message) {

	/*
	 * If connection is not established, return 0
	 */

	return -1;

}

int ModbusController::readRegistersOfADevice(int device_id,
		int* registers_to_read, unsigned char* array_to_read_in,
		int total_registers_count, int& change_counter, bool show_output) {

	/*	if (!establishConnection()) {
		return 0;
		}*/

	int data_pointer = 3;

	array_to_read_in[1] = 3 + (total_registers_count * 4) + (5); //calculate the message length
	array_to_read_in[2] = 1; //total records = total devices attached

	array_to_read_in[data_pointer] = device_id; // fill in the device id
	array_to_read_in[++data_pointer] = total_registers_count; //fill in the number of records.

	data_pointer++;

	uint16_t result[1];
	result[0] = 0;

	if (show_output) {
		cout << endl << "****************************************";
		cout << endl << "Device_id = " << device_id << endl;
	}

	int command_type = 0;

	for (int ctr = 0; ctr < total_registers_count; ctr++) {

		command_type = 4;

		if (registers_to_read[ctr] == 1001 || registers_to_read[ctr] == 1002
				|| registers_to_read[ctr] == 1004) {
			command_type = 3;
		}

		sendReadCommand(device_id, registers_to_read[ctr], 1, command_type,
				result);

		//fill in the register number - this will take next two bytes
		m_utility.loadIntInArray(registers_to_read[ctr], array_to_read_in,
				data_pointer);
		data_pointer += 2; //reposition the pointer

		int oldValue = m_utility.getIntFromByte(array_to_read_in, data_pointer);

		int newValue = result[0];

		if (oldValue != newValue) {
			if (show_output) {
				cout << "-------------------------------------" << endl;
				printf("\nr= %d   v= %d   oldv=%d \n", registers_to_read[ctr],
						result[0], oldValue);
				cout << endl << "-------------------------------------" << endl;
			}
			change_counter++;
		}

		//fill in the register value -  this will take next two bytes
		m_utility.loadIntInArray(newValue, array_to_read_in, data_pointer);

#if DEBUG == 1
		printf("r= %d   v= %d\toldv=%d \n", registers_to_read[ctr], result[0] , oldValue );
		cout << endl << "-------------------------------------" << endl;
#endif

		data_pointer += 2; //reposition the pointer
	}

	//releaseConnection();

	return data_pointer;

}

/**
 * modbus_report_slave_id - returns a description of the controller
 */

int ModbusController::getSlaveId() {

	//TODO:
	return -1;
}

int ModbusController::sendRawCommand(uint8_t* raw_request, int reqs_length,
		uint8_t* raw_reply, int rep_length) {

	int n = -1;

	//	cout << ("\nBytes write on the port! \n");
	//        printBytes(raw_request, reqs_length);
	n = write(m_fd, raw_request, reqs_length);  //Send data

	if (raw_reply[1] == 16) {
		cout << (" Total bytes wrote. ");
		cout << n << endl;
		printBytes(raw_request, reqs_length);
	}

	usleep(35 * 500);

	n = select(m_fd + 1, &m_rdfs, NULL, NULL, &response_timeout);

#if DEBUG == 1
	cout << (" Select returned  ") << n << endl;
	cout << (" About to read bytes. ");
	cout << rep_length << endl;
#endif

	usleep(35 * 500);

	n = read(m_fd, raw_reply, rep_length);

#if DEBUG == 1
	cout << ("bytes read ") << n;
#endif

	// check if an error has occurred
	if (n < 0) {
#if DEBUG == 1
		cout << ("select failed\n");
#endif
	} else if (n == 0) {
#if DEBUG == 1
		cout << ("Timeout! \n");
#endif
	} else {

		if (raw_reply[1] == 16) {
			cout << ("\nBytes detected on the port! \n");
			printBytes(raw_reply, n);
		}
		return raw_reply[2]; //3rd byte has the length of data
	}
	return n;

}

int ModbusController::readInputRegister(int slaveId, int function, int addr,
		int nb, uint8_t* dest) {

	dest[0] = slaveId;
	dest[1] = function;
	dest[2] = addr >> 8;
	dest[3] = addr & 0x00ff;
	dest[4] = nb >> 8;
	dest[5] = nb & 0x00ff;

	uint16_t req_length = _MODBUS_RTU_PRESET_REQ_LENGTH;
	uint16_t crc = crc16(dest, req_length);
	dest[req_length++] = crc >> 8;
	dest[req_length++] = crc & 0x00FF;

	return req_length;

}

/**

  Preset Multiple Registers (FC=16)

  Request

  This command is writing the contents of two analog output holding registers # 40002 & 40003 to the slave device with address 17.
  ==================================================
  11 10 0001 0002 04 000A 0102 C6F0
  ==================================================
11: 	The Slave Address (17 = 11 hex)
10: 	The Function Code (Preset Multiple Registers 16 = 10 hex)
0001:	The Data Address of the first register. (# 40002 - 40001 = 1 )
0002: 	The number of registers to write
04: 	The number of data bytes to follow (2 registers x 2 bytes each = 4 bytes)
000A: 	The value to write to register 40002
0102: 	The value to write to register 40003
C6F0: 	The CRC (cyclic redundancy check) for error checking.

 */
int ModbusController::writeHoldingRegister(int slaveId, int function, int addr,
		int value, uint8_t* dest) {

	int number_of_reg = 1; //we will write one register at a time as the register are not in sequence

	dest[0] = slaveId;
	dest[1] = function;
	dest[2] = addr >> 8;
	dest[3] = addr & 0x00ff;
	dest[4] = number_of_reg >> 8;
	dest[5] = number_of_reg & 0x00ff;

	//The number of data bytes to follow (2 registers x 2 bytes each = 4 bytes)
	dest[6] = 2;

	//The value to write to register
	dest[7] = value >> 8;
	dest[8] = value & 0x00ff;

	uint16_t req_length = _MODBUS_RTU_WRITE_REQ_LENGTH;
	uint16_t crc = crc16(dest, req_length);
	dest[req_length++] = crc >> 8;
	dest[req_length++] = crc & 0x00FF;

	return req_length;

}

uint16_t ModbusController::crc16(uint8_t *buffer, uint16_t buffer_length) {
	uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
	uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
	unsigned int i; /* will index into CRC lookup */

	/* pass through message buffer */
	while (buffer_length--) {
		i = crc_hi ^ *buffer++; /* calculate the CRC  */
		crc_hi = crc_lo ^ table_crc_hi[i];
		crc_lo = table_crc_lo[i];
	}

	return (crc_hi << 8 | crc_lo);
}

void ModbusController::printBytes(uint8_t* buffer, int len) {

#if DEBUG == 3
	cout << "printing bytes = ";
	int i;

	for (i = 0; i < len; i++) {
		if (i > 0) {
			cout << (":");
		}
		printf("%02X", buffer[i]);
	}
	cout << endl <<"Interger view: "<<endl;

	for (i = 0; i < len; i++) {
		if (i > 0) {
			cout << (",");
		}
		printf("%d", buffer[i]);
	}
	cout << endl;
#endif

}

unsigned char* ModbusController::getLastReply(int& length_of_data) {

	length_of_data = m_reply_length;
	return m_reply_message;

}
void ModbusController::create_reply(unsigned char* mac,int commandid, bool success){
	memcpy(m_reply_message+m_reply_length,mac,6);
	m_reply_message[m_reply_length+6]=commandid;
	if(success)
		m_reply_message[m_reply_length+7]=1;
	else
		m_reply_message[m_reply_length+7]=0;
	m_reply_message[m_reply_length+8]=0;
	m_reply_message[m_reply_length+9]=0;
	m_reply_length=m_reply_length+10;


}
int ModbusController::remote_update(uint8_t* buffer, int len,struct s_timeout_info * timeout_info) {

	uint8_t data_buf[UPDATE_APPLICATION_DATASIZE];
	FILE * pFileTXT;
	int buff_length;

	if (buffer[7]!=2){
		printf("Not a request!!!\n");
		return 0;
	}
	/*if (len!=96){
	  printf("Wrong Size!!!\n");
	  return 0;
	  }*/

	//	int packet_num = m_utility.getIntFromByte(buffer, 8);
	long int packet_num=  m_utility.getLongIntFromByte(buffer,8);
	long int total_packets=  m_utility.getLongIntFromByte(buffer,16);
	long packet_crc = m_utility.getIntFromByte(buffer, len-2);
	buff_length=len-(UPDATA_APPLICATION_FRAMESIZE+CRCSIZE);
	long crc=crc16(buffer, len-2);
	if(crc!=packet_crc)
		return 0;
	memcpy(data_buf,buffer+UPDATA_APPLICATION_FRAMESIZE,buff_length);
	if(packet_num<1){
		pFileTXT = fopen ("recv_file","w");
		(timeout_info[0].timeout_status)=true;
	}
	else
		pFileTXT = fopen ("recv_file","a");// use "a" for append, "w" to overwrite, previous content will be deleted


	fwrite(data_buf, sizeof(data_buf[0]), buff_length , pFileTXT);
	fclose (pFileTXT); // must close after opening
	if(packet_num>=(total_packets-1)){

		(timeout_info[0].timeout_status)=false;

/*		system("cp recv_file 1.0.0.tar.gz");

		sleep(1);
		system("chmod 755 *");

		sleep(1);

		system("tar xvf 1.0.0.tar.gz");


		system("mkdir remote_cache");
		sleep(1);*/

		system("mkdir remote_cache");

		sleep(1);
		system("chmod 755 *");

		sleep(1);
		system("cp recv_file remote_cache/remote_update.tar.gz");
		sleep(1);

		system("tar xvf remote_cache/remote_update.tar.gz -C remote_cache/");
		sleep(3);

		system("sh remote_cache/install.sh");
		sleep(3);

	}
	return 1;

}

int ModbusController::processModbusCommands(unsigned char* data,int data_pointer){
	int device_id = 0;
	int total_registers_count = 0;
	int total_devices_count = 0;
	int register_address = 0;
	int register_value = 0;

	uint16_t result[1];
	result[0] = 0;
	int remaining_packet_length = m_utility.getIntFromByte(data, data_pointer);



	if (remaining_packet_length >6) {
		data_pointer += 2;

		//check how many devices we need to update.
		total_devices_count = data[data_pointer++];

		//loop through the devices
		for (int i = 0; i < total_devices_count; i++) {
			device_id = data[data_pointer++];
			total_registers_count = data[data_pointer++];

			cout << endl << "Device ID = " << device_id << endl;

			//check the registers that are needed to be updated.
			//loop through them and send the write register command one by one
			//it is that easy.
			for (int ctr = 0; ctr < total_registers_count; ctr++) {
				register_address = m_utility.getIntFromByte(data, data_pointer);
				data_pointer += 2;

				register_value = m_utility.getIntFromByte(data, data_pointer);
				data_pointer += 2;

				cout << endl << "-------------------------------------" << endl;
				printf("r= %d   v= %d\n", register_address, register_value);
				cout << endl << "-------------------------------------";

				sendWriteCommand(device_id, register_address, register_value,
						result);
			}

			cout << endl;
		}
	}
}

int ModbusController::parseCommands(unsigned char* buff, int length,unsigned char* mac_address,struct s_timeout_info* timeout_info){

	//Sample data coming in
	//
	//00 :0C :29 :AF :71 :3B :31 :02 :00 :07 :01 :1F :01 :03 :E9 :00 :01
	//
	//0-5    MAC ID
	//6th    Command number, must be 49

	int device_id = 0;
	int total_registers_count = 0;
	int total_devices_count = 0;
	int data_pointer = 6;
	int register_address = 0;
	int register_value = 0;
	int reply_state;
	int num_reply;

	uint16_t result[1];
	result[0] = 0;


	unsigned char data[2100];
	int packet_length=0;
	int pakcet_index=0;


	memset(m_reply_message,0,MAX_MESSAGE_LENGTH);
	m_reply_length=0;
	packet_length=m_utility.findpattren(buff, mac_address, length,6,pakcet_index);
	if(packet_length<0)
		return 0;
	do{
		data_pointer = 6;
		memset(data,0,1000);
		packet_length=m_utility.findpattren(buff, mac_address, length,6,pakcet_index+6);
		if(packet_length<0)
			packet_length=length-pakcet_index;
		else{
			packet_length=packet_length-pakcet_index;

		}

		memcpy(data,buff+pakcet_index,packet_length);

		int command=data[data_pointer++];
		int status=data[data_pointer++];
		switch(command){
			case 100:
				reply_state=remote_update(data, packet_length,timeout_info);
				create_reply(mac_address,command,reply_state);

				timeout_info[0].timeout_cycle=true;
				timeout_info[0].timeout_len=m_reply_length;

				memcpy(timeout_info[0].timeout_packet,m_reply_message,m_reply_length);
				break;

			default:
				break;


		}


		int remaining_packet_length = m_utility.getIntFromByte(data, data_pointer);



		//move the pointer forward as last remaining_packet_length is of two bytes.
		if(command==4 || command==5 || command==49 || command==50 ){
			if (remaining_packet_length >6) {
				data_pointer += 2;

				//check how many devices we need to update.
				total_devices_count = data[data_pointer++];

				//loop through the devices
				for (int i = 0; i < total_devices_count; i++) {
					device_id = data[data_pointer++];
					total_registers_count = data[data_pointer++];

					cout << endl << "Device ID = " << device_id << endl;

					//check the registers that are needed to be updated.
					//loop through them and send the write register command one by one
					//it is that easy.
					for (int ctr = 0; ctr < total_registers_count; ctr++) {
						register_address = m_utility.getIntFromByte(data, data_pointer);
						data_pointer += 2;

						register_value = m_utility.getIntFromByte(data, data_pointer);
						data_pointer += 2;

						cout << endl << "-------------------------------------" << endl;
						printf("r= %d   v= %d\n", register_address, register_value);
						cout << endl << "-------------------------------------";

						sendWriteCommand(device_id, register_address, register_value,
								result);
					}

					cout << endl;
				}
			}
		}

		pakcet_index=pakcet_index+packet_length;
	}while(pakcet_index<length);
	return 0;
}

/**

  ==================================================================================================
  Name	                Length	    Hex	                        Dec	            Description
  ==================================================================================================
  MAC Address	        6	        00 0C 29 AF 71 3B
  Command	            1	        51	                        50	            Command Number
  Status	                1	        1	                        1	            1 = Success
  Length	                2	        44	                        68	            Length of remaining packet
  Number of Records	    1	        2	                        2	            Total records in this packet. One record corresponds to one slave.
  Slave ID	            1	        1F	                        31	            Slave Number
  Records Length	        1	        22	                        34	            Number of registers along with their values in this packet.
  Address	            2	        00 CB	                    203
  Value	                2	        09 2F	                    2351
  Address	            2	        00 DD	                    221
  Value	                2	        00 00	                    0
  Address	            2	        03 EB	                    1003
  Value	                2	        00 00	                    0
  ===================================================================================================

 */
int ModbusController::parseCommand(unsigned char* data, int length) {

	//Sample data coming in
	//
	//00 :0C :29 :AF :71 :3B :31 :02 :00 :07 :01 :1F :01 :03 :E9 :00 :01
	//
	//0-5    MAC ID
	//6th    Command number, must be 49

	int device_id = 0;
	int total_registers_count = 0;
	int total_devices_count = 0;
	int data_pointer = 6;
	int register_address = 0;
	int register_value = 0;


	uint16_t result[1];
	result[0] = 0;
	int command=data[data_pointer++];
	int status=data[data_pointer++];
	switch(command){
		case 3:
			printf("yas %d: Read Holding Registers Status=%d func=%s, file=%s\n",__LINE__,status,__FUNCTION__,__FILE__);
			break;
		case 4:
			printf("yas %d: Read Input Registers Status=%d func=%s, file=%s\n",__LINE__,status,__FUNCTION__,__FILE__);
			break;
		case 16:
			printf("yas %d: Write one or more holding registers Status=%d func=%s, file=%s\n",__LINE__,status,__FUNCTION__,__FILE__);
			break;
		case 20:
			printf("yas %d: RAW Modbus Command Status=%d func=%s, file=%s\n",__LINE__,status,__FUNCTION__,__FILE__);
			break;
		case 40:
			printf("yas %d: Connect Message Status=%d func=%s, file=%s\n",__LINE__,status,__FUNCTION__,__FILE__);
			break;
		case 50:
			printf("yas %d: Get Status Status=%d func=%s, file=%s\n",__LINE__,status,__FUNCTION__,__FILE__);
			break;
		case 51:
			printf("yas %d: Update Sys Datetime Status=%d func=%s, file=%s\n",__LINE__,status,__FUNCTION__,__FILE__);
			break;
		case 52:
			printf("yas %d: Reboot Gateway? Status=%d func=%s, file=%s\n",__LINE__,status,__FUNCTION__,__FILE__);
			break;
		case 100:
			printf("yas %d: Update Application Status=%d func=%s, file=%s\n",__LINE__,status,__FUNCTION__,__FILE__);
			break;
		case 49:
			printf("yas %d: Write one or more holding registers Status=%d func=%s, file=%s\n",__LINE__,status,__FUNCTION__,__FILE__);
			break;
		default:
			printf("yas %d: Command ID=%d Status=%d func=%s, file=%s\n",__LINE__,command,status,__FUNCTION__,__FILE__);
			//printBytes(data,length,length);

	}

	/////////////////////////////////////////////end temp/////////////////////////////////////
	//check the command number on 7th bit (starting from zero)
	/*	if ( data[data_pointer++] != 0x31) {
		return 0;
		}

	//check if status on 8th bit is set to 2 (means it is a request to act on)
	if (data[data_pointer++] != 0x02) {
	return 0;
	}*/

	//get the length of remaining packet from 9th and 10th position
	int remaining_packet_length = m_utility.getIntFromByte(data, data_pointer);

	//even if one register has to be written, remaining_packet_length can not be less than 7
	if (remaining_packet_length < 7) {
		return 0;
	}

	/*	if (!establishConnection()) {
		return 0;
		}*/

	//move the pointer forward as last remaining_packet_length is of two bytes.
	data_pointer += 2;

	//check how many devices we need to update.
	total_devices_count = data[data_pointer++];

	//loop through the devices
	for (int i = 0; i < total_devices_count; i++) {
		device_id = data[data_pointer++];
		total_registers_count = data[data_pointer++];

		cout << endl << "Device ID = " << device_id << endl;

		//check the registers that are needed to be updated.
		//loop through them and send the write register command one by one
		//it is that easy.
		for (int ctr = 0; ctr < total_registers_count; ctr++) {
			register_address = m_utility.getIntFromByte(data, data_pointer);
			data_pointer += 2;

			register_value = m_utility.getIntFromByte(data, data_pointer);
			data_pointer += 2;

			cout << endl << "-------------------------------------" << endl;
			printf("r= %d   v= %d\n", register_address, register_value);
			cout << endl << "-------------------------------------";

			sendWriteCommand(device_id, register_address, register_value,
					result);
		}

		cout << endl;
	}

	//releaseConnection();

	return 0;
}

/**
 * Send command to given device id
 */
int ModbusController::sendWriteCommand(int device_id, int register_address,
		int value_to_write, uint16_t* dest) {

	int i;
	for (i = 0; i < 2; i++) {
		dest[0] = 0;
	}

	uint8_t tab_reg[_MODBUS_RTU_WRITE_REGISTER_LENGTH];

	int req_length = writeHoldingRegister(device_id, 16, register_address,
			value_to_write, tab_reg);

	int rep_length = _MODBUS_RTU_WRITE_REQ_LENGTH - 1;

	uint8_t raw_reply[rep_length];

	for (i = 0; i < rep_length; i++) {
		raw_reply[0] = 0;
	}

	int bytes_read = sendRawCommand(tab_reg, req_length, raw_reply, rep_length);

	if (bytes_read > 0) {

		int offset = 1;

		for (i = 0; i < 2; i++) {
			dest[i] = (raw_reply[offset + 2 + (i << 1)] << 8)
				| raw_reply[offset + 3 + (i << 1)];
		}
	}

	return bytes_read;
}

