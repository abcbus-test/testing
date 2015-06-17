/*
 * ModbusController.h
 *
 *  Created on: Feb 13, 2015
 *      Author: imtiazahmed
 */

#ifndef MODBUSCONTROLLER_H_
#define MODBUSCONTROLLER_H_

#include <sys/time.h>
#include <string>
#include "Utility.h"


#define _MIN_REQ_LENGTH 12
#define _MODBUS_RTU_HEADER_LENGTH      1
#define _MODBUS_RTU_PRESET_REQ_LENGTH  6
#define _MODBUS_RTU_PRESET_RSP_LENGTH  2
#define _MODBUS_RTU_CHECKSUM_LENGTH    2
#define _MODBUS_RTU_READ_REGISTER_LENGTH    	8
#define _MODBUS_RTU_WRITE_REGISTER_LENGTH    	11
#define _MODBUS_RTU_WRITE_REQ_LENGTH  9

typedef unsigned char		uint8_t;
typedef unsigned short int	uint16_t;


// following about UART operatin mode
// ioctl command define
// ioctl(fd, MOXA_SET_OP_MODE, &mode)
// ioctl(fd, MOXA_GET_OP_MODE, &mode)
#define MOXA_SET_OP_MODE      (0x400+66)	// to set operating mode
#define MOXA_GET_OP_MODE      (0x400+67)	// to get now operating mode



class ModbusController {

	public:
		ModbusController();
		virtual ~ModbusController();
		int readRegistersInByteArray(int *device_ids, int *registers_to_read, unsigned char* m_communication_message);

		//read the data of a given device of given registers in given array
		int readRegistersOfADevice(int device_id, int *registers_to_read,
				unsigned char* array_to_read_in, int total_registers_count, int& change_index, bool show_output);

		/**
		 * Parse the command and write the values to registers
		 */
		int parseCommand(unsigned char* data, int length);

		int parseCommands(unsigned char* buff, int length,unsigned char* mac_address,struct s_timeout_info* timeout_info);

		bool establishConnection();
		void releaseConnection();
		int remote_update(uint8_t* buffer, int len, struct s_timeout_info* timeout_info);
		unsigned char* getLastReply(int& length_of_data);
		void create_reply(unsigned char* mac,int commandid, bool success);
		int processModbusCommands(unsigned char* buff,int data_pointer);


	private:

		int configurePort();

		void setTimeout();
		bool setSlaveId(int slave_id);
		int getSlaveId();
		int readInputRegister(int slaveId, int function, int addr, int nb, uint8_t *dest);
		uint16_t crc16(uint8_t *buffer, uint16_t buffer_length);

		int writeHoldingRegister(int slaveId, int function, int addr,
				int nb, uint8_t* dest);

		int sendRawCommand(uint8_t* raw_request, int req_length, uint8_t* raw_reply,  int rep_length);
		void printBytes(uint8_t *buffer, int len);
		int sendReadCommand(int device_id, int register_address, int length_to_read, int command_type, uint16_t* tab_reply);

		int sendWriteCommand(int device_id, int register_address,
				int value_to_write, uint16_t* dest);

		const char *m_device;
		int m_time_out;
		int m_fd;
		bool m_select;
		fd_set m_rdfs;
		struct timeval response_timeout;

		Utility m_utility;

		unsigned char m_communication_message[MAX_MESSAGE_LENGTH];

		unsigned char m_reply_message[MAX_MESSAGE_LENGTH];
		int m_reply_length;


};

#endif /* MODBUSCONTROLLER_H_ */

