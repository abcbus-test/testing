/*
 * ServerController.cpp
 *
 *  Created on: Feb 14, 2015
 *      Author: imtiazahmed
 */

#include "ServerController.h"
#include <sstream>
#include <iostream>
#include <string.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdlib.h>     /* strtoul */
#include <errno.h>

#include "ModbusController.h"

using namespace std;

ServerController::ServerController() {
	m_server_ip = "";
	m_socket_file_descriptor = 0;

	s_sock_status=1;

	int i;

	for (i = 0; i < MAX_MESSAGE_LENGTH; i++) {
		m_communication_message[i] = 0;
	}

	for (i = 0; i < m_buffer_length; i++) {
		m_buffer[i] = 0;
	}

}

ServerController::~ServerController() {
}

bool ServerController::connectWithServer(string server_ip, int server_port) {

	int code;

	m_port = server_port;
	m_server_ip = server_ip; //server_ip;
	m_server = gethostbyname(m_server_ip.c_str()); //server_ip
	if(m_socket_file_descriptor)
		close(m_socket_file_descriptor);
	m_socket_file_descriptor = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

#if DEBUG  == 1
	cout << "Server = " << server_ip << " Port = " << server_port << endl;
#endif

	if (m_socket_file_descriptor < 0) {
		cout << ("ERROR opening socket\n") << endl;
		return false;
	}

	if (m_server == NULL) {
		cout << "ERROR, no such host" << endl;
		return false;
	}

	bzero((char *) &m_serv_addr, sizeof(m_serv_addr));

	m_serv_addr.sin_family = AF_INET;

	bcopy((char *) m_server->h_addr, (char *)&m_serv_addr.sin_addr.s_addr, m_server->h_length);

	m_serv_addr.sin_port = htons(m_port);

	do{
		code = connect(m_socket_file_descriptor,
				(struct sockaddr *) &m_serv_addr, sizeof(m_serv_addr));

		if (code < 0) {
			//cout << "Connect failed. Error " << endl;
			printf("Connect failed %d:   %s\n",errno, strerror(errno));
			sleep(3);
			//	return false;
		} 
	}while(code<0);
	sleep(10);
#if 1
	timeout.tv_sec = 90;
	timeout.tv_usec = 0;

	if (setsockopt (m_socket_file_descriptor, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout,
				sizeof(timeout)) < 0){
		printf("setsockopt failed\n");
		return false;
	} else {
#if DEBUG  == 1
		cout << "Connected with server!" << endl;
#endif
		//		return true;
	}

	if (setsockopt (m_socket_file_descriptor, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout,
				sizeof(timeout)) < 0){
		printf("setsockopt failed\n");
		return false;
	}
#endif
	s_sock_status=0;
	return true;

}

int ServerController::disconnect() {

	if (m_socket_file_descriptor > 0) {
#if DEBUG  == 1
		//cout << "Disconnected successfully" << endl;
#endif
		return close(m_socket_file_descriptor);

	} else {
		cout << "Socket was not connected" << endl;
		return -1;
	}

}

void ServerController::setIDForCommunication(unsigned char* mac_address) {

	//copy 6 bytes of mac address at the beginning of the message
	for (int i = 0; i < 6; i++) {
		m_communication_message[i] = mac_address[i];
	}

}

/**
 * Send welcome command
 */
void ServerController::sendWelcomeMessage() {

	clearMessage();

	m_communication_message[COMMAND_INDEX] = CONNECT_MESSAGE_COMMAND;
	updateCommandLength(0);

	//for this command the read and write length is same (9)
	int bytesToWrite = MESSAGE_LENGTH_INDEX + 2;

	sendPreparedBytesToServer(bytesToWrite, bytesToWrite);

}
ssize_t ServerController::socketWrite(int sock_fd, void *sock_buf, size_t sock_count){
	return write(sock_fd, sock_buf,sock_count);
}
ssize_t ServerController::socketRead(int sock_fd, void *sock_buf, size_t sock_count){

	//	return read(sock_fd,sock_buf,sock_count);

	return recv(sock_fd , sock_buf , sock_count , 0);
}

/**
 * Send prepared m_communication_message to server.
 * Before calling this method, the data in m_communication_message needs to be prepared first.
 *
 * write_length indicates the number of bytes to send from m_communication_message
 * read_length indicates the number of bytes to read from incoming reply
 */
int ServerController::sendPreparedBytesToServer(unsigned int write_length,
		unsigned int read_length) {

	try {

		m_length = socketWrite(m_socket_file_descriptor, m_communication_message,
				write_length);

		if (m_length < 0) {
			cout << "ERROR writing to socket" << endl;
			m_length = 0;
			s_sock_status=1;
			close(m_socket_file_descriptor);

		} else {

#if DEBUG == 1
			printBytes(true, m_length);
			cout << "Total bytes written on socket" << m_length << endl;
#endif
			//Bytes written on socket, now going to read the server reply
			m_reading_length = read(m_socket_file_descriptor, m_buffer,
					MAX_MESSAGE_LENGTH);

			if(m_reading_length == 0 || errno==11)
			{
				s_sock_status=1;
				close(m_socket_file_descriptor);
				puts("Client disconnected");
			}
			else if(m_reading_length == -1)
			{
				s_sock_status=1;
				close(m_socket_file_descriptor);
				fprintf(stdout,"[Error] Socket read failed%d:	%s",errno, strerror(errno));
			}
			else {

#if DEBUG == 1

				printBytes(false, m_reading_length);
				cout << "Total bytes read from Server" << m_reading_length << endl;
#endif
			}

		}

	} catch (...) {
		m_length = 0;
	}

	return m_length;

}

unsigned long ServerController::getLongFromByte(unsigned char* array,
		int start_position) {

	return ((array[start_position + 3] << 24)
			| (array[start_position + 2] << 16)
			| (array[start_position + 1] << 8) | (array[start_position]));

}

bool ServerController::sendDevicesStatus(unsigned char* data, int length) {

	clearMessage();
	int bytesToWrite;
	int byteToRead;

	if(length>0){
		m_communication_message[COMMAND_INDEX] = GET_STATUS_OF_ALL_SLAVES_COMMAND;
		updateCommandLength(length);

		//for this command write length comes as a parameter
		bytesToWrite = MESSAGE_LENGTH_INDEX + 1 + length;
		byteToRead = 9;

		for (int i = 0; i < MAX_MESSAGE_LENGTH; i++) {
			m_communication_message[MESSAGE_LENGTH_INDEX + i] = data[i];
		}
	}
	else{
		m_communication_message[COMMAND_INDEX] = REPORT_SLAVE_STATUS_COMMAND;
		updateCommandLength(4);

		//for this command write length comes as a parameter
		bytesToWrite = MESSAGE_LENGTH_INDEX + 1 + 6;
		byteToRead = 9;
		m_communication_message[10]=1;
		m_communication_message[11]=data[3];

		/*                for (int i = 0; i < 4; i++) {
				  m_communication_message[10 + i] = data[i];
				  }*/
	}

	return sendPreparedBytesToServer(bytesToWrite, byteToRead) > 0;

}

void ServerController::clearMessage() {

	for (int i = COMMAND_STATUS_INDEX; i < MAX_MESSAGE_LENGTH; i++) {
		m_communication_message[i] = 0;
	}

}

bool ServerController::isConnected(){
	if(s_sock_status==0)
		return true;
	else
		return false;

}
void ServerController::updateCommandLength(int length) {

	//Set the status to 2 to indicate that this message is request.
	m_communication_message[COMMAND_STATUS_INDEX] = 2;

	for (int i = MESSAGE_LENGTH_INDEX; i < MESSAGE_LENGTH_INDEX + 2; i++)
		m_communication_message[i] = (length >> (i * 8));

}

void ServerController::getArrayFromLong(unsigned long number,
		unsigned char* array, int start_position) {

	for (int i = start_position; i < 4; i++)
		array[i] = (number >> (i * 8));

}

unsigned char* ServerController::getLastReplyFromServer(int& length_of_data) {

	length_of_data = m_reading_length;
	return m_buffer;

}

void ServerController::printBytes(bool out_going, int length) {

#if DEBUG == 1
	cout << endl << "Printing bytes = ";
	int i;

	for (i = 0; i < length; i++) {
		if (i > 0) {
			printf(":");
		}
		if(out_going)
		{
			printf("%02X", m_communication_message[i]);
		}
		else
		{
			printf("%02X", m_buffer[i]);
		}
	}
	cout << endl;
#endif

}

void ServerController::sendCommandReply(unsigned char* packets,int length) {

	int n=socketWrite(m_socket_file_descriptor, packets,length);
	if (n < 0){ 
		printf("ERROR writing to socket");
		s_sock_status=1;
		close(m_socket_file_descriptor);
	}

	//TODO implement the code to reply back the result of writing
	//to registers
}
