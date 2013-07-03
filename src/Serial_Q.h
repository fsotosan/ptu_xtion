/*
 * Serial_Q.h
 *
 *  Created on: Jun 28, 2013
 *      Author: youbot
 */

#ifndef SERIAL_Q_H_
#define SERIAL_Q_H_

#include <iostream>		/* Definiciones de la librería estandar de entrada/salida de C++*/
#include <cstdio>   	/* Definiciones de la librería estandar de entrada/salida */
#include <cstdlib>		/* Definiciones de la librería estandar de propósito general */
#include <cstring>  	/* Definiciones relacionadas con cadenas de caracteres */
#include <cerrno>   	/* Definiciones de códigos de error */
#include <csignal> 		/* Definiciones de señales */

using namespace std;

extern "C" {

	#include <termios.h> 	/* Definiciones relacionadas con control de terminal POSIX */
	#include <fcntl.h>   	/* Definiciones relacionadas con control de ficheros */
	#include <stdbool.h>	/* Definiciones para uso de variables booleanas */
}

#define	Parity_None 	IGNPAR
#define	Parity_Even  	PARENB
#define	Parity_Odd  	PARENB | PARODD

#define StopBits_1 	0
#define StopBits_2  CSTOPB

#define DataBits_5	CS5
#define DataBits_6	CS6
#define DataBits_7	CS7
#define DataBits_8	CS8

#define ReadMode_SyncBlocking		0
#define ReadMode_SyncNonBlocking	1
#define ReadMode_AsyncWithSignal	2

typedef void (*signalHandler)(int);

namespace Serial {

	//////////////////////////////////////////////
	// Clase para comunicación serie con termios//
	//////////////////////////////////////////////

	class Serial {

	private:

		// Miembros privados

		struct termios myOldTIO, myNewTIO;
		struct sigaction mySigAction;

		char* myDevice;
		int myBaudRate;
		int myDataBits;
		int myStopBits;
		int myParity;
		int myReadMode;

	protected:

		int myHandler;

		signalHandler mySignalHandler;

	public:

		// Miembros públicos

		char* LastError;

		Serial(const char* inDevice, int inBaudRate, int inDataBits, int inParity, int inStopBits, int inReadMode, signalHandler inSignalHandler = NULL);

		~Serial();

		bool connect();
		int send(const char* inBytes);
		int receive(char* inBytes, int inMaxNumBytes);

	};

	//////////
	// Cola //
	//////////

	class Queue {

	private:

		// Miembros privados

		int mySize;
		unsigned char* myBuff;
		int myHead;
		int myNumBytes;

	public:

		// Miembros públicos

		Queue(int inSize);

		~Queue();

		int getNumBytes();
		int getSize();
		unsigned char* getFullContent(bool inDequeueBytes);
		unsigned char dequeue();
		void enqueue(const unsigned char c);

	};

	//////////////////////////////////////////////////////////////////////////////
	// Clase de comunicación serie simplificada 						  		//
	//																			//
	//  - Se configura solo con la velocidad y el nombre del dispositivo  		//
	//    usando los demás parámetros por defecto (los habituales 8N1)	 		//
	//																			//
	//  - Utiliza lectura síncrona sin bloqueo insertando los bytes recibidos   //
	//    en una cola (en las llamadas a checkDataAndEnqueue())     			//
	//																			//
	//////////////////////////////////////////////////////////////////////////////

	class Serial_Q : public Serial {

	private:

		Queue* myQ;

	public:

		Serial_Q(const char* inDevice, int inBaudRate);

		bool checkDataAndEnqueue();
		unsigned char dequeue();
		unsigned char* getFullQueueContent(bool inDequeueBytes);
		int getNumBytesInQ();

	};

}

#endif /* SERIAL_Q_H_ */
