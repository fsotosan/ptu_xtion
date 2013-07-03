//============================================================================
// Name        : Serial_Q.cpp
// Author      : Fernando Soto
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

/*
 * Serial.cpp
 *
 *  Created on: Jun 27, 2013
 *      Author: Fernando Soto
 *
 *  Clases:
 *
 *  	Serial: implementación en C++ de comunicación serie con termios
 *  	Queue: cola
 *  	Serial_Q: clase para comunicación serie simplificada
 *
 */

#include "Serial_Q.h"

namespace Serial {

	Serial::Serial(const char* inDevice, int inBaudRate, int inDataBits, int inParity, int inStopBits, int inReadMode, signalHandler inSignalHandler) {

		int theFlags;

		// Inicializamos los miembros privados con los datos de configuración

		this->myDevice = strdup(inDevice);
		this->myBaudRate = inBaudRate;
		this->myDataBits = inDataBits;
		this->myParity = inParity;
		this->myStopBits = inStopBits;
		this->myReadMode = inReadMode;

		cout << "Iniciando conexión serie en " << this->myDevice << " a " << this->myBaudRate << " bps " << this->myDataBits << this->myParity << this->myStopBits << " en modo " << this->myReadMode << endl;

		this->mySignalHandler = inSignalHandler;
		this->LastError = NULL;

		// Establecemos los Flags de configuración para abrir el dispositivo serie
		// O_RDRW para comunicación bidireccional (lectura/escritura)
		// O_NOCTTY para que el terminal no controle nuestro proceso.
		//   (esto evita entre otras cosas que se reciban señales inesperadas procedentes del terminal).

		theFlags =  O_RDWR | O_NOCTTY;

		// Configuración según tipo de lectura

		if (this->myReadMode !=  ReadMode_SyncBlocking) {

			// O_NONBLOCK para indicar que las lecturas deben retornar el control inmediatamente,
			// sin esperar por un número mínimo de bytes recibidos.

			theFlags |= O_NONBLOCK;
		}

		// Abrimos dispositivo serie.
		// Si no hay error, open devolverá un entero positivo

		this->myHandler = open(this->myDevice,theFlags);

		// Si se produce un error, el código de error se puede obtener en la variable errno
		// y sus significado mediante strerror(errno).
		// Es necesario incluir cabecera errno.h para esto.

		if (this->myHandler < 0) {
			this->LastError = strerror(errno);
		}

		// Salvamos la configuración original del dispositivo serie en myOldTIO

		tcgetattr(this->myHandler,&this->myOldTIO);


		// Inicializamos la nueva configuración (myNewTIO) con ceros

		bzero(&this->myNewTIO, sizeof(this->myNewTIO));


		// Comprobamos si se trata de modo asíncrono

		if (this->myReadMode == ReadMode_AsyncWithSignal) {

			// Las lecturas asíncronas se gestionan mediante el uso de señales.
			// El sistema operativo envia una señal SIGIO cada vez que llegue un dato al dispositivo serie
			// Es responsabilidad del usuario de esta clase el programar la función que reaccionará ante la señal
			// y pasar un puntero a la misma en la llamada constructor de Serial.
			// Importante: el envío de señales a un proceso dormido (mediante sleep) provoca su reanudación.

			// Asignamos la función delegada para hacerse cargo de la señal

			this->mySigAction.sa_handler = this->mySignalHandler;
			//mySigAction.sa_mask = 0;
			this->mySigAction.sa_flags = 0;
			this->mySigAction.sa_restorer = NULL;

			sigaction(SIGIO,&this->mySigAction,NULL);

			// Permitimos al proceso recibir señales

			fcntl(this->myHandler, F_SETOWN, getpid());

			// Hacemos asíncrono el descriptor

			fcntl(this->myHandler, F_SETFL, FASYNC); // FASYNC | FNDELAY  indicado en http://ulisse.elettra.trieste.it/services/doc/serial/
		}


		/////////////////////
		// Modo de control //
		/////////////////////

		// Sin control de flujo
		// No modificar el propietario del dispositivo. Habilitar recepción.

		this->myNewTIO.c_cflag =  CLOCAL | CREAD;

		this->myNewTIO.c_cflag &=  ~CRTSCTS;

		// velocidad, bits de datos, bits de parada ...

		this->myNewTIO.c_cflag |= this->myBaudRate | this->myDataBits | this->myStopBits | this->myParity;

		/////////////////////
		// Modo de entrada //
		/////////////////////

		this->myNewTIO.c_iflag = 0;

		////////////////////
		// Modo de salida //
		////////////////////

		// Nada en especial.

		this->myNewTIO.c_oflag = 0;

		////////////////
		// Modo local //
		////////////////

		// No Canónico. Sin eco, ...

		this->myNewTIO.c_lflag = 0;

		//////////////////////////////////////////////////
		// Configuración específica de modo NO CANÓNICO //
		//////////////////////////////////////////////////

		// VTIME: Tiempo mínimo (en décimas de segundo) a esperar por posibles
		// VMIN: recepciones de bytes antes de que read() devuelva el control

		if (this->myReadMode == ReadMode_SyncBlocking) {

			this->myNewTIO.c_cc[VTIME] = 5;
			this->myNewTIO.c_cc[VMIN] = 9;

		} else {

			this->myNewTIO.c_cc[VTIME] = 0;
			this->myNewTIO.c_cc[VMIN] = 0;

		}

		//////////////////////////
		// Fin de configuración //
		//////////////////////////

		// Limpiamos las colas de entrada y salida
		// descartando los bytes recibidos aún no leidos
		// y los bytes enviados al dispositivo aún no transmitidos

		tcflush(this->myHandler, TCIOFLUSH);

		// Aplicamos la nueva configuración

		tcsetattr(this->myHandler,TCSANOW,&this->myNewTIO);

	}

	// Destructor

	Serial::~Serial() {

		// Liberamos los recursos utilizados por el objeto Serial
		// 1. Restablecer los parámetros originales del dispositivo
		// 2. Cerrar el dispositivo

		if (this->myHandler > 0) {
			tcsetattr(this->myHandler,TCSANOW,&this->myOldTIO);
			close(this->myHandler);
		}

	}

	// Función de envío de cadena de caracteres.
	// Se asume que la cadena está correctamente terminada (por '\0')

	int Serial::send(const char* inBytes) {
		int theNumBytes, theBytesSent;
		theNumBytes = strlen(inBytes);
		theBytesSent = write(this->myHandler,inBytes, theNumBytes);
		cout << "Serial::send. Enviados " << theBytesSent << " bytes" << endl;
		return theBytesSent;
	}

	// Función de recepción de cadena de caracteres

	int Serial::receive(char* inBytes, int inMaxNumBytes) {

		int theBytesReceived;
		theBytesReceived = read(this->myHandler,inBytes, inMaxNumBytes);
		cout << "Serial::receive. Recibidos " << theBytesReceived << " bytes" << endl;
		return theBytesReceived;
	}

	// Constructor

	Queue::Queue(int inSize) {

		this->mySize = inSize;
		this->myBuff = (unsigned char *)malloc(this->mySize);
		this->myNumBytes = 0;
		this->myHead = 0;

	}

	// Insertar un byte en la cola

	void Queue::enqueue(unsigned char inC) {

		if (this->myNumBytes < this->mySize) {
			this->myHead++;
			if (this->myHead >= this->mySize) {
				this->myHead = 0;
			}
			this->myBuff[this->myHead] = inC;
			this->myNumBytes++;
		} else {
			cout << "Overflow!" << endl;
		}

	}

	// Extraer un byte de la cola

	unsigned char Queue::dequeue() {

		char outByte = 0;
		int theTail;

		if (this->myNumBytes > 0) {
			theTail = this->myHead - this->myNumBytes+1;
			if (theTail < 0) theTail += this->mySize;
			outByte = this->myBuff[theTail];
			this->myNumBytes--;
		}

		return outByte;

	}

	// Mostrar el contenido de la cola (opcionalmente consumiendo los bytes leidos)

	unsigned char* Queue::getFullContent(bool inDequeueBytes) {

		int i;
		int theLength = this->myNumBytes;
		unsigned char* outStr = (unsigned char*)malloc(theLength+1);

		if(outStr == NULL) return NULL;

		for (i=0;i<theLength;i++) {
			outStr[i] = this->dequeue();
			if (!inDequeueBytes) {
				this->enqueue(outStr[i]);
			}
		}
		outStr[theLength] = '\0';

		return outStr;
	}

	// Mostrar el número de bytes disponibles en la cola

	int Queue::getNumBytes() {
		return this->myNumBytes;
	}

	// Muestrar el tamaño máximo de la cola

	int Queue::getSize() {
		return this->mySize;
	}

	// Constructor

	Serial_Q::Serial_Q(const char* inDevice, int inBaudRate) :  Serial(inDevice, inBaudRate, DataBits_8, Parity_None, StopBits_1, ReadMode_SyncNonBlocking) {
		this->myQ = new Queue(256);
	}

	// Cada llamada a checkDataAndEnqueue() inserta en la cola todos los datos disponibles en el dispositivo

	bool Serial_Q::checkDataAndEnqueue() {

		int theBytesRead, i;
		char* theBuff;

		if (this->myQ->getNumBytes() < this->myQ->getSize()) {
			theBuff = (char*)malloc(32);
			theBytesRead = Serial::receive(theBuff,32);
			while ((theBytesRead > 0)&&(this->myQ->getNumBytes() < this->myQ->getSize())) {
				for (i = 0;i< theBytesRead; i++) {
					this->myQ->enqueue((unsigned char)theBuff[i]);
				}
				theBytesRead = Serial::receive(theBuff,256);
			}
		}

		return (myQ->getNumBytes() > 0);
	}

	// Extrae de la cola el siguiente byte

	unsigned char Serial_Q::dequeue() {
		return this->myQ->dequeue();
	}

	// Devuelve todo el contenido de la cola. Mantiene o no en la cola los caracteres devueltos según sen indique en inDequeueBytes

	unsigned char* Serial_Q::getFullQueueContent(bool inDequeueBytes) {
		return this->myQ->getFullContent(inDequeueBytes);
	}

	int Serial_Q::getNumBytesInQ() {
		return this->myQ->getNumBytes();
	}

}


