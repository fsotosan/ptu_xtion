#include <iostream>
#include <OpenNI.h>
#include <cmath>
#include "ros/ros.h"
#include "Serial_Q.h"

#define PI 3.14159265359
#define PAN_RESOLUTION 	185.1428
#define TILT_RESOLUTION 185.1428
#define SERIALDEVICE	"/dev/ttyUSB0"

#define PAN		0
#define TILT 	1

#define ABSOLUTE 0
#define RELATIVE 1

#define OFFSET_CAMARA_EJE_TILT_MM 70

enum estado {
	IDLE = 0,
	WAIT_COMMAND_CONF = 1,
	WAIT_POS_PAN = 2,
	WAIT_POS_TILT = 3
};

estado STATUS = IDLE;

using namespace std;

openni::Status theStatus;
openni::Device theDevice;
openni::VideoStream theDepth;

//ros::NodeHandle* myNodeHandle = 0;

Serial::Serial_Q *Ptu;

class Pixel3D {

public:

  int X;
  int Y;
  unsigned short int Z;

  void print() {
	  printf("Pixel3D (%d,%d,%d)\n",X,Y,Z);
  }

};

class Point {

public:

	float X;
	float Y;
	float Z;

	Point() {
		this->X = 0.0;
		this->Y = 0.0;
		this->Z = 0.0;
	}

	Point(float inX, float inY, float inZ) {
		this->X = inX;
		this->Y = inY;
		this->Z = inZ;
	}

	void print() {
		printf("Point (%g,%g,%g)\n",X,Y,Z);
	}

};

class Vector {

public:

	float X;
	float Y;
	float Z;

	Vector(Point p1, Point p2) {

		this->X = p2.X - p1.X;
		this->Y = p2.Y - p1.Y;
		this->Z = p2.Z - p1.Z;

	}

	float getModulo() {

		return sqrt(this->X*this->X + this->Y*this->Y+this->Z*this->Z);

	}

	float getTilt() {
		return getTiltConsideringOffsetY(0);
	}

	float getTiltConsideringOffsetY(float inOffsetY) {

		float theModulo = this->getModulo();
		float theCosPan = cos(getPan());
		if ((theModulo > 0)&&(theCosPan != 0)) {
			return asin((sqrt(theModulo*theModulo - inOffsetY*inOffsetY)*this->Y - inOffsetY*this->Z / theCosPan)/(theModulo*theModulo));
		} else {
			return 0.0;
		}

	}

	float getPan() {

		if (this->Z > 0) {
			return atan2(this->X, this->Z);
		} else {
			return 0.0;
		}

	}

	void print() {
		printf("Point (%g,%g,%g)\n",this->X,this->Y,this->Z);
	}

	void printPanTilt() {
		printf("MODULO: %g, PAN: %g, TILT: %g \n",this->getModulo(),this->getPan(),this->getTilt());
	}

	void printPanTiltDeg() {
		printf("MODULO: %d, PAN: %d, TILT: %d \n",(int)this->getModulo(),(int)(this->getPan()*180/PI),(int)(this->getTilt()*180/PI));
	}

};


// Obtiene las coordenadas del punto con menor profundidad
// Las coordenadas X e Y identifican un número de pixel dentro del cuadro
// La coordenada Z es la profundidad
openni::Status calculaPuntoMasCercano(Pixel3D* closestPoint, openni::VideoFrameRef* rawFrame) {

	openni::DepthPixel* pDepth = (openni::DepthPixel*)rawFrame->getData();
	bool found = false;
	closestPoint->Z = 0xffff;
	int width = rawFrame->getWidth();
	int height = rawFrame->getHeight();

	for (int y = 0; y < height; ++y)
		for (int x = 0; x < width; ++x, ++pDepth)
		{
			if (*pDepth < closestPoint->Z && *pDepth != 0)
			{
				closestPoint->X = x;
				closestPoint->Y = y;
				closestPoint->Z = *pDepth;
				found = true;
			}
		}

	if (!found)
	{
		return openni::STATUS_ERROR;
	}

	return openni::STATUS_OK;
}

bool getPosCommand(float inDeg, int inTargetJoint, int inMode, char* outCommand) {

	int thePosVal;
	char theParam = 'P';
	char theMode = ABSOLUTE;
	bool ok = true;

	switch(inTargetJoint) {
		case PAN:
			thePosVal = (int) (inDeg * 3600 / PAN_RESOLUTION);
			theParam = 'P';
			break;
		case TILT:
			thePosVal = (int) (inDeg * 3600 / TILT_RESOLUTION);
			theParam = 'T';
			break;
		default: 	// inesperado
			ok = false;
	}

	switch (inMode) {
		case ABSOLUTE:	// PP o TP
			theMode = 'P';
			break;
		case RELATIVE:	// PO o TO
			theMode = 'O';
			break;
		default: 	// inesperado
			ok = false;
	}

	if (ok) {
		sprintf(outCommand,"%c%c%d ",theParam,theMode,thePosVal);
	}

	printf("%s\n",outCommand);

	return ok;

}

bool extractInt(const string inStr, int* outNum) {

	stringstream ss(inStr);
	string tmp;
	ss >> tmp >> *outNum;
	return true;

}

void processPtuComm() {

	string theResp;
	std::size_t theFound;

	if (Ptu->checkDataAndEnqueue()) {

		theResp.assign((char *)Ptu->getFullQueueContent(true));

		// Un signo de exclamación indica que se ha producido un error

		theFound = theResp.find("!");
		if (theFound != string::npos) {
			STATUS = IDLE;
			printf("PTU-46 ha devuelto un error: %s",theResp.c_str());
			return;
		}

		switch (STATUS) {

			case IDLE:

				// Recepción inesperada
				// Mostramos datos recibidos

				ROS_ERROR("PTU-46 ha enviado un dato inesperado: %s",theResp.c_str());

				break;

			case WAIT_COMMAND_CONF:

				theFound = theResp.find("*");
				if (theFound != string::npos) {
					printf("PTU-46 ha confirmado la última orden: %s",theResp.c_str());
					STATUS = IDLE;
				}

				break;

			case WAIT_POS_PAN:

				break;

			case WAIT_POS_TILT:

				// Comprobamos que el comando se ha admitido
				// Y eliminamos el asterisco de confirmación

				int thePos;

				theFound = theResp.find("*");
				if (theFound != string::npos) {
					theResp.erase(0,theFound+1);
					STATUS = IDLE;
				}

				// Leemos el dato devuelto

				if (extractInt(theResp,&thePos)) {

					float theDeg;

					if (STATUS == WAIT_POS_PAN) {
						theDeg = (float)thePos * PAN_RESOLUTION / 3600;
						//thePtuJointState.position[0] = theDeg;
					} else if (STATUS == WAIT_POS_TILT) {
						theDeg = (float)thePos * TILT_RESOLUTION / 3600;
						//thePtuJointState.position[1] = theDeg;
					}

				}

				STATUS = IDLE;
				break;

			default:

				STATUS = IDLE;
				break;

		}

	}

	return;

}

void movePtu(float inPanDeg, float inTiltDeg) {

	char thePanCommand[16];
	char theTiltCommand[16];
	bool theBuildCommandOk = false;

	theBuildCommandOk = getPosCommand(inPanDeg,PAN,RELATIVE,thePanCommand);


	theBuildCommandOk = theBuildCommandOk && getPosCommand(inTiltDeg,TILT,RELATIVE,theTiltCommand);

	if (theBuildCommandOk) {
		if (Ptu->send(thePanCommand) > 0) {
			STATUS = WAIT_COMMAND_CONF;
			processPtuComm();
			//Ptu->send("A ");
		}
		if (Ptu->send(theTiltCommand) > 0) {
			STATUS = WAIT_COMMAND_CONF;
			processPtuComm();
			//Ptu->send("A ");
		}
	} else {
		// Error construyendo mensaje. no enviar nada
		// ...
	}

}

void ceroPtu() {

	Ptu->send("I ");	// Modo inmediato
	usleep(200000);
	processPtuComm();
	Ptu->send("FT ");	// Respuestas escuetas
	usleep(200000);
	processPtuComm();
	Ptu->send("PP0 ");	// Posición PAN 0
	usleep(200000);
	processPtuComm();
	Ptu->send("A ");	// Espera alcanzar las posiciones indicadas
	usleep(200000);
	processPtuComm();
	Ptu->send("TP-300 ");	// Posición TILT max
	usleep(200000);
	processPtuComm();
	Ptu->send("A ");	// Espera alcanzar las posiciones indicadas
	usleep(200000);
	processPtuComm();
	Ptu->send("TP600 ");	// Posición TILT max
	usleep(200000);
	processPtuComm();
	Ptu->send("A ");	// Espera alcanzar las posiciones indicadas
	usleep(200000);
	processPtuComm();

}

int main(int argc, char ** argv) {

	cout << "Inicializando PTU ..." << endl;

	// Init PTU-46

	Ptu = new Serial::Serial_Q(SERIALDEVICE, B9600);

	ceroPtu();

	usleep(500000);

	movePtu(0,-20);

	cout << "Inicializando OpenNI ..." << endl;
	theStatus = openni::OpenNI::initialize();

	if (theStatus != openni::STATUS_OK) {
		printf("Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 1;
	}

	cout << "Iniciando sensor de profundidad ...";

	openni::Array<openni::DeviceInfo> theDevices;
	openni::OpenNI::enumerateDevices(&theDevices);

	// Busca dispositivos compatibles
	// Inicializa la variable theDevice con cada uno
	// E imprime por pantalla su numero de serie

	for (int i = 0; i != theDevices.getSize(); ++i) {
		const openni::DeviceInfo& theDeviceInfo = theDevices[i];
		string uri = theDeviceInfo.getUri();
		theDevice.open(uri.c_str());
		char theSerialNumber[1024];
		theDevice.getProperty(ONI_DEVICE_PROPERTY_SERIAL_NUMBER, &theSerialNumber);
		cout << "Device " << i << ". Serial Number: " << theSerialNumber << endl;
	}

	// Crea un stream de cuadros a partir del dispositivo inicializado anteriormente
	// Se indica que el dispositivo es de tipo profundidad

	theStatus = theDepth.create(theDevice, openni::SENSOR_DEPTH);

	if (theStatus == openni::STATUS_OK) {
		theStatus = theDepth.start();
		if (theStatus != openni::STATUS_OK) {
			printf("Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			theDepth.destroy();
		}
	} else {
		printf("Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
	}

	if (!theDepth.isValid()) {
		printf("No valid streams. Exiting\n");
		openni::OpenNI::shutdown();
		return 2;
	}

	openni::VideoFrameRef theRawFrame;
	Pixel3D theClosestPoint;
	Point theRealPoint;
	Point theOrigin;

	float thePanDeg, theTiltDeg, theDist;

	// Por cada cuadro recibido se obtiene el punto más cercano y se muestra por pantalla
	while(theDepth.isValid()){

		openni::Status rc = theDepth.readFrame(&theRawFrame);
		if (rc != openni::STATUS_OK) {
			printf("readFrame failed\n%s\n", openni::OpenNI::getExtendedError());
		} else {
			calculaPuntoMasCercano(&theClosestPoint,&theRawFrame);
			//theClosestPoint.print();
			openni::CoordinateConverter::convertDepthToWorld(theDepth,theClosestPoint.X,theClosestPoint.Y,theClosestPoint.Z,&theRealPoint.X, &theRealPoint.Y, &theRealPoint.Z);

			// Adaptamos las coordenadas para considerar el desajuste
			// entre las coordenadas de la cámara y las de la base pan-tilt

			theRealPoint.Y += OFFSET_CAMARA_EJE_TILT_MM / 1000;

			theRealPoint.print();
			Vector theV(theOrigin, theRealPoint);

			thePanDeg = theV.getPan()*180/PI;
			//theTiltDeg = theV.getTiltConsideringOffsetY(OFFSET_CAMARA_EJE_TILT_MM / 1000)*180/PI - 90;
			theTiltDeg = theV.getTiltConsideringOffsetY(OFFSET_CAMARA_EJE_TILT_MM / 1000)*180/PI;
			theDist = theV.getModulo();

			//if (theDist > 600) {
			if ((abs(thePanDeg) > 2)||(abs(theTiltDeg) > 2)) {
				movePtu(thePanDeg, theTiltDeg);
			}
			//}

			theV.printPanTiltDeg();

			usleep(1500000);
		}

	}

	theDepth.stop();
	theDepth.destroy();

	cout << "Terminando" << endl;
	openni::OpenNI::shutdown();

	return 0;

}





