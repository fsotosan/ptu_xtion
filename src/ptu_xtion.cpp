#include <iostream>
#include <OpenNI.h>
#include <cmath>

#define PI 3.14159265359

using namespace std;

openni::Status theStatus;
openni::Device theDevice;
openni::VideoStream theDepth;

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

		float theModulo = this->getModulo();
		if (theModulo > 0) {
			return acos(this->Y/theModulo);
		} else {
			return 0.0;
		}

	}

	float getPan() {

		float sinTilt = sin(this->getTilt());
		float theModulo = this->getModulo();
		if ((sinTilt > 0)&&(theModulo > 0)) {
			return asin(this->X / (theModulo*sinTilt));
		} else {
			return 0.0;
		}
	}

	void print() {
		printf("Point (%g,%g,%g)\n",this->X,this->Y,this->Z);
	}

	void printPanTilt() {
		printf("MODULO: %g, PAN: %g, TILT: %g                   \n",this->getModulo(),this->getPan(),this->getTilt());
	}

	void printPanTiltDeg() {
		printf("MODULO: %d, PAN: %d, TILT: %d                   \n",(int)this->getModulo(),(int)(this->getPan()*180/PI),(int)(this->getTilt()*180/PI));
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

int main(int argc, char ** argv) {

	cout << "Inicializando ...";
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

	// Por cada cuadro recibido se obtiene el punto más cercano y se muestra por pantalla
	while(theDepth.isValid()){

		openni::Status rc = theDepth.readFrame(&theRawFrame);
		if (rc != openni::STATUS_OK) {
			printf("readFrame failed\n%s\n", openni::OpenNI::getExtendedError());
		} else {
			calculaPuntoMasCercano(&theClosestPoint,&theRawFrame);
			//theClosestPoint.print();
			openni::CoordinateConverter::convertDepthToWorld(theDepth,theClosestPoint.X,theClosestPoint.Y,theClosestPoint.Z,&theRealPoint.X, &theRealPoint.Y, &theRealPoint.Z);
			theRealPoint.print();
			Vector theV(theOrigin, theRealPoint);
			theV.printPanTiltDeg();
		}

	}

	theDepth.stop();
	theDepth.destroy();

	cout << "Terminando" << endl;
	openni::OpenNI::shutdown();

	return 0;

}





