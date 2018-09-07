#ifndef _PYLON_PROXY_H_
#define _PYLON_PROXY_H_

#ifdef MATLAB
#include "mex.h"
#endif

/*
 *  definition of camera info array
 *  uint64_t Info
 *	  1   x				   0
 *	  2   y				   1
 *	  3   size in bytes	   2
 *	  4   exposure min		3
 *	  5   exposure max		4
 *	  6   exposure value	  5
 *	  7   gain min			6
 *	  8   gain max			7
 *	  9   gain value		  8
 *	 10   black level min	 9
 *	 11   black level max	 10
 *	 12   black level value   11
 *
 *	  matlab index			c index
 *
 */

#include <pylon/PylonIncludes.h>

#if defined( USE_1394 )
// Settings to use  Basler 1394 cameras
#include <pylon/1394/Basler1394Camera.h>
typedef Pylon::CBasler1394Camera Camera_t;
using namespace Basler_IIDC1394CameraParams;
using namespace Basler_IIDC1394StreamParams;
#elif defined ( USE_GIGE )
// Settings to use Basler GigE cameras
#include <pylon/gige/BaslerGigECamera.h>
typedef Pylon::CBaslerGigECamera Camera_t;
using namespace Basler_GigECameraParams;
using namespace Basler_GigEStreamParams;
#else
#error Camera type is not specified. For example, define USE_GIGE for using GigE cameras
#endif

struct MyContext
{
  int number;
};

class PylonProxy
{
public:
	PylonProxy();
	~PylonProxy();

	void getInfoArray(uint64_t* array);
	void setInfoArray(uint64_t* array);
	uint64_t getWidth();
	uint64_t getHeight();
	
	void startContinuous(int16_t *buffers, int numBuffers, size_t buffer_size);
	void stopContinuous();
	int getFrame();
	/* get a single image (number in the ring buffer) during a continuous
	* acquisition, no real data transfer here,
	* we just ask for the current image in the ring buffer*/
	
	
	void acquire(int16_t *buffer);
	/* acquire a single image (trigger a camera shot and transfer the data) */
	
	
	void requeue(int number);
	/* when we are done with one image in the ring buffer we have to
	* re-register that spot so the queue manager can fill it with new data */

	void setMessageFcn(void (*FcnPtr)(const char*));
	bool isActive();
	
private:
	void initPylon();
	bool pylonIsActive;
	
	Camera_t *Camera;
	Camera_t::StreamGrabber_t *StreamGrabber;
	Pylon::StreamBufferHandle *hBuffer;
	int numRegBuffers;
	//stores image size in bytes
	int imageSize;
	size_t bufferSize;
	int imageWidth, imageHeight;
	void (*messageFcn)(const char*);
};

#endif
