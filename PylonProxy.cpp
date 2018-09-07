#include "PylonProxy.h"

using namespace Pylon;
using namespace std;

void stdcerr(const char* msg)
{
	std::cerr << msg << std::flush;
}

PylonProxy::PylonProxy()
{
	messageFcn = *stdcerr;
	initPylon();
}

PylonProxy::~PylonProxy()
{
	if(Camera)
	{
		// buffers should be deregistered already
		try{			
			if (numRegBuffers > 0)
			  stopContinuous();
			  for ( int i = 0; i < numRegBuffers; ++i ) {
				StreamGrabber->DeregisterBuffer(hBuffer[i]);
			  }
		}
		catch (GenICam::GenericException &e)
		{
			(*messageFcn)("An exception occurred!");
			(*messageFcn)(e.GetDescription());
		}  
		StreamGrabber->Close();
		Camera->Close();
	}

	//get rid of Pylon API
	if (pylonIsActive)
		Pylon::PylonTerminate();
}

void PylonProxy::initPylon()
{
 //initialize Pylon API
	Pylon::PylonInitialize();
	Camera = NULL;
	StreamGrabber = NULL;
	hBuffer = NULL;

	try
	{
		// Get the transport layer factory
		CTlFactory& TlFactory = CTlFactory::GetInstance();

		// Create the transport layer object needed to enumerate or
		// create a camera object of type Camera_t::DeviceClass()
		ITransportLayer *pTl = TlFactory.CreateTl(Camera_t::DeviceClass());

		// Exit the application if the specific transport layer is not available
		if (! pTl)
		{
			(*messageFcn)("Failed to create transport layer!");
			Pylon::PylonTerminate();
			pylonIsActive = FALSE;
			return;
		}

		// Get all attached cameras and return if no camera is found
		DeviceInfoList_t devices;
		if (0 == pTl->EnumerateDevices(devices))
		{
			(*messageFcn)("No camera present!");
			Pylon::PylonTerminate();
			pylonIsActive = FALSE;
			return;
		}

		// Create the camera object of the first available camera.
		// The camera object is used to set and get all available
		// camera features.
		Camera = new Camera_t(pTl->CreateDevice(devices[0]));
		Camera->Open();

		// Get the first stream grabber object of the selected camera
		StreamGrabber = new Camera_t::StreamGrabber_t(Camera->GetStreamGrabber(0));

		// Open the stream grabber
		StreamGrabber->Open();

		// Set the image format and AOI
		Camera->PixelFormat.SetValue(PixelFormat_Mono16);
		Camera->OffsetX.SetValue(0);
		Camera->OffsetY.SetValue(0);
		Camera->Width.SetValue(Camera->Width.GetMax());
		Camera->Height.SetValue(Camera->Height.GetMax());
		imageWidth  = (int)Camera->Width();
		imageHeight = (int)Camera->Height();
		imageSize   = imageHeight*imageWidth;
		bufferSize  = (size_t)(Camera->PayloadSize());

		Camera->PixelFormat.SetValue(PixelFormat_Mono16);
		Camera->ExposureMode = ExposureMode_Timed;
		Camera->ExposureTimeRaw = 100;

		pylonIsActive = TRUE;
	}
	catch (GenICam::GenericException &e)
	{
		// Error handling
		Camera = NULL;
		StreamGrabber = NULL;
		Pylon::PylonTerminate();
		pylonIsActive = FALSE;
		(*messageFcn)("An exception occurred!");
		(*messageFcn)(e.GetDescription());
		return;
	}   
}

void PylonProxy::startContinuous(int16_t *buffers, int numBuffers, size_t buffer_size)
{
	try{
		if (buffer_size != (int)Camera->PayloadSize())
		{
			(*messageFcn)("buffer sizes dont match, wont start acquisition!");
			return;
		}
		
		// Parameterize the stream grabber
		//(*messageFcn)(sprintf("got %d buffers, starting at %d, size %d bytes each\n", numBuffers, buffers, buffer_size));
		Camera->TriggerSelector.SetValue( TriggerSelector_AcquisitionStart );
		Camera->TriggerMode.SetValue( TriggerMode_Off );
		Camera->AcquisitionMode.SetValue( AcquisitionMode_Continuous );

		StreamGrabber->MaxBufferSize = bufferSize;
		StreamGrabber->MaxNumBuffer = numBuffers;
		StreamGrabber->PrepareGrab();
		// Allocate and register image buffers, put them into the
		// grabber's input queue
		(*messageFcn)("registering buffers ");
		hBuffer = new StreamBufferHandle[numBuffers];
		MyContext *context = new MyContext[numBuffers];
		for ( int i = 0; i < numBuffers; ++i )
		{
			(*messageFcn)(".");
			hBuffer[i] = StreamGrabber->RegisterBuffer( (void*)( ((int16_t*)buffers) + i*imageSize ), bufferSize);
			context[i].number = i;
			StreamGrabber->QueueBuffer( hBuffer[i], &context[i] );
			numRegBuffers = i+1;
		}
		(*messageFcn)("\nstart!\n");
		// Start image acquisition
		Camera->AcquisitionStart.Execute();
	}
	catch (GenICam::GenericException &e)
	{
		(*messageFcn)("An exception occurred during start!");
		(*messageFcn)(e.GetDescription());
		return;
	}
}

void PylonProxy::stopContinuous()
{
	try{
		(*messageFcn)("stop\n");
		Camera->AcquisitionStop.Execute();

		// Flush the input queue, grabbing may have failed
		StreamGrabber->CancelGrab();

		// Consume all items from the output queue
		GrabResult Result;
		while ( StreamGrabber->GetWaitObject().Wait(0) )
			StreamGrabber->RetrieveResult( Result );

		// Deregister buffers
		for ( int i = 0; i < numRegBuffers; ++i )
			StreamGrabber->DeregisterBuffer(hBuffer[i]);

		numRegBuffers = 0;

		StreamGrabber->FinishGrab();
	}
	catch (GenICam::GenericException &e)
	{
		(*messageFcn)("An exception occurred during stop!");
		(*messageFcn)(e.GetDescription());
		return;
	}
}

int PylonProxy::getFrame()
/* get a single image (number in the ring buffer) during a continuous
 * acquisition, no real data transfer here,
 * we just ask for the current image in the buffer*/
{
	if (StreamGrabber->GetWaitObject().Wait(1000))
	{
		// Get the grab result from the grabber's result queue
		GrabResult Result;
		StreamGrabber->RetrieveResult(Result);

		if (Result.Succeeded())
		{
			imageWidth = Result.GetSizeX();
			imageHeight = Result.GetSizeY();
			return ((MyContext*)Result.Context())->number;
		}
		else
		{
			(*messageFcn)("No image acquired!");
			(*messageFcn)(Result.GetErrorDescription());
			//even if the image was not transferred, we should re queue the buffer
			requeue(((MyContext*)Result.Context())->number);
			return -1;
		}
	}
	else
	{
		(*messageFcn)("Timeout occurred!");
		return -1;
	}
}

void PylonProxy::acquire(int16_t *buffer)
/* acquire a single image (trigger a camera shot and transfer the data) */
{
	try{
		if ((!Camera) || !(Camera->IsOpen()))
		{
			(*messageFcn)("no camera connection available or camera not active, cannot acquire image");
			return;
		}
		if(!(StreamGrabber))
		{
			(*messageFcn)("no Stream Grabber initialized, cannot acquire image");
			return;
		}
		
		//Disable acquisition start trigger if available
		GenApi::IEnumEntry* acquisitionStart = Camera->TriggerSelector.GetEntry( TriggerSelector_AcquisitionStart);
		if ( acquisitionStart && GenApi::IsAvailable( acquisitionStart))
		{
			Camera->TriggerSelector.SetValue( TriggerSelector_AcquisitionStart);
			Camera->TriggerMode.SetValue( TriggerMode_Off);
		}

		//Disable frame start trigger if available
		GenApi::IEnumEntry* frameStart = Camera->TriggerSelector.GetEntry( TriggerSelector_FrameStart);
		if ( frameStart && GenApi::IsAvailable( frameStart))
		{
			Camera->TriggerSelector.SetValue( TriggerSelector_FrameStart);
			Camera->TriggerMode.SetValue( TriggerMode_Off);
		}
		if (!hBuffer)

		Camera->AcquisitionMode.SetValue(AcquisitionMode_SingleFrame);

		// Create an image buffer
		StreamGrabber->MaxBufferSize = bufferSize;
		StreamGrabber->MaxNumBuffer = 1;

		// Allocate all resources for grabbing. Critical parameters like image
		// size now must not be changed until FinishGrab() is called.
		StreamGrabber->PrepareGrab();
		//mexPrintf("reg ing buffer\n");
		if (!hBuffer)
			hBuffer = new StreamBufferHandle;
		*hBuffer = StreamGrabber->RegisterBuffer(buffer, bufferSize);
		numRegBuffers = 1;
		//mexPrintf("q ing buffer\n");
		StreamGrabber->QueueBuffer(*hBuffer);
		
		//mexPrintf("acq start\n");
		Camera->AcquisitionStart.Execute();

		// Wait for the grabbed image with a timeout of 3 seconds
		if (StreamGrabber->GetWaitObject().Wait(3000))
		{
			// Get the grab result from the grabber's result queue
			GrabResult Result;
			//mexPrintf("retrieving buffer\n");
			StreamGrabber->RetrieveResult(Result);

			if (Result.Succeeded())
			{
				//mexPrintf("got buffer\n");
				imageWidth = Result.GetSizeX();
				imageHeight = Result.GetSizeY();
			}
			else
			{
				(*messageFcn)("No image acquired!");
				(*messageFcn)(Result.GetErrorDescription());
			}

			// Free all resources used for grabbing
			StreamGrabber->FinishGrab();
		}
		else
		{
			(*messageFcn)("Timeout occurred!");
			StreamGrabber->CancelGrab();
			// Get all buffers back
			//for (GrabResult r; StreamGrabber->RetrieveResult(r););
		}
		(*messageFcn)("deregistering single buffer\n");
		StreamGrabber->DeregisterBuffer(*hBuffer);
		numRegBuffers = 0;
	}
	catch (GenICam::GenericException &e)
	{
		(*messageFcn)("An exception occurred!");
		(*messageFcn)(e.GetDescription());
		return;
	}
}

void PylonProxy::requeue(int number)
/* when we are done with one image in the ring buffer we have to
 * re-register that spot so the queue manager can fill it with new data */
{
	if (Camera && Camera->IsOpen())
	{
		MyContext *context = new MyContext;
		context->number = number;
		StreamGrabber->QueueBuffer( (hBuffer[number]), context);
	}
}

void PylonProxy::setMessageFcn(void (*FcnPtr)(const char*))
{
	messageFcn = FcnPtr;
	(*FcnPtr)("test...!");
}

void PylonProxy::getInfoArray(uint64_t* array)
{
	if (Camera && Camera->IsOpen())
	{
		array[0] = imageWidth;
		array[1] = imageHeight;
		array[2] = Camera->PayloadSize();
		array[3] = Camera->ExposureTimeRaw.GetMin();
		array[4] = Camera->ExposureTimeRaw.GetMax();
		array[5] = Camera->ExposureTimeRaw.GetValue();
		array[6] = Camera->GainRaw.GetMin();
		array[7] = Camera->GainRaw.GetMax();
		array[8] = Camera->GainRaw.GetValue();
		array[9] = Camera->BlackLevelRaw.GetMin();
		array[10] = Camera->BlackLevelRaw.GetMax();
		array[11] = Camera->BlackLevelRaw.GetValue();
	}
}

void PylonProxy::setInfoArray(uint64_t* array)
{
	if (Camera && Camera->IsOpen())
	{
		Camera->ExposureTimeRaw.SetValue(array[5]);
		Camera->GainRaw.SetValue(array[8]);
		Camera->BlackLevelRaw.SetValue(array[11]);			
	}
}


uint64_t PylonProxy::getWidth()
{
	return imageWidth;
}

uint64_t PylonProxy::getHeight()
{
	return imageHeight;
}

bool PylonProxy::isActive()
{
	return pylonIsActive;
}
