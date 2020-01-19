

#include <QtDebug>
#include <QFuture>
#include <QtConcurrent/QtConcurrent>  //Location for Qt 5.x.x
#include <QThread>
#include "usbcomm.h"
#include <QImage>
#include  <QVector>
//-------------------------------------------------------------------------------
//VID/PID Definitions: These must match the VID/PID in the USB device descriptor,
//in the device firmware, as well as in the driver package .inf file,
//in order for this application to find and connect to the USB device.
#define USB_VID_TO_FIND 0x04D8
#define USB_PID_TO_FIND 0x0053
//-------------------------------------------------------------------------------


//Wide scope variables
libusb_device_handle* libUSBDeviceHandle;
bool libusbInitialized = false;
QMutex* mutex;


//So we can gain access to the static/private msleep() function.
class Sleeper: public QThread {
public:
    static void msleep(int ms)
    {
        QThread::msleep(ms);
    }
};


//Constructor function.  Executes when we initialize a new instance of this class.
Usbcomm::Usbcomm(QObject *parent) : QObject(parent)
{
    //Initialize state tracking and other variables.
    libusbInitialized = false;
    isConnected = false;
    libUSBDeviceHandle = NULL;
    mutex = new QMutex;

}


//Destructor function
Usbcomm::~Usbcomm() {
    //Don't need to do much here.  The important
    //shutdown code is located in the PollUSBConnection()
    //thread.  Make sure to set the *pThreadAbortFlag = true
    //so as to gracefully shutdown the thread and libusb
    //before exiting the application.
}




//This function is intended to be executed as a separate thread.  It periodically
//checks the system for the presence of the USB device with matching VID and PID.
//If a device is found, it automatically connects up to it and notifies the application.
//Similarly, this function can also detect detach events and will also alert the application.
void Usbcomm::PollUSBConnection(volatile bool* pThreadAbortFlag)
{
    int status;
    libusb_device** list;
    ssize_t numberOfDevices;
    libusb_device_descriptor deviceDescriptor;
    ssize_t i;
    bool deviceFound;
    Sleeper SleepThread;
    libusb_device_handle* localLibUSBDeviceHandle = NULL;
    bool oldIsConnected = true;


    //Infinite polling loop, until the calling thread tries to signal abort.
    while(1)
    {
        //Check if the calling thread it trying to tell us to abort/quit this thread
        if(*pThreadAbortFlag == true)
        {
            //Clean things up and exit
            if(localLibUSBDeviceHandle != NULL)
            {
                //libusb_release_interface(); //Normally supposed to call this prior to calling lubusb_close(), but since
                //the USB device is presumably detached right now, this is rather extraneous and should error out with LIBUSB_ERROR_NO_DEVICE
                libusb_close(localLibUSBDeviceHandle);   //
                localLibUSBDeviceHandle = NULL;
            }
            mutex->lock();
            libUSBDeviceHandle = localLibUSBDeviceHandle;
            mutex->unlock();
            isConnected = false;
            emit(comm_update_connection(isConnected));
            return;
        }

        //Check if the libusb library has been initialized yet.
        if(libusbInitialized == false)
        {
            //The library wasn't already initialized.  Try to initialize it now.
            status = libusb_init(NULL); //Must be called prior to calling any other libusb functions
            if(status == 0)
            {
                //The init function worked.  Keep track of this so we know it is safe to call other libusb functions.
                libusbInitialized = true;
            }
            else
            {
                //Couldn't open the libusb library correctly for some unknown reason.
                //Bug out for now, will try again next time the while loop executes.
                //Sleep for awhile in the while(1) loop, so we don't monopolize the CPU.
                SleepThread.msleep(50);
                continue;
            }
        }//if(libusbInitialized == false)


        //Check if the device was last known to be not connected, if so, check
        //and see if it has recently become connected.
        if(isConnected == false)
        {
            //Check to see if there is a "dead" handle (to a device that has subsequently been
            //unplugged) still consuming resources.  If so, destroy it before trying to make a new one.
            if(localLibUSBDeviceHandle != NULL)
            {
                //libusb_release_interface(); //Normally supposed to call this prior to calling lubusb_close(), but since
                //the USB device is presumably detached right now, this is rather extraneous and should error out with LIBUSB_ERROR_NO_DEVICE
                libusb_close(localLibUSBDeviceHandle);   //
                localLibUSBDeviceHandle = NULL;
            }

            //Try to find if a USB device of matching VID/PID is attached to the system.
            //First, we need to get a list of devices attached to the system.
            numberOfDevices = libusb_get_device_list(NULL, &list);
            //Now search through every item in the list, to see if any of them match
            //our device with matching VID/PID
            for(i = 0; i < numberOfDevices; i++)
            {
                //Fetch the device descriptor for a device in the list
                libusb_get_device_descriptor(list[i], &deviceDescriptor);
                //Check if the VID/PID in the device descriptor match the one we are looking for
                if((deviceDescriptor.idVendor == USB_VID_TO_FIND) && (deviceDescriptor.idProduct == USB_PID_TO_FIND))
                {
                    //Now try to open the device.
                    if(libusb_open(list[i], &localLibUSBDeviceHandle) == 0)
                    {
                        //We successfully opened the device! The LibUSBDeviceHandle
                        //will have a valid non-null value.
                        break;  //Exit for() loop.  We found the device of interest.
                    }
                    else
                    {
                        //Something went wrong if we get to here...
                        libusb_close(localLibUSBDeviceHandle);
                        localLibUSBDeviceHandle = NULL;
                    }
                }
            }

            //Free up the resources associated with the device list, now that we are done using it
            libusb_free_device_list(list, 1);

            //Check if we successfully opened the device.
            if(localLibUSBDeviceHandle != NULL)
            {
                //A device with matching VID/PID was found.  Now try to "claim"
                //the USB interface #0 on the device (should normally be interface 0
                //unless we are connecting to a composite device with multiple interfaces).
                status = libusb_claim_interface(localLibUSBDeviceHandle, 0);
                if(status == 0)
                {
                    //We found and connected (to interface 0) successfully on the first device with matching VID/PID
                    isConnected = true;
                    oldIsConnected = false; //intentionally opposite of isConnected, so as to send alert to app
                }
            }
            else
            {
                //No device with matching VID/PID was attached to the system
                isConnected = false;
            }
        }//if(isConnected == false)
        else
        {
            //The device has previously been connected.  Re-check the connection
            //status to make sure the user hasn't recently unplugged the device.
            //First, we need to get a list of devices attached to the system.
            numberOfDevices = libusb_get_device_list(NULL, &list);
            //Now search through every item in the list, to see if any of them match
            //our device with matching VID/PID
            deviceFound = false;
            for(i = 0; i < numberOfDevices; i++)
            {
                //Fetch the device descriptor for a device in the list
                libusb_get_device_descriptor(list[i], &deviceDescriptor);
                //Check if the VID/PID in the device descriptor match the one we are looking for
                if((deviceDescriptor.idVendor == 0x04D8) && (deviceDescriptor.idProduct == 0x0053))
                {
                    deviceFound = true;
                    break;
                }
            }

            //Free up the resources associated with the device list, now that we are done using it
            libusb_free_device_list(list, 1);

            //Verify if we found the device or not.
            if(deviceFound == false)
            {
                //The device must have recently been unplugged.
                isConnected = false;
            }
        }

        //Update the broad scope libUSBDeviceHandle to match our local, more up to date copy.
        //We are using a mutex to make sure other thread(s) are not currently using the
        //libUSBDeviceHandle while we are writing to it.  Note: The libUSBDeviceHandle is
        //a pointer, and in theory, assignment of a pointer should normally be an atomic
        //opertion on an x86 CPU.  However, since this is a cross platform application, it is
        //probably best not to make this assumption.  Some CPU types, especially embedded
        //CPUs, can have an ALU data width that is shorter than the address bus width, whereby
        //pointer assignments typically would not be atomic.  Therefore, to be safe, we will
        //protect usage/access to the libUSBDeviceHandle variable with a mutex.
        mutex->lock();
        libUSBDeviceHandle = localLibUSBDeviceHandle;
        mutex->unlock();

        //If the connection status has recently changed, alert the application
        //of the new status (so it can execute callbacks/do other things,
        //such as disabling buttons, etc., based on USB connection status).
        if(oldIsConnected != isConnected)
        {
            oldIsConnected = isConnected;   //Update for next time we check
            //Let the application know of the recently changed status
            emit(comm_update_connection(isConnected));
        }

        //Sleep for awhile in the while() loop, so we don't monopolize the CPU.
        SleepThread.msleep(50);
    }//while(1)
}//void Usbcomm::PollUSBConnection(void)




#define DISPLAY_FRAME_WIDTH    240
#define DISPLAY_BYTES_PER_PIXEL 2
#define DISPLAY_LINE_LENGTH (DISPLAY_FRAME_WIDTH * DISPLAY_BYTES_PER_PIXEL)
#define BYTES_PER_USB_TRANSFER 1024
#define USB_TRANSFER_HEADER_LENGTH 3
#define USB_TRANSFER_BYTES_PAYLOAD (BYTES_PER_USB_TRANSFER - USB_TRANSFER_HEADER_LENGTH)
void Usbcomm::usb_send_frame_slot(QImage image_frame)
{
    int status;

    int NumBytesSent;
    QVector<uint8_t> data_v;
    int total_frame_bytes = image_frame.height() * image_frame.bytesPerLine(); //bytes in a frame
    int number_of_transfer = static_cast<int>(ceil(static_cast<double>(total_frame_bytes) / USB_TRANSFER_BYTES_PAYLOAD)); //number of payload tranfers
    int total_bytes_for_header = (number_of_transfer * USB_TRANSFER_HEADER_LENGTH);
    int total_transfer_bytes = total_frame_bytes + total_bytes_for_header; //add header length to total transfers
    data_v.resize(total_transfer_bytes); //transfer protocol will contain 2 byte header on each usb tranfer (similar to UVC)
    static bool frame_toggle = false;


    int i = 0;
    int j = 0;
    uint8_t frame_index = 0;
    for (; i < total_transfer_bytes; i = i + BYTES_PER_USB_TRANSFER, j = j +USB_TRANSFER_BYTES_PAYLOAD )
    {
        data_v[i] = 0x03;
        data_v[i+1] = frame_toggle? 0x01: 0x00;
        data_v[i+2] = frame_index++;

        memcpy(&data_v[i + USB_TRANSFER_HEADER_LENGTH], image_frame.bits()+ j , USB_TRANSFER_BYTES_PAYLOAD);

    }


    frame_toggle = !frame_toggle;

    //Verify the USB device and software is in a proper state where it is legal to
    //use the read/write APIs, prior to actually calling them.
    mutex->lock();   //Get a mutex lock so as to be certain the libUSBDeviceHandle can't change while we are using it.
    if((libUSBDeviceHandle == NULL) || (isConnected == false) || (libusbInitialized == false))
    {
        //Device status isn't ready to communicate for whatever reason.  Therefore
        //we won't try to call the read/write APIs.
        mutex->unlock();  //Done using libUSBDeviceHandle, release our lock on it.
        return;
    }

    //Send Frame data
    status = libusb_interrupt_transfer(libUSBDeviceHandle, 0x01, &data_v[0], total_transfer_bytes, &NumBytesSent, 100);

    mutex->unlock();  //Done using libUSBDeviceHandle, release our lock on it.
    if(status == 0)
    {

        //We successfully sent all of the data if the NumBytesSent variable
        //now contains '1' (which was our intended transfer size).
    }
    else if((status == LIBUSB_ERROR_NO_DEVICE) || (status == LIBUSB_ERROR_IO) || (total_transfer_bytes !=NumBytesSent) )
    {
        //The device may have been detached...  Notify the app.
        isConnected = false;

        //Let the application know of the detached status
        emit(comm_update_connection(isConnected));
    }


}



