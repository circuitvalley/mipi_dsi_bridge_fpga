This is a cross platform project that is similar in functionality as the
"plug_and_play_example\src\vc++2008_express" project.  However, 
this version is developed in the Qt 5.x.x development environment (from www.qt.org), 
and uses the "libusb v1.0.x" APIs.

The libusb v1.0.x "driver" provides top level libusb API functions for accessing 
the USB device, but uses the Microsoft WinUSB driver when the application is 
used on the Windows platform.  Therefore, when this project is built and run 
on a Windows based machine, install and use the standard WinUSB driver package 
that comes with this demo, even when using this libusb application project.

The libusb-1.0.dll file and respective .lib file provided here are based on 
the libusb v1.0.19 source.  See www.libusb.org for details on the libusb APIs 
and links to the latest libusb source code and header download.

If using this project to build a Linux or Mac OS X executable, make sure
that the libusb v1.0.19 driver is available and installed correctly.