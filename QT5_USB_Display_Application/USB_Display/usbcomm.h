#ifndef USBCOMM_H
#define USBCOMM_H

#include <QObject>
#include <QTimer>
#include <QThread>
#include <QFuture>
#include <QImage>

#include <wchar.h>
#include <string.h>
#include <stdlib.h>

#include "libusb.h"

#define MAX_STR 65

class Usbcomm : public QObject
//class Usbcomm : public QThread
{
    Q_OBJECT
public:
    explicit Usbcomm(QObject *parent = 0);
    ~Usbcomm();

signals:
    void comm_update_connection(bool isConnected);

public slots:
    void PollUSBConnection(volatile bool*); //Note: should be run with QtConcurrent as a separate thread
    void usb_send_frame_slot(QImage);

private:
    bool isConnected;
    QImage frame_image;

};

#endif // LIBUSB_DEMO_H
