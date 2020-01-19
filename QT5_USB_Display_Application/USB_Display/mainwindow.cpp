/*
 * File:   mainwindow.cpp
 * Author:  Gaurav
 * website: www.circuitvalley.com
 * Created on Jan 19, 2020, 1:33 AM
 *	This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *	Email: gauravsingh@circuitvalley.com
************************************************************************/

#include <QFileDialog>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QtDebug>
#include <QtConcurrent/QtConcurrent>  //Location for Qt 5.x.x
#include "usbcomm.h"
#include <QGuiApplication>
#include <QColorDialog>
#include <QItemEditorFactory>
#include <QGraphicsPixmapItem>
#include <QWindow>
#include <QMainWindow>

#include <QScreen>
#include <QDesktopWidget>
//Wide scope variables
QFuture<void> connectionPoller;

volatile bool threadAbortFlag = false;  //flag used to tell PollUSBConnection() concurrent thread to shutdown gracefully
#define DISPLAY_HEIGHT 240
#define DISPLAY_WIDTH 240
#define DISPLAY_BYTE_PER_PIXEL 2

//Constructor function, executed when launching the form.
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    //Initialize the items painted on the GUI form.
    ui->setupUi(this);

    this->setWindowTitle("USB Display: Searching for Device");


    //Send out a debugger message
    qDebug() << "Hello world, from the main window constructor.";

    //Create and initialize an instance of the Usbcomm class.
    usbcomm = new Usbcomm();

    //Execute PollUSBConnection() in a separate thread, so that
    //it can periodically check for USB attach/detach events in the
    //background, without blocking the user interface of this GUI app.
    connectionPoller = QtConcurrent::run(usbcomm, &Usbcomm::PollUSBConnection, &threadAbortFlag);

    connect(usbcomm, SIGNAL(comm_update_connection(bool)), this, SLOT(Update_GUI_Connection(bool)));

    connect(this, SIGNAL(usb_send_frame_signal(QImage)), usbcomm, SLOT(usb_send_frame_slot(QImage)));
    send_frame_timer = new QTimer();
    connect(send_frame_timer, SIGNAL(timeout()), this, SLOT(send_frame_timer_timeout()));
    next_image_timer = new QTimer();

    connect(next_image_timer, SIGNAL(timeout()), this, SLOT(next_image_timer_timeout()));
    connect(ui->next_timer_enable_checkBox, SIGNAL(clicked()), this, SLOT(on_timer_enable_checkBox_clicked()));
    send_frame_timer->start(ui->frame_rate_spinbox->value()); //send refresh frame signal

    scene = new paintScene();       // Initialize the graphic scene.
    ui->graphicsView->setScene(scene);  // Set the graphic scene
    ui->graphicsView->setGeometry(0,0,DISPLAY_WIDTH, DISPLAY_HEIGHT);
    ui->statusBar->showMessage(QString("View Size %1x%2").arg(QString::number(ui->graphicsView->width()), QString::number(ui->graphicsView->height())));


    const QRect screenGeometry = QApplication::desktop()->geometry();

    ui->capture_width_spinBox->setRange(DISPLAY_WIDTH, screenGeometry.width());
    ui->capture_height_spinBox->setRange(DISPLAY_HEIGHT, screenGeometry.height());
    ui->capture_start_x_spinBox->setRange(0, screenGeometry.width() - DISPLAY_WIDTH);
    ui->capture_start_y_spinBox->setRange(0, screenGeometry.height() - DISPLAY_HEIGHT);
    timer = new QTimer();       // Initialize the timer
    connect(timer, &QTimer::timeout, this, &MainWindow::slotTimer);
    timer->start(100);          // Start the timer
}


//Destructor function, executed when closing the form.
MainWindow::~MainWindow()
{
    //Tell the PollUSBConnection() to halt gracefully
    threadAbortFlag = true;
    connectionPoller.waitForFinished(); //Wait until the thread has shut itself down
    delete ui;
}



//Public Slot (callback function) for updating the USB device connection status on the GUI form.
void MainWindow::Update_GUI_Connection(bool isConnected)
{
    if(isConnected)
    {
        this->setWindowTitle("USB Display: Device Found");

    }
    else
    {
        this->setWindowTitle("USB Display: Device not found");
    }
}



void MainWindow::on_clear_push_button_clicked()
{
    ui->graphicsView->scene()->clear();
}


void MainWindow::on_stop_push_button_clicked()
{
    send_frame_timer->stop();
}

void MainWindow::on_color_picker_pushButton_clicked()
{
    QColor color = QColorDialog::getColor(Qt::yellow, this );
    if (color.isValid())
    {
        scene->set_color(color);
    }
}


void MainWindow::on_start_push_button_clicked()
{
    send_frame_timer->start();
}

void MainWindow::on_next_pushButton_clicked()
{

    this->next_image();
}

void MainWindow::on_pen_size_spinBox_valueChanged()
{
    scene->set_pen_size(ui->pen_size_spinBox->value());
}

void MainWindow::slotTimer()
{
    /*We redefine the size of the graphic scene depending on the size of the window
     * */
    timer->stop();
    scene->setSceneRect(0,0, ui->graphicsView->width() - 20, ui->graphicsView->height() - 20);

}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    int i = ui->graphicsView->height();
    ui->graphicsView->setFixedWidth(i);
    ui->statusBar->showMessage(QString("View Size %1x%2").arg(QString::number(ui->graphicsView->width()), QString::number(ui->graphicsView->height())));

    timer->start(10);
}


void MainWindow::send_frame_timer_timeout()
{
    send_frame_timer->start(1000/ ui->frame_rate_spinbox->value()); //send refresh frame 1FPS ->> 1000ms, resolution 1ms, so 60 FPS -> ~16 ms -> 62 fps

    if (ui->follow_cursor_checkBox->isChecked())
    {
        QPoint globalCursorPos = QCursor::pos();
        ui->capture_start_x_spinBox->setValue(globalCursorPos.x());
        ui->capture_start_y_spinBox->setValue(globalCursorPos.y());


    }


    if (ui->capture_screen_checkBox->isChecked())
    {
        QScreen *screen = QGuiApplication::primaryScreen();
        if (const QWindow *window = windowHandle())
        {
            screen = window->screen();
        }

        if (!screen)
            return;

        QPixmap originalPixmap = screen->grabWindow(0,ui->capture_start_x_spinBox->value(), ui->capture_start_y_spinBox->value(), ui->capture_width_spinBox->value(), ui->capture_height_spinBox->value());
        scene->clear();
        QGraphicsPixmapItem *pixitem = scene->addPixmap(originalPixmap.scaled(ui->graphicsView->size(),
                                                                              Qt::KeepAspectRatioByExpanding,
                                                                              Qt::FastTransformation));
        pixitem->setPos(-10,-10);   //Fix for image having 10 pixel offset , reason at this moment unknow.
        ui->graphicsView->setScene(scene);
    }
    QImage image = ui->graphicsView->grab().scaled(DISPLAY_WIDTH,DISPLAY_HEIGHT,Qt::KeepAspectRatioByExpanding,Qt::FastTransformation).toImage().convertToFormat(QImage::Format_RGB16);
    if (image.sizeInBytes() != (DISPLAY_HEIGHT*DISPLAY_WIDTH*DISPLAY_BYTE_PER_PIXEL))
    {
        int i = ui->graphicsView->height();
        ui->graphicsView->setFixedWidth(i);
        return;
    }
    emit usb_send_frame_signal(image); // grab QPixmap convet to Qimage with RGB16
}

void MainWindow::next_image()
{
    static int next = 0;
    int image_index = 0;
    next ++;


    QDir directory("images/");
    QStringList images = directory.entryList(QStringList() << "*.jpg" << "*.JPG",QDir::Files);

    if (next > images.count())
    {
        next = 0;
    }

    foreach(QString filename, images) {
        if (image_index++ == next)
        {
            QString image_file =  "images/" + filename;
            QPixmap img(image_file);
            scene->clear(); //clear all previous pixmaps or they will remain there and cosume memory
            QGraphicsPixmapItem *pixitem = scene->addPixmap(img.scaled(ui->graphicsView->size(),
                                                                       Qt::KeepAspectRatioByExpanding,
                                                                       Qt::FastTransformation));
            pixitem->setPos(-10,-10);   //Fix for image having 10 pixel offset , reason at this moment unknow.
            ui->graphicsView->setScene(scene);
            ui->graphicsView->show();
        }
    }

    if (ui->next_timer_enable_checkBox->isChecked())
    {
        next_image_timer->start(ui->next_image_timer_spinBox->value());
    }
    else
    {
        next_image_timer->stop();
    }
}

void MainWindow::next_image_timer_timeout()
{
    this->next_image();
}

void MainWindow::on_timer_enable_checkBox_clicked()
{
    if (ui->next_timer_enable_checkBox->isChecked())
    {
        next_image_timer->start(ui->next_image_timer_spinBox->value());
    }
    else
    {
        next_image_timer->stop();
    }
}
