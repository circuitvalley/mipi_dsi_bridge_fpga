#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "usbcomm.h"

#include <paintscene.h>


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    
private:
    Ui::MainWindow *ui;
    Usbcomm* usbcomm;
    QTimer *timer;     /* Define a timer for preparing actual sizes
                            * graphic scene
                            * */
    paintScene *scene;  // Declare a custom graphic scene

    QTimer *send_frame_timer;
    QTimer *next_image_timer;
     void next_image();

public slots:
    void Update_GUI_Connection(bool isConnected);
    void resizeEvent(QResizeEvent * event);

private slots:
    void slotTimer();
    void send_frame_timer_timeout();
    void next_image_timer_timeout();
signals:
    void usb_send_frame_signal(QImage);

private slots:
    void on_clear_push_button_clicked();
    void on_stop_push_button_clicked();
    void on_start_push_button_clicked();
    void on_color_picker_pushButton_clicked();
    void on_next_pushButton_clicked();
    void on_pen_size_spinBox_valueChanged();
    void on_timer_enable_checkBox_clicked();
//    void on_follow_cursor_checkBox_toggled();
//    void on_capture_screen_checkBox_toggled();
};

#endif // MAINWINDOW_H
