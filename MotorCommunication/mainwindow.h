#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "ui_mainwindow.h"
#include "capture.h"
#include "udp.h"
#include "dialogfile.h"
#include <time.h>
#include <QObject>
#include <QThread>
#include <QTableWidget>
#include <vector>

#include <QDebug>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <QHBoxLayout>
#include <QSizeGrip>
#include <QMouseEvent>
#include <QRubberBand>

using namespace std;

class capture;

class Resize_RubberBand : public QWidget {
public:
    Resize_RubberBand(QWidget* parent = 0);

private:
    QRubberBand* rubberband;
    QPoint lastPoint, newPoint;
    bool move_rubberband;
    QPoint rubberband_offset;
    void resizeEvent(QResizeEvent *);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);

};

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class DialogFile;
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void close();


    void DisplayPulse();
    void DisplayPosition();

    capture *videoCapture;

private slots:
    void MultiPoint();

    void on_btnConnected_clicked();

    void on_btnServo_clicked();

    void Thread_Pos();

    void Read();

    void TransportPosX();

    void on_btnHome_clicked();

    void on_btnMoveJ_clicked();

    void on_chkCartesianPos_clicked();

    void on_btnTeaching_clicked();

    void on_chkcarPulPos_clicked();

    void on_btnClearData_clicked();

    void on_btnDelete_clicked();

    void on_btnPlayback_clicked();

    void on_btnLoadJob_clicked();

    void on_btnStartJob_clicked();

    void on_btnExport_clicked();

    void on_btnX1_pressed();

    void on_btnX1_released();

    void on_SliderSpeed_sliderMoved();

    void on_btnX2_pressed();

    void on_btnX2_released();

    void on_btnY1_pressed();

    void on_btnY1_released();

    void on_btnY2_pressed();

    void on_btnY2_released();

    void on_tbnZ1_pressed();

    void on_tbnZ1_released();

    void on_btnZ2_pressed();

    void on_btnZ2_released();

    void on_btnR1_pressed();

    void on_btnR1_released();

    void on_btnR2_pressed();

    void on_btnR2_released();

    void on_btnP1_pressed();

    void on_btnP1_released();

    void on_btnP2_pressed();

    void on_btnP2_released();

    void on_btnYaw1_pressed();

    void on_btnYaw1_released();

    void on_btnYaw2_pressed();

    void on_btnYaw2_released();

    void on_pushButton_clicked();

    void on_btnSaveJob_clicked();

    void on_btnJobExec_clicked();

    void on_pushButtonSave_clicked();

    void handleButton();

    void on_displayImage_clicked();

    QImage cvMatToQImage(const cv::Mat &inMat);
    QPixmap cvMatToQPixmap(const cv::Mat &inMat);
    cv::Mat QImageToMat(QImage image);

    void on_displayQPixmap_clicked();

    void on_camera_clicked();

    void on_camera_off_clicked();

    void on_Get_Background_clicked();

    void on_Mask_clicked();

    void on_chooseObject_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

private:
    Ui::MainWindow *ui;

    bool Find_Mask_Ready = 1;
    Resize_RubberBand *band;

    udp *socket;

    time_t start;
    time_t stop;
    QTimer *startPos;
    QTimer *Multi;

    QTimer *TransPosX;

    QTableWidget *table;

    int count;

    int Move;

    void subtractX();
    void addX();
    void subtractY();
    void addY();
    void subtractZ();
    void addZ();
    void subtractRx();
    void addRx();
    void subtractRy();
    void addRy();
    void subtractRz();
    void addRz();




    void setValueAt( int ix, int jx, const QString &value);
    QString getValueAt(int ix, int jx);

    std::vector<QString> TableValue(int row);

    enum Position
    {
        PosX, PosY, PosZ, PosRx, PosRy, PosRz, Speeds
    };



};
#endif // MAINWINDOW_H
