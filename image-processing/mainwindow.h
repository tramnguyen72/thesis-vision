#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "ui_mainwindow.h"
#include "capture.h"

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

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    capture *videoCapture;

private slots:
    void on_displayImage_clicked();

    void on_videoCapture_clicked();

    void on_camera_clicked();

    void on_camera_off_clicked();

    void on_cbx_StreamCamera_currentTextChanged(const QString &arg1);

    void on_background_subtraction_clicked();

    void on_displayQPixmap_clicked();

    void handleButton();

    QImage cvMatToQImage(const cv::Mat &inMat);
    QPixmap cvMatToQPixmap(const cv::Mat &inMat);
    cv::Mat QImageToMat(QImage image);

    void on_Get_Background_clicked();

    void on_Mask_clicked();

private:
    Ui::MainWindow *ui;
    bool Find_Mask_Ready = 1;
    Resize_RubberBand *band;

};
#endif // MAINWINDOW_H
