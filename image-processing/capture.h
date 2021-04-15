#ifndef CAPTURE_H
#define CAPTURE_H

#include <QObject>
#include <QPixmap>
#include <QImage>
#include <QThread>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

class capture : public QThread
{
    Q_OBJECT
public:
    explicit capture(QObject *parent = nullptr);

    QPixmap pixmap_Color() const
    {
        return Pixmap_Color ;
    }

    int StreamOption=0;
    rs2::pipeline pipe;
    bool camera_running = true;
    bool CreateMaskSingalAlready,MaskSignal = 0;
    QImage BackgroundImage;
    cv::Point MaskTLPoint, MaskBRPoint = cv::Point(0,0);

    void run();

    // If called it will stop the thread
    void stop() { camera_running = false; }

    cv::Mat BGSubtraction;
    cv::Mat Mask;

signals:
    void newPixmapCaptured_Color();

private:
    QPixmap Pixmap_Color;

    QImage cvMatToQImage(const cv::Mat &inMat);
    QPixmap cvMatToQPixmap(const cv::Mat &inMat);
    cv::Mat QImageToMat(QImage image);
    int cnt;
    double sum;

};

#endif // CAPTURE_H
