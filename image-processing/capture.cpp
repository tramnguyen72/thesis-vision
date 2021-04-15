#include "capture.h"
#include <opencv2/opencv.hpp>
#include <QDebug>

capture::capture(QObject *parent) : QThread(parent)
{

}

cv::Mat CreateMask (cv::Point TL, cv::Point BR ,QImage BackgroundImage)
{
    cv::Mat Mask(BackgroundImage.height(),BackgroundImage.width(), CV_8UC3, cv::Scalar(0, 0, 0));
    cv::rectangle(Mask, BR, TL, cv::Scalar(255, 255, 255),-1);
    return Mask;
}

cv::Mat Background_Subtraction(cv::Mat Frame, cv::Mat BackgroundMat, cv::Mat Mask, cv::Point TL, cv::Point BR)
{
    cv::Mat FrameGray, BGGray, MaskGray, Subtraction;
    cv::cvtColor(Frame, FrameGray, cv::COLOR_BGR2GRAY );
    cv::cvtColor(BackgroundMat, BGGray, cv::COLOR_BGR2GRAY );
    cv::cvtColor(Mask, MaskGray, cv::COLOR_BGR2GRAY );
    cv::GaussianBlur(FrameGray, FrameGray, cv::Size(5, 5), 0 );
    cv::GaussianBlur(BGGray, BGGray, cv::Size(5, 5), 0 );

    cv::absdiff(FrameGray,BGGray,Subtraction) ;
    Subtraction = Subtraction & MaskGray ;


    int sum,cnt = 0;
    for (int i = TL.x; i < BR.x; i++)
    {
        for (int j = TL.y; j < BR.y; j++)
        {
            if ( ((int)Subtraction.at<uchar>(j,i) <= 255)&& ((int)Subtraction.at<uchar>(j,i) >= 0))
            {cnt++;
            sum += (int)Subtraction.at<uchar>(j,i) ;}
        }
    }

    float threshold = sum/cnt ;

    //std::cout << "threshold = " << threshold << std::endl;
    if (threshold <20)  threshold = 20;

    cv::threshold(Subtraction,Subtraction,threshold,255,0);

    return Subtraction;

}


void capture::run()
{
    rs2::config cfg;
    // Start streaming with default recommended configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start(cfg);
    rs2::align align_to_color(RS2_STREAM_COLOR);

    while (camera_running)
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        data = align_to_color.process(data);
        rs2::frame depth = data.get_depth_frame().apply_filter(color_map);
        rs2::frame color = data.get_color_frame();

        // Query frame size (width and height)
        const int w_depth = depth.as<rs2::video_frame>().get_width();
        const int h_depth = depth.as<rs2::video_frame>().get_height();

        const int w_color = color.as<rs2::video_frame>().get_width();
        const int h_color = color.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        cv::Mat depth_frame(cv::Size(w_depth, h_depth), CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat color_frame(cv::Size(w_color, h_color), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);

        cv::Mat BackgroundMat = QImageToMat(BackgroundImage);

        if (!BackgroundMat.empty() && MaskSignal==1)
        {
            Mask = CreateMask(MaskTLPoint,MaskBRPoint,BackgroundImage);
            CreateMaskSingalAlready = 1;

        }
        if (!BackgroundMat.empty() && CreateMaskSingalAlready ==1)
        {
            BGSubtraction = Background_Subtraction(color_frame,BackgroundMat,Mask,MaskTLPoint,MaskBRPoint);
        }
        if (!color_frame.empty() && !depth_frame.empty())
        {
            if(StreamOption == 0)
                Pixmap_Color = cvMatToQPixmap(color_frame);
            else
                Pixmap_Color = cvMatToQPixmap(depth_frame);
            Q_EMIT newPixmapCaptured_Color();
        }
     }
}
QPixmap capture:: cvMatToQPixmap( const cv::Mat &inMat )
   {
      return QPixmap::fromImage( cvMatToQImage( inMat ) );
   }

QImage capture:: cvMatToQImage( const cv::Mat &inMat )
   {
      switch ( inMat.type() )
      {
         // 8-bit, 4 channel
         case CV_8UC4:
         {
            QImage image( inMat.data,
                          inMat.cols, inMat.rows,
                          static_cast<int>(inMat.step),
                          QImage::Format_ARGB32 );

            return image;
         }

         // 8-bit, 3 channel
         case CV_8UC3:
         {
            QImage image( inMat.data,
                          inMat.cols, inMat.rows,
                          static_cast<int>(inMat.step),
                          QImage::Format_RGB888 );

            return image.rgbSwapped();
         }

         // 8-bit, 1 channel
         case CV_8UC1:
         {
#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 2)
            QImage image( inMat.data,
                          inMat.cols, inMat.rows,
                          static_cast<int>(inMat.step),
                          QImage::Format_Grayscale8 );
#else
            static QVector<QRgb>  sColorTable;

            // only create our color table the first time
            if ( sColorTable.isEmpty() )
            {
               sColorTable.resize( 256 );

               for ( int i = 0; i < 256; ++i )
               {
                  sColorTable[i] = qRgb( i, i, i );
               }
            }

            QImage image( inMat.data,
                          inMat.cols, inMat.rows,
                          static_cast<int>(inMat.step),
                          QImage::Format_Indexed8 );

            image.setColorTable( sColorTable );
#endif

            return image;
         }

         default:
            qWarning() << "ASM::cvMatToQImage() - cv::Mat image type not handled in switch:" << inMat.type();
            break;
      }

      return QImage();
   }

cv::Mat capture::QImageToMat(QImage image)
{
    cv::Mat mat;
    switch (image.format())
    {
    case QImage::Format_ARGB32:
    case QImage::Format_RGB32:
    case QImage::Format_ARGB32_Premultiplied:
        mat = cv::Mat(image.height(), image.width(), CV_8UC4, (void*)image.constBits(), image.bytesPerLine());
        break;
    case QImage::Format_RGB888:
        mat = cv::Mat(image.height(), image.width(), CV_8UC3, (void*)image.constBits(), image.bytesPerLine());
        cv::cvtColor(mat, mat, cv::COLOR_BGR2RGB);
        break;
    /*case QImage::Format_Grayscale8:
        mat = cv::Mat(image.height(), image.width(), CV_8UC1, (void*)image.constBits(), image.bytesPerLine());
        break;*/
    }
    return mat;
}

