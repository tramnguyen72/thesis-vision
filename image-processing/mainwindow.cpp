#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "capture.h"

using namespace cv;
using namespace std;
using namespace rs2;
int Stream_option;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->cbx_StreamCamera->addItem("Color Stream");
    ui->cbx_StreamCamera->addItem("Depth Stream");

    videoCapture = new capture(this);
    QObject::connect(videoCapture,SIGNAL(newPixmapCaptured_Color()), this, SLOT (handleButton()));
}

MainWindow::~MainWindow()
{
    delete ui;
    videoCapture->terminate();
}

void MainWindow::handleButton()
{
    ui->cameraFrame->setPixmap(videoCapture->pixmap_Color().scaled(640,480));
}

void MainWindow::on_displayImage_clicked()
{
    ui->label->setText("hello");
    Mat image = imread("E:/dog.jpg");
    imshow("dog", image);
    waitKey(0); // Wait for any keystroke in the window
    destroyAllWindows();
}

void MainWindow::on_videoCapture_clicked()
{
    VideoCapture videoCapture;
    Mat videoFrame;
    videoCapture.open(1);
    namedWindow("VideoCapture", WINDOW_AUTOSIZE);
    if(!videoCapture.isOpened())
    {
      cout << "Can't open camera" << endl;
    }
    else
    {
      while (true)
      {
        videoCapture.read(videoFrame);
        QPixmap qframe_color = cvMatToQPixmap(videoFrame);
        ui->cameraFrame->setPixmap(qframe_color.scaled(640,480));

        //imshow("VideoCapture", videoFrame);
        if(waitKey(30) >= 0) break;
      }
     }
}

void MainWindow::on_camera_clicked()
{
    if(videoCapture->camera_running == false)
    {
        videoCapture->camera_running = true;
    }
    videoCapture->start(QThread::HighPriority);
}

void MainWindow::on_camera_off_clicked()
{
    videoCapture->stop();
}

void MainWindow::on_cbx_StreamCamera_currentTextChanged(const QString &arg1)
{
    if(ui->cbx_StreamCamera->currentText()=="Color Stream")
    {
        videoCapture->StreamOption = 0;
    }
    else if(ui->cbx_StreamCamera->currentText()=="Depth Stream")
    {
        videoCapture->StreamOption = 1;
    }
}

void MainWindow::on_Get_Background_clicked()
{
    videoCapture->MaskSignal = 0;
    QPixmap OriginalPix(*ui->cameraFrame->pixmap());

    videoCapture->BackgroundImage = OriginalPix.toImage();

    if(Find_Mask_Ready)
    {
        band = new Resize_RubberBand(ui->cameraFrame);
        band->move(100, 100);
        band->resize(50, 50);
        band->setMinimumSize(30, 30);
        Find_Mask_Ready = 0;
    }
}

void MainWindow::on_Mask_clicked()
{
    QImage Background = videoCapture->BackgroundImage;

    QPoint center;
    center = band->pos();
    if (!Find_Mask_Ready)
    {
        videoCapture->MaskTLPoint = cv::Point((center.x() ),(center.y() ));
        videoCapture->MaskBRPoint = cv::Point((center.x() + band->width()),(center.y() + band->height()));
        videoCapture->MaskSignal = 1;
        band->close();
        Find_Mask_Ready = 1;

    }
}

void MainWindow::on_background_subtraction_clicked()
{
    /*QImage Background = videoCapture->BackgroundImage;
    cv::Mat bg = QImageToMat(Background);
    imshow("aa", bg);
    waitKey(0); // Wait for any keystroke in the window*/
    Mat bg = videoCapture->Mask;
    Mat bgs = videoCapture->BGSubtraction;
    imshow("bb",bgs);
    imshow("a",bg);
    waitKey(0);
}

Resize_RubberBand::Resize_RubberBand(QWidget *parent) : QWidget(parent) {
    //tell QSizeGrip to resize this widget instead of top-level window
    setWindowFlags(Qt::SubWindow);
    QHBoxLayout* layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    QSizeGrip* grip1 = new QSizeGrip(this);
    QSizeGrip* grip2 = new QSizeGrip(this);
    layout->addWidget(grip1, 0, Qt::AlignLeft | Qt::AlignTop);
    layout->addWidget(grip2, 0, Qt::AlignRight | Qt::AlignBottom);
    rubberband = new QRubberBand(QRubberBand::Rectangle, this);
    rubberband->move(0, 0);
    rubberband->show();
    show();
}

void Resize_RubberBand::resizeEvent(QResizeEvent *) {
    rubberband->resize(size());
}
void Resize_RubberBand::mousePressEvent(QMouseEvent *event)
{
    if(rubberband->geometry().contains(event->pos()))
    {
        rubberband_offset = event->pos() - rubberband->pos();
        move_rubberband = true;
    }
}

void Resize_RubberBand::mouseMoveEvent(QMouseEvent *event)
{
    if(move_rubberband)
    {
        rubberband->move(event->pos() - rubberband_offset);
    }
}

void Resize_RubberBand::mouseReleaseEvent(QMouseEvent *event)
{
    move_rubberband = false;
}

void MainWindow::on_displayQPixmap_clicked()
{
    Mat img_Mat = imread("E:/dog.jpg");
    QPixmap img_QPixmap = cvMatToQPixmap(img_Mat);
    ui->cameraFrame->setPixmap(img_QPixmap.scaled(640,480));
}

QPixmap MainWindow:: cvMatToQPixmap( const cv::Mat &inMat )
   {
      return QPixmap::fromImage( cvMatToQImage( inMat ) );
   }

QImage MainWindow:: cvMatToQImage( const cv::Mat &inMat )
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
cv::Mat MainWindow::QImageToMat(QImage image)
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

