#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <time.h>
#include <QDebug>
#include <QMessageBox>
#include <QTimer>
#include <QObject>
#include <QThread>
#include <QTableWidget>
#include <QFileDialog>
#include <QTextStream>
#include <QFile>
#include <QDir>

#include "capture.h"

using namespace cv;
using namespace std;
using namespace rs2;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("Robot Communication");
    setWindowIcon(QIcon("C:/Users/TD/Desktop/Yaskawa/Qt_Controller/MotorCommunication/motomini.png"));
    connect(ui->btnQuit,SIGNAL(clicked()),this,SLOT(close()));
    startPos = new QTimer(this);
    startPos->setInterval(14);
    connect(startPos,SIGNAL(timeout()),this,SLOT(Thread_Pos()));   
    table = ui->tableWidgetPoints;
    count = 0;
    Multi = new QTimer(this);
    Multi->setInterval(2000);
    connect(Multi,SIGNAL(timeout()),this,SLOT(MultiPoint()));


    TransPosX = new QTimer(this);
    TransPosX->setInterval(250);
    connect(TransPosX,SIGNAL(timeout()),this,SLOT(TransportPosX()));


    videoCapture = new capture(this);
    QObject::connect(videoCapture,SIGNAL(newPixmapCaptured_Color()), this, SLOT (handleButton()));


    videoCapture->StreamOption=0;

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

//--------------Thread Position-------------------------------
void MainWindow::Thread_Pos()
{
    if(ui->chkcarPulPos->isChecked()==true)
    {
        socket->INSTANCE = 0x01;
    }
    else
    {
        socket->INSTANCE = 0x65;
    }
    socket->start();

}
void MainWindow::DisplayPosition()
{
    ui->tbxX->setText(QString::number((double)socket->ByteArray2Int32(socket->Get_rx_buffer(),52,4)/1000));
    ui->tbxY->setText(QString::number((double)socket->ByteArray2Int32(socket->Get_rx_buffer(),56,4)/1000));
    ui->tbxZ->setText(QString::number((double)socket->ByteArray2Int32(socket->Get_rx_buffer(),60,4)/1000));
    ui->tbxRoll->setText(QString::number((double)socket->ByteArray2Int32(socket->Get_rx_buffer(),64,4)/10000));
    ui->tbxPitch->setText(QString::number((double)socket->ByteArray2Int32(socket->Get_rx_buffer(),68,4)/10000));
    ui->tbxYaw->setText(QString::number((double)socket->ByteArray2Int32(socket->Get_rx_buffer(),72,4)/10000));

}

void MainWindow::DisplayPulse()
{
    ui->tbxX->setText(QString::number(socket->ByteArray2Int32(socket->Get_rx_buffer(),52,4)));
    ui->tbxY->setText(QString::number(socket->ByteArray2Int32(socket->Get_rx_buffer(),56,4)));
    ui->tbxZ->setText(QString::number(socket->ByteArray2Int32(socket->Get_rx_buffer(),60,4)));
    ui->tbxRoll->setText(QString::number(socket->ByteArray2Int32(socket->Get_rx_buffer(),64,4)));
    ui->tbxPitch->setText(QString::number(socket->ByteArray2Int32(socket->Get_rx_buffer(),68,4)));
    ui->tbxYaw->setText(QString::number(socket->ByteArray2Int32(socket->Get_rx_buffer(),72,4)));
}

//------------------------------------------------------------
void MainWindow::Read()
{
    socket->ReceiveData();
    stop = clock();
    ui->textBrowserReceiveData->setText(socket->rx_data);
    switch (socket->Get_rx_buffer()->at(11)) {
    case 02:
        DisplayPosition();
        break;
    case 03:
        DisplayPulse();
        break;
    default:
        break;
    }
}



//---------Connected/Disconnected------------------------
void MainWindow::on_btnConnected_clicked()
{


    QHostAddress h;
    if(ui->btnConnected->text()=="Connected")
    {
        ui->btnConnected->setText("Disconnected");
        h.setAddress(ui->tbxIPAdress->toPlainText());
        quint16 p = ui->tbxPort->toPlainText().toUInt();
        socket = new udp(h,p);       
        socket->ConnectMotoman();        
        connect(socket->client,SIGNAL(readyRead()),this,SLOT(Read()));
        startPos->start();

    }
    else
    {
        ui->btnConnected->setText("Connected");

        socket->DisconnectMotoman();
    }

}
//------------------------------------------------------------
//----------Servo---------------------------------------------
void MainWindow::on_btnServo_clicked()
{
    if(ui->btnServo->text() == "ON Servo")
    {
        ui->btnServo->setText("OFF Servo");
        ui->lblStatusServo->setText("ON");
        start = clock();
        socket->OnServo();

        ui->textBrowserSendData->setText(socket->tx_data);

    }
    else
    {
        ui->btnServo->setText("ON Servo");
        ui->lblStatusServo->setText("OFF");
        start = clock();
        socket->OffServo();

        ui->textBrowserSendData->setText(socket->tx_data);
    }
}

//-------HomePos---------------------------------------------------
void MainWindow::on_btnHome_clicked()
{
    int32_t Speed,X,Y,Z,Rx,Ry,Rz;
    Speed = 10*100;
    X = 182.231*1000;
    Y = 1.503*1000;
    Z = 86.23*1000;
    Rx = 179.9577*10000;
    Ry = -0.9132*10000;
    Rz = -5.4698*10000;

    start = clock();
    socket->WritePosCar(Speed,X,Y,Z,Rx,Ry,Rz);
//    socket->HomePos();
    ui->textBrowserSendData->setText(socket->tx_data);
}
//-----------------------------------------------------------------

//----------Move Joint---------------------------------------------
void MainWindow::on_btnMoveJ_clicked()
{
    bool x,y,z,rx,ry,rz,v;
    x = ui->tbxMoveX->toPlainText().size()==0;
    y = ui->tbxMoveY->toPlainText().size()==0;
    z = ui->tbxMoveZ->toPlainText().size()==0;
    rx = ui->tbxMoveRoll->toPlainText().size()==0;
    ry = ui->tbxMovePitch->toPlainText().size()==0;
    rz = ui->tbxMoveYaw->toPlainText().size()==0;
    v = ui->tbxMovSpeed->toPlainText().size()==0;

    if(x||y||z||rx||ry||rz)
    {
        QMessageBox::critical(this,"Error","Please insert the position!");
    }
    else if (v) {
        QMessageBox::critical(this,"Error","Please insert the velocity!");
    }
    else
    {
    if(ui->chkCartesianPos->isChecked()==true)
    {
        int32_t speed = ui->tbxMovSpeed->toPlainText().toDouble(&v)*100;
        int32_t X = ui->tbxMoveX->toPlainText().toDouble(&x)*1000;
        int32_t Y = ui->tbxMoveY->toPlainText().toDouble(&y)*1000;
        int32_t Z = ui->tbxMoveZ->toPlainText().toDouble(&z)*1000;
        int32_t RX = ui->tbxMoveRoll->toPlainText().toDouble(&rx)*10000;
        int32_t RY = ui->tbxMovePitch->toPlainText().toDouble(&ry)*10000;
        int32_t RZ = ui->tbxMoveYaw->toPlainText().toDouble(&rz)*10000;
         if(x&&y&&z&&rx&&ry&&rz&&v)
         {
             start = clock();
             socket->WritePosCar(speed,X,Y,Z,RX,RY,RZ);
             ui->textBrowserSendData->setText(socket->tx_data);
         }
         else
         {
              QMessageBox::critical(this,"Error","Please check the number format!");
         }
    }
    else
    {
        int32_t speed = ui->tbxMovSpeed->toPlainText().toInt(&v)*100;
        int32_t R1 = ui->tbxMoveX->toPlainText().toInt(&x);
        int32_t R2 = ui->tbxMoveY->toPlainText().toInt(&y);
        int32_t R3 = ui->tbxMoveZ->toPlainText().toInt(&z);
        int32_t R4 = ui->tbxMoveRoll->toPlainText().toInt(&rx);
        int32_t R5 = ui->tbxMovePitch->toPlainText().toInt(&ry);
        int32_t R6 = ui->tbxMoveYaw->toPlainText().toInt(&rz);
        if(x&&y&&z&&rx&&ry&&rz&&v)
        {
            start = clock();
            socket->WritePosPulse(speed,R1,R2,R3,R4,R5,R6);
            ui->textBrowserSendData->setText(socket->tx_data);
        }

    }

    }
}
void MainWindow::on_chkcarPulPos_clicked()
{
    if(ui->chkcarPulPos->isChecked()==true)
    {
        ui->lblCurrentX->setText("S");
        ui->lblCurrentY->setText("L");
        ui->lblCurrentZ->setText("U");
        ui->lblCurrentRx->setText("R");
        ui->lblCurrentRy->setText("B");
        ui->lblCurrentRz->setText("T");
        ui->lblCurrentUnitX->setText("deg");
        ui->lblCurrentUnitY->setText("deg");
        ui->lblCurrentUnitZ->setText("deg");
        ui->btnX1->setText("S-");
        ui->btnX2->setText("S+");
        ui->btnY1->setText("L-");
        ui->btnY2->setText("L+");
        ui->tbnZ1->setText("U-");
        ui->btnZ2->setText("U+");
        ui->btnR1->setText("R-");
        ui->btnR2->setText("R+");
        ui->btnP1->setText("B-");
        ui->btnP2->setText("B+");
        ui->btnYaw1->setText("T-");
        ui->btnYaw2->setText("T+");

    }
    else
    {
        ui->lblCurrentX->setText("X");
        ui->lblCurrentY->setText("Y");
        ui->lblCurrentZ->setText("Z");
        ui->lblCurrentRx->setText("Rx");
        ui->lblCurrentRy->setText("Ry");
        ui->lblCurrentRz->setText("Rz");
        ui->lblCurrentUnitX->setText("mm");
        ui->lblCurrentUnitY->setText("mm");
        ui->lblCurrentUnitZ->setText("mm");

        ui->btnX1->setText("X-");
        ui->btnX2->setText("X+");
        ui->btnY1->setText("Y-");
        ui->btnY2->setText("Y+");
        ui->tbnZ1->setText("Z-");
        ui->btnZ2->setText("Z+");
        ui->btnR1->setText("RX-");
        ui->btnR2->setText("RX+");
        ui->btnP1->setText("RY-");
        ui->btnP2->setText("RY+");
        ui->btnYaw1->setText("RZ-");
        ui->btnYaw2->setText("RZ+");
    }
}
//-----------------------------------------------------------------

//-----------Teaching Multi Points---------------------------------
void MainWindow::on_chkCartesianPos_clicked()
{
    if(ui->chkCartesianPos->isChecked()==true)
    {
        ui->lblMovX->setText("X");
        ui->lblMovY->setText("Y");
        ui->lblMovZ->setText("Z");
        ui->lblMovRx->setText("Roll");
        ui->lblMovRy->setText("Pitch");
        ui->lblMovRz->setText("Yaw");
        ui->lblUnitCar1->setText("mm");
        ui->lblUnitCar2->setText("mm");
        ui->lblUnitCar3->setText("mm");
        ui->lblUnitCar4->setText("deg");
        ui->lblUnitCar5->setText("deg");
        ui->lblUnitCar6->setText("deg");
    }
    else
    {
        ui->lblMovX->setText("S");
        ui->lblMovY->setText("L");
        ui->lblMovZ->setText("U");
        ui->lblMovRx->setText("R");
        ui->lblMovRy->setText("B");
        ui->lblMovRz->setText("T");
        ui->lblUnitCar1->setText("deg");
        ui->lblUnitCar2->setText("deg");
        ui->lblUnitCar3->setText("deg");
        ui->lblUnitCar4->setText("deg");
        ui->lblUnitCar5->setText("deg");
        ui->lblUnitCar6->setText("deg");
    }
}
void MainWindow::on_btnTeaching_clicked()
{
    int Num = ui->tableWidgetPoints->rowCount() ;
    table->setRowCount(Num);
    table->setColumnCount(6);
    qDebug()<<Num;

    ui->tableWidgetPoints->insertRow(ui->tableWidgetPoints->rowCount());
    ui->tableWidgetPoints->setItem(Num,PosX,new QTableWidgetItem(QString::number(ui->tbxX->toPlainText().toDouble())));
    ui->tableWidgetPoints->setItem(Num,PosY,new QTableWidgetItem(QString::number(ui->tbxY->toPlainText().toDouble())));
    ui->tableWidgetPoints->setItem(Num,PosZ,new QTableWidgetItem(QString::number(ui->tbxZ->toPlainText().toDouble())));
    ui->tableWidgetPoints->setItem(Num,PosRx,new QTableWidgetItem(QString::number(ui->tbxRoll->toPlainText().toDouble())));
    ui->tableWidgetPoints->setItem(Num,PosRy,new QTableWidgetItem(QString::number(ui->tbxPitch->toPlainText().toDouble())));
    ui->tableWidgetPoints->setItem(Num,PosRz,new QTableWidgetItem(QString::number(ui->tbxYaw->toPlainText().toDouble())));


}

//------------------------------------------------------------------

//---------Clear Data-----------------------------------------------
void MainWindow::on_btnClearData_clicked()
{
    int Row = table->rowCount();
    ui->tableWidgetPoints->removeRow(Row-1);
}
//------------------------------------------------------------------

//---------Import Data----------------------------------------------
void MainWindow::on_btnExport_clicked()
{
    auto filename =  QFileDialog::getSaveFileName(this,"Choose an directory",QDir::homePath(), "Txt File (*.txt)");
    if(filename.isEmpty())
    {
        return;
    }
    QFile file(filename);
    if(!file.open(QFile::WriteOnly | QFile::Text))
    {
        return;
    }
    QTextStream xout(&file);
    const int Row = table->rowCount();
    const int Col = table->columnCount();
    for (int ix =0; ix < Row; ++ix)
    {
        xout<<getValueAt(ix,0);
        for (int jx = 1 ;jx < Col; ++jx)
        {
           xout<<','<<getValueAt(ix,jx);
        }
        xout<<"\n";
    }
    file.flush();
    file.close();
}

QString MainWindow::getValueAt(int ix, int jx)
{
    if(!table->item(ix,jx))
    {
        return "";
    }
    return table->item(ix,jx)->text();
}
//void MainWindow::on_btnImport_clicked()
//{

//    auto filename = QFileDialog::getOpenFileName(this,"open file",
//                                       QDir::homePath(), "Txt File (*.txt)");
//    QMessageBox::information(this,"..",filename);
//    if(filename.isEmpty())
//    {
//        return;
//    }
//    QFile file(filename);
//    if(!file.open(QFile::ReadOnly | QFile::Text))
//    {
//        return;
//    }

//    QTextStream in(&file);

//    int ix = 0;
//    while (!in.atEnd())
//    {

//       auto line = in.readLine();
//       auto values = line.split(",");
//       const int colCount = values.size();
//       table->setRowCount(500);
//       table->setColumnCount(colCount);
//       for(int jx = 0; jx< colCount; jx++)
//       {
//           QTableWidgetItem *newItem = new QTableWidgetItem(values.at(jx));
//           table->setItem(ix,jx,newItem);
//       }
//       ++ix;
//       table->setRowCount(ix);
//    }

//    file.close();
//}
//------------------------------------------------------------------

//--------Delete Table----------------------------------------------
void MainWindow::on_btnDelete_clicked()
{
   table->clearContents();
}
//------------------------------------------------------------------

//--------Play back-------------------------------------------------
void MainWindow::on_btnPlayback_clicked()
{

    Multi->start();

}
void MainWindow::MultiPoint()
{

    int row = table->rowCount();
    uint32_t speed = table->item(count,Speeds)->text().toInt()*100;
    uint32_t X = table->item(count,PosX)->text().toDouble()*1000;
    uint32_t Y = table->item(count,PosY)->text().toDouble()*1000;
    uint32_t Z = table->item(count,PosZ)->text().toDouble()*1000;
    uint32_t Rx = table->item(count,PosRx)->text().toDouble()*10000;
    uint32_t Ry = table->item(count,PosRy)->text().toDouble()*10000;
    uint32_t Rz = table->item(count,PosRz)->text().toDouble()*10000;
    socket->WritePosCar(speed,X,Y,Z,Rx,Ry,Rz);
    count+=1;
    qDebug()<<count;

    if(count == row)
    {
         Multi->stop();
         count = 0;
    }
}
//------------------------------------------------------------------

//-------Start Job--------------------------------------------------
void MainWindow::on_btnLoadJob_clicked()
{
    QString Job = ui->tbxLoadJob->toPlainText();

    char jobname[Job.length()] ;
    strcpy(jobname,Job.toStdString().c_str());
    qDebug()<<jobname;
    socket->SelectJob(jobname);
}
void MainWindow::on_btnStartJob_clicked()
{
    socket->StartJob();
}
//------------------------------------------------------------------

//---------Di chuyen------------------------------------------------
void MainWindow::on_btnX1_pressed()
{
   TransPosX->start();
   Move = 1;
}
void MainWindow::on_btnX1_released()
{
    TransPosX->stop();
    Move = 0;
}
void MainWindow::on_btnX2_pressed()
{
    TransPosX->start();
    Move = 2;
}
void MainWindow::on_btnX2_released()
{
    TransPosX->stop();
    Move = 0;
}
void MainWindow::on_btnY1_pressed()
{
    TransPosX->start();
    Move = 3;
}
void MainWindow::on_btnY1_released()
{
    TransPosX->stop();
    Move = 0;
}
void MainWindow::on_btnY2_pressed()
{
    TransPosX->start();
    Move = 4;
}
void MainWindow::on_btnY2_released()
{
    TransPosX->stop();
    Move = 0;
}
void MainWindow::on_tbnZ1_pressed()
{
    TransPosX->start();
    Move = 5;
}
void MainWindow::on_tbnZ1_released()
{
    TransPosX->stop();
    Move = 0;
}
void MainWindow::on_btnZ2_pressed()
{
    TransPosX->start();
    Move = 6;
}
void MainWindow::on_btnZ2_released()
{
    TransPosX->stop();
    Move = 0;
}
void MainWindow::on_btnR1_pressed()
{
    TransPosX->start();
    Move = 7;
}
void MainWindow::on_btnR1_released()
{
    TransPosX->stop();
    Move = 0;
}
void MainWindow::on_btnR2_pressed()
{
    TransPosX->start();
    Move = 8;
}
void MainWindow::on_btnR2_released()
{
    TransPosX->stop();
    Move = 0;
}
void MainWindow::on_btnP1_pressed()
{
    TransPosX->start();
    Move = 9;
}
void MainWindow::on_btnP1_released()
{
    TransPosX->stop();
    Move = 0;
}
void MainWindow::on_btnP2_pressed()
{
    TransPosX->start();
    Move = 10;
}
void MainWindow::on_btnP2_released()
{
    TransPosX->stop();
    Move = 0;
}
void MainWindow::on_btnYaw1_pressed()
{
    TransPosX->start();
    Move = 11;
}
void MainWindow::on_btnYaw1_released()
{
    TransPosX->stop();
    Move = 0;
}
void MainWindow::on_btnYaw2_pressed()
{
    TransPosX->start();
    Move = 12;
}
void MainWindow::on_btnYaw2_released()
{
    TransPosX->stop();
    Move = 0;
}
void MainWindow::TransportPosX()
{
    switch(Move)
    {
    case 1:
        subtractX();
        break;
    case 2:
        addX();
        break;
    case 3:
        subtractY();
        break;
    case 4:
        addY();
        break;
    case 5:
        subtractZ();
        break;
    case 6:
        addZ();
        break;
    case 7:
        subtractRx();
        break;
    case 8:
        addRx();
        break;
    case 9:
        subtractRy();
        break;
    case 10:
        addRy();
        break;
    case 11:
        subtractRz();
        break;
    case 12:
        addRz();
        break;
    default:
        break;
    }
}
void MainWindow::subtractX()
{
    if(ui->chkcarPulPos->isChecked())
    {
        int32_t speed = ui->progressBarSpeed->value()*100;
        int32_t Y = ui->tbxY->toPlainText().toInt();
        int32_t Z = ui->tbxZ->toPlainText().toInt();
        int32_t RX = ui->tbxRoll->toPlainText().toInt();
        int32_t RY = ui->tbxPitch->toPlainText().toInt();
        int32_t RZ = ui->tbxYaw->toPlainText().toInt();
        int32_t X1 = 100*speed/100;
        int32_t X = ui->tbxX->toPlainText().toInt() - X1;
        socket->WritePosPulse(speed,X,Y,Z,RX,RY,RZ);
    }
    else
    {
       int32_t speed = ui->progressBarSpeed->value()*100;
        int32_t Y = ui->tbxY->toPlainText().toDouble()*1000;
        int32_t Z = ui->tbxZ->toPlainText().toDouble()*1000;
        int32_t RX = ui->tbxRoll->toPlainText().toDouble()*10000;
        int32_t RY = ui->tbxPitch->toPlainText().toDouble()*10000;
        int32_t RZ = ui->tbxYaw->toPlainText().toDouble()*10000;
        int32_t X1 = 5*speed/100;
        int32_t X = ui->tbxX->toPlainText().toDouble()*1000 - X1*1000;
        socket->WritePosCar(speed,X,Y,Z,RX,RY,RZ);
    }
}
void MainWindow::addX()
{
    if(ui->chkcarPulPos->isChecked())
    {
        int32_t speed = ui->progressBarSpeed->value()*100;
        int32_t Y = ui->tbxY->toPlainText().toInt();
        int32_t Z = ui->tbxZ->toPlainText().toInt();
        int32_t RX = ui->tbxRoll->toPlainText().toInt();
        int32_t RY = ui->tbxPitch->toPlainText().toInt();
        int32_t RZ = ui->tbxYaw->toPlainText().toInt();
        int32_t X1 = 100*speed/100;
        int32_t X = ui->tbxX->toPlainText().toInt() + X1;
        socket->WritePosPulse(speed,X,Y,Z,RX,RY,RZ);
    }
    else
    {
        int32_t speed = ui->progressBarSpeed->value()*100;
        int32_t Y = ui->tbxY->toPlainText().toDouble()*1000;
        int32_t Z = ui->tbxZ->toPlainText().toDouble()*1000;
        int32_t RX = ui->tbxRoll->toPlainText().toDouble()*10000;
        int32_t RY = ui->tbxPitch->toPlainText().toDouble()*10000;
        int32_t RZ = ui->tbxYaw->toPlainText().toDouble()*10000;
        int32_t X1 = 5*speed/100;
        int32_t X = ui->tbxX->toPlainText().toDouble()*1000 + X1*1000;
        socket->WritePosCar(speed,X,Y,Z,RX,RY,RZ);
    }
}
void MainWindow::subtractY()
{
    if(ui->chkcarPulPos->isChecked())
    {
        int32_t speed = ui->progressBarSpeed->value()*100;
        int32_t X = ui->tbxX->toPlainText().toInt();
        int32_t Z = ui->tbxZ->toPlainText().toInt();
        int32_t RX = ui->tbxRoll->toPlainText().toInt();
        int32_t RY = ui->tbxPitch->toPlainText().toInt();
        int32_t RZ = ui->tbxYaw->toPlainText().toInt();
        int32_t Y1 = 100*speed/100;
        int32_t Y = ui->tbxY->toPlainText().toInt() - Y1;
        socket->WritePosPulse(speed,X,Y,Z,RX,RY,RZ);
    }
    else
    {
       int32_t speed = ui->progressBarSpeed->value()*100;
        int32_t X = ui->tbxX->toPlainText().toDouble()*1000;
        int32_t Z = ui->tbxZ->toPlainText().toDouble()*1000;
        int32_t RX = ui->tbxRoll->toPlainText().toDouble()*10000;
        int32_t RY = ui->tbxPitch->toPlainText().toDouble()*10000;
        int32_t RZ = ui->tbxYaw->toPlainText().toDouble()*10000;
        int32_t Y1 = 5*speed/100;
        int32_t Y = ui->tbxY->toPlainText().toDouble()*1000 - Y1*1000;
        socket->WritePosCar(speed,X,Y,Z,RX,RY,RZ);
    }
}
void MainWindow::addY()
{
    if(ui->chkcarPulPos->isChecked())
    {
        int32_t speed = ui->progressBarSpeed->value()*100;
        int32_t X = ui->tbxX->toPlainText().toInt();
        int32_t Z = ui->tbxZ->toPlainText().toInt();
        int32_t RX = ui->tbxRoll->toPlainText().toInt();
        int32_t RY = ui->tbxPitch->toPlainText().toInt();
        int32_t RZ = ui->tbxYaw->toPlainText().toInt();
        int32_t Y1 = 100*speed/100;
        int32_t Y = ui->tbxY->toPlainText().toInt() + Y1;
        socket->WritePosPulse(speed,X,Y,Z,RX,RY,RZ);
    }
    else
    {
        int32_t speed = ui->progressBarSpeed->value()*100;
        int32_t X = ui->tbxX->toPlainText().toDouble()*1000;
        int32_t Z = ui->tbxZ->toPlainText().toDouble()*1000;
        int32_t RX = ui->tbxRoll->toPlainText().toDouble()*10000;
        int32_t RY = ui->tbxPitch->toPlainText().toDouble()*10000;
        int32_t RZ = ui->tbxYaw->toPlainText().toDouble()*10000;
        int32_t Y1 = 5*speed/100;
        int32_t Y = ui->tbxY->toPlainText().toDouble()*1000 + Y1*1000;
        socket->WritePosCar(speed,X,Y,Z,RX,RY,RZ);
    }
}
void MainWindow::subtractZ()
{
    if(ui->chkcarPulPos->isChecked())
    {
        int32_t speed = ui->progressBarSpeed->value()*100;
        int32_t X = ui->tbxX->toPlainText().toInt();
        int32_t Y = ui->tbxY->toPlainText().toInt();
        int32_t RX = ui->tbxRoll->toPlainText().toInt();
        int32_t RY = ui->tbxPitch->toPlainText().toInt();
        int32_t RZ = ui->tbxYaw->toPlainText().toInt();
        int32_t Z1 = 100*speed/100;
        int32_t Z = ui->tbxZ->toPlainText().toInt() - Z1;
        socket->WritePosPulse(speed,X,Y,Z,RX,RY,RZ);
    }
    else
    {
        int32_t speed = ui->progressBarSpeed->value()*100;
        int32_t X = ui->tbxX->toPlainText().toDouble()*1000;
        int32_t Y = ui->tbxY->toPlainText().toDouble()*1000;
        int32_t RX = ui->tbxRoll->toPlainText().toDouble()*10000;
        int32_t RY = ui->tbxPitch->toPlainText().toDouble()*10000;
        int32_t RZ = ui->tbxYaw->toPlainText().toDouble()*10000;
        int32_t Z1 = 5*speed/100;
        int32_t Z = ui->tbxZ->toPlainText().toDouble()*1000 - Z1*1000;
        socket->WritePosCar(speed,X,Y,Z,RX,RY,RZ);
    }
}
void MainWindow::addZ()
{
    if(ui->chkcarPulPos->isChecked())
    {
        int32_t speed = ui->progressBarSpeed->value()*100;
        int32_t X = ui->tbxX->toPlainText().toInt();
        int32_t Y = ui->tbxY->toPlainText().toInt();
        int32_t RX = ui->tbxRoll->toPlainText().toInt();
        int32_t RY = ui->tbxPitch->toPlainText().toInt();
        int32_t RZ = ui->tbxYaw->toPlainText().toInt();
        int32_t Z1 = 100*speed/100;
        int32_t Z = ui->tbxZ->toPlainText().toInt() + Z1;
        socket->WritePosPulse(speed,X,Y,Z,RX,RY,RZ);
    }
    else
    {
        int32_t speed = ui->progressBarSpeed->value()*100;
        int32_t X = ui->tbxX->toPlainText().toDouble()*1000;
        int32_t Y = ui->tbxY->toPlainText().toDouble()*1000;
        int32_t RX = ui->tbxRoll->toPlainText().toDouble()*10000;
        int32_t RY = ui->tbxPitch->toPlainText().toDouble()*10000;
        int32_t RZ = ui->tbxYaw->toPlainText().toDouble()*10000;
        int32_t Z1 = 5*speed/100;
        int32_t Z = ui->tbxZ->toPlainText().toDouble()*1000 + Z1*1000;
        socket->WritePosCar(speed,X,Y,Z,RX,RY,RZ);
    }
}
void MainWindow::subtractRx()
{
    if(ui->chkcarPulPos->isChecked())
    {
        int32_t speed = ui->progressBarSpeed->value()*100;
        int32_t X = ui->tbxX->toPlainText().toInt();
        int32_t Y = ui->tbxY->toPlainText().toInt();
        int32_t Z = ui->tbxZ->toPlainText().toInt();
        int32_t RY = ui->tbxPitch->toPlainText().toInt();
        int32_t RZ = ui->tbxYaw->toPlainText().toInt();
        int32_t Rx1 = 100*speed/100;
        int32_t RX = ui->tbxRoll->toPlainText().toInt() - Rx1;
        socket->WritePosPulse(speed,X,Y,Z,RX,RY,RZ);
    }
    else
    {
        int32_t speed = ui->progressBarSpeed->value()*100;
        int32_t X = ui->tbxX->toPlainText().toDouble()*1000;
        int32_t Y = ui->tbxY->toPlainText().toDouble()*1000;
        int32_t Z = ui->tbxZ->toPlainText().toDouble()*1000;
        int32_t RY = ui->tbxPitch->toPlainText().toDouble()*10000;
        int32_t RZ = ui->tbxYaw->toPlainText().toDouble()*10000;
        int32_t RX1 = 0.2*speed/100;
        int32_t RX = ui->tbxRoll->toPlainText().toDouble()*10000 - RX1*10000;
        socket->WritePosCar(speed,X,Y,Z,RX,RY,RZ);
    }
}
void MainWindow::addRx()
{
    if(ui->chkcarPulPos->isChecked())
    {
        int32_t speed = ui->progressBarSpeed->value()*100;
        int32_t X = ui->tbxX->toPlainText().toInt();
        int32_t Y = ui->tbxY->toPlainText().toInt();
        int32_t Z = ui->tbxZ->toPlainText().toInt();
        int32_t RY = ui->tbxPitch->toPlainText().toInt();
        int32_t RZ = ui->tbxYaw->toPlainText().toInt();
        int32_t Rx1 = 100*speed/100;
        int32_t RX = ui->tbxRoll->toPlainText().toInt() + Rx1;
        socket->WritePosPulse(speed,X,Y,Z,RX,RY,RZ);
    }
    else
    {
        int32_t speed = ui->progressBarSpeed->value()*100;
        int32_t X = ui->tbxX->toPlainText().toDouble()*1000;
        int32_t Y = ui->tbxY->toPlainText().toDouble()*1000;
        int32_t Z = ui->tbxZ->toPlainText().toDouble()*1000;
        int32_t RY = ui->tbxPitch->toPlainText().toDouble()*10000;
        int32_t RZ = ui->tbxYaw->toPlainText().toDouble()*10000;
        int32_t RX1 = 0.2*speed/100;
        int32_t RX = ui->tbxRoll->toPlainText().toDouble()*10000 + RX1*10000;
        socket->WritePosCar(speed,X,Y,Z,RX,RY,RZ);
    }
}
void MainWindow::subtractRy()
{
    if(ui->chkcarPulPos->isChecked())
    {
        int32_t speed = ui->progressBarSpeed->value()*100;
        int32_t X = ui->tbxX->toPlainText().toInt();
        int32_t Y = ui->tbxY->toPlainText().toInt();
        int32_t Z = ui->tbxZ->toPlainText().toInt();
        int32_t RX = ui->tbxRoll->toPlainText().toInt();
        int32_t RZ = ui->tbxYaw->toPlainText().toInt();
        int32_t Rx1 = 100*speed/100;
        int32_t RY = ui->tbxPitch->toPlainText().toInt() - Rx1;
        socket->WritePosPulse(speed,X,Y,Z,RX,RY,RZ);
    }
    else
    {
        int32_t speed = ui->progressBarSpeed->value()*100;
        int32_t X = ui->tbxX->toPlainText().toDouble()*1000;
        int32_t Y = ui->tbxY->toPlainText().toDouble()*1000;
        int32_t Z = ui->tbxZ->toPlainText().toDouble()*1000;
        int32_t RX = ui->tbxRoll->toPlainText().toDouble()*10000;
        int32_t RZ = ui->tbxYaw->toPlainText().toDouble()*10000;
        int32_t RY1 = 0.2*speed/100;
        int32_t RY = ui->tbxPitch->toPlainText().toDouble()*10000 - RY1*10000;
        socket->WritePosCar(speed,X,Y,Z,RX,RY,RZ);
    }
}
void MainWindow::addRy()
{
    if(ui->chkcarPulPos->isChecked())
    {
        int32_t speed = ui->progressBarSpeed->value()*100;
        int32_t X = ui->tbxX->toPlainText().toInt();
        int32_t Y = ui->tbxY->toPlainText().toInt();
        int32_t Z = ui->tbxZ->toPlainText().toInt();
        int32_t RX = ui->tbxRoll->toPlainText().toInt();
        int32_t RZ = ui->tbxYaw->toPlainText().toInt();
        int32_t Rx1 = 100*speed/100;
        int32_t RY = ui->tbxPitch->toPlainText().toInt() + Rx1;
        socket->WritePosPulse(speed,X,Y,Z,RX,RY,RZ);
    }
    else
    {
        int32_t speed = ui->progressBarSpeed->value()*100;
        int32_t X = ui->tbxX->toPlainText().toDouble()*1000;
        int32_t Y = ui->tbxY->toPlainText().toDouble()*1000;
        int32_t Z = ui->tbxZ->toPlainText().toDouble()*1000;
        int32_t RX = ui->tbxRoll->toPlainText().toDouble()*10000;
        int32_t RZ = ui->tbxYaw->toPlainText().toDouble()*10000;
        int32_t RY1 = 0.2*speed/100;
        int32_t RY = ui->tbxPitch->toPlainText().toDouble()*10000 + RY1*10000;
        socket->WritePosCar(speed,X,Y,Z,RX,RY,RZ);
    }
}
void MainWindow::subtractRz()
{
    if(ui->chkcarPulPos->isChecked())
    {
        int32_t speed = ui->progressBarSpeed->value()*100;
        int32_t X = ui->tbxX->toPlainText().toInt();
        int32_t Y = ui->tbxY->toPlainText().toInt();
        int32_t Z = ui->tbxZ->toPlainText().toInt();
        int32_t RX = ui->tbxRoll->toPlainText().toInt();
        int32_t RY = ui->tbxPitch->toPlainText().toInt();
        int32_t Rx1 = 100*speed/100;
        int32_t RZ = ui->tbxYaw->toPlainText().toInt() - Rx1;
        socket->WritePosPulse(speed,X,Y,Z,RX,RY,RZ);
    }
    else
    {
        int32_t speed = ui->progressBarSpeed->value()*100;
        int32_t X = ui->tbxX->toPlainText().toDouble()*1000;
        int32_t Y = ui->tbxY->toPlainText().toDouble()*1000;
        int32_t Z = ui->tbxZ->toPlainText().toDouble()*1000;
        int32_t RX = ui->tbxRoll->toPlainText().toDouble()*10000;
        int32_t RY = ui->tbxPitch->toPlainText().toDouble()*10000;
        int32_t RZ1 = 0.2*speed/100;
        int32_t RZ = ui->tbxYaw->toPlainText().toDouble()*10000 - RZ1*10000;
        socket->WritePosCar(speed,X,Y,Z,RX,RY,RZ);
    }
}
void MainWindow::addRz()
{
    if(ui->chkcarPulPos->isChecked())
    {
        int32_t speed = ui->progressBarSpeed->value()*100;
        int32_t X = ui->tbxX->toPlainText().toInt();
        int32_t Y = ui->tbxY->toPlainText().toInt();
        int32_t Z = ui->tbxZ->toPlainText().toInt();
        int32_t RX = ui->tbxRoll->toPlainText().toInt();
        int32_t RY = ui->tbxPitch->toPlainText().toInt();
        int32_t Rx1 = 100*speed/100;
        int32_t RZ = ui->tbxYaw->toPlainText().toInt() + Rx1;
        socket->WritePosPulse(speed,X,Y,Z,RX,RY,RZ);
    }
    else
    {
        int32_t speed = ui->progressBarSpeed->value()*100;
        int32_t X = ui->tbxX->toPlainText().toDouble()*1000;
        int32_t Y = ui->tbxY->toPlainText().toDouble()*1000;
        int32_t Z = ui->tbxZ->toPlainText().toDouble()*1000;
        int32_t RX = ui->tbxRoll->toPlainText().toDouble()*10000;
        int32_t RY = ui->tbxPitch->toPlainText().toDouble()*10000;
        int32_t RZ1 = 0.2*speed/100;
        int32_t RZ = ui->tbxYaw->toPlainText().toDouble()*10000 + RZ1*10000;
        socket->WritePosCar(speed,X,Y,Z,RX,RY,RZ);
    }
}
void MainWindow::on_SliderSpeed_sliderMoved()
{
    ui->progressBarSpeed->setValue(ui->SliderSpeed->value());
    qDebug()<<ui->SliderSpeed->value();
}
//--------------------------------------------------------------

void MainWindow::on_pushButton_clicked()
{
   QString a = "PICKA9.JBI";
   char b[a.length()];
   strcpy(b,a.toStdString().c_str());
   socket->SAVEFILE(b,10);
}


void MainWindow::on_btnSaveJob_clicked()
{

}


//------Button JobExecuse----------------------------
void MainWindow::on_btnJobExec_clicked()
{
    DialogFile file;
    file.setModal(true);
    file.getValueTable(TableValue(table->rowCount()),table->rowCount());
    file.exec();
    startPos->stop();

}
std::vector<QString> MainWindow::TableValue(int row)
{

   std::vector<QString> s;
   for(int i =0;i<row;i++){
       for(int j=0;j<7;j++){
      s.push_back(table->item(i,j)->text());
       }
   }

   return s;
}
//------------------------------------------------------------

void MainWindow::on_pushButtonSave_clicked()
{

}

void MainWindow::on_displayImage_clicked()
{
    Mat image = imread("E:/dog.jpg");
    cv::putText(image, //target image
                "Hello, OpenCV!", //text
                cv::Point(10, image.rows / 2), //top-left position
                cv::FONT_HERSHEY_DUPLEX,
                1.0,
                CV_RGB(118, 185, 0), //font color
                2);
    imshow("dog", image);
    waitKey(0); // Wait for any keystroke in the window
    destroyAllWindows();
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

void MainWindow::on_displayQPixmap_clicked()
{
    Mat img_Mat = imread("E:/dog.jpg");
    QPixmap img_QPixmap = cvMatToQPixmap(img_Mat);
    ui->cameraFrame->setPixmap(img_QPixmap.scaled(640,480));
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

    videoCapture->findcontour_ready =1;
}

void MainWindow::on_chooseObject_clicked()
{
    videoCapture->numberBlob = videoCapture->numberBlob + 1;
    cout << videoCapture->numberBlob << endl;
}



void MainWindow::on_pushButton_2_clicked()
{
    videoCapture->StreamOption = 1;
}

void MainWindow::on_pushButton_3_clicked()
{
    videoCapture->StreamOption = 0;
}
