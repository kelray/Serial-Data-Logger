#include "mainwindow.h"
#include <QApplication>
#include <QColor>
#include <QComboBox>
#include <QDebug>
#include <QGraphicsItem>
#include <QGraphicsWidget>
#include <QLabel>
#include <QtMath>
#include <QPushButton>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QString>
#include <QSignalMapper>
#include <QTimer>
#include <QWidget>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_grid.h>
#include <qwt_symbol.h>
#include <qwt_legend.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <process.h>
#include "visa.h"

#define BUFFERSIZE 256
//#pragma comment(linker, "/SUBSYSTEM:windows /ENTRY:mainCRTStartup")

//Serial port variables
QSerialPort *serial;
QComboBox *selectCOMdroplist, *selectBauddroplist;
QTimer *ChartUpdateTimer;
QTimer *LoggingTimer;

//QWT plot
QwtPlot *qwtPlot;
QwtPlotCurve *curve;
QwtPlotGrid *grid;
QwtPlot *plot;
QPolygonF points;
QPointF point;

//Other variables
int count = 0, i = 0;
double xVal = 0, yVal = 0, zVal = 0, xAxisVar = 0.0;
int x1RangeMax = 512, x1RangeMin = 0, y1RangeMax = 4096, y1RangeMin = 0;
int x2RangeMax = 180, x2RangeMin = 0, y2RangeMax = 4096, y2RangeMin = 0;
double xValArray[1024], xAxisVarArray[1024];

qreal pi = 22/7;
char RxData[BUFFERSIZE]; //= {"0,0,0\n\r"};    //serial port receive buffer
char *token;
int RxBufferCounter = 0;
QString xValStr, yValStr, zValStr;
double IMUval[6];
double AccVal[3];
double GyroVal[3];
char *pch = NULL;
int j = 0;
int index = 0, AvgSize = 100;
double tempNum = 0;
double AveragingResult = 0;
std::vector<double> data_y, data_x;
QByteArray SerialRxData;
int LastIndex = 1;

//VISA definitions
#if defined(_MSC_VER) && !defined(_CRT_SECURE_NO_DEPRECATE)
#define _CRT_SECURE_NO_DEPRECATE
#endif

//Global Variables for VISA COM Interface
static ViSession defaultRM;
static ViSession instr;
static ViUInt32 retCount;
static ViUInt32 writeCount;
static ViStatus status;
static unsigned char buffer[BUFFERSIZE];
int BaudRate = 115200;
FILE * logFile;

/*void __cdecl ReadSerial(void *c)
{
    //serial->readLine(RxData, BUFFERSIZE);
    SerialRxData = serial->readAll();
    //SerialRxData.append(serial->readAll());
    qDebug() << "SerialRxData size: " << SerialRxData.length();//sizeof(SerialRxData);

    //Check if the last character is a new line
    //while(SerialRxData.data()[(SerialRxData.length()-LastIndex)] != '\n')
    while(!SerialRxData.endsWith('\n'))
    {
        SerialRxData.chop(1);
        //SerialRxData.remove((SerialRxData.length()-LastIndex), 1);
        qDebug() << "Size now: " << SerialRxData.length();
        //LastIndex++;
    }
    LastIndex = 1;
    //serial->clear(QSerialPort::AllDirections);
    memcpy(RxData, SerialRxData, sizeof(SerialRxData));
}*/

void ExitFunc()
{
    //Close VISA device
    status = viClose(instr);
    status = viClose(defaultRM);
    //serial->close();

    //Close logging file
    fclose(logFile);
    qDebug() << "closing application";
}

void ConfigSerialVISA(char *PortName)
{
    qDebug() << "VISA port selected:" << PortName;
    /* First we must call viOpenDefaultRM to get the manager
     * handle.  We will store this handle in defaultRM. */
    status = viOpenDefaultRM(&defaultRM);
    if (status < VI_SUCCESS)
    {
        qDebug() << "Could not open a session to the VISA Resource Manager!";
        //exit(EXIT_FAILURE);
    }

    //Now we will open a VISA session to the serial port (COM5).
    //status = viOpen(defaultRM, "ASRL9::INSTR", VI_NULL, VI_NULL, &instr);
    status = viOpen(defaultRM, PortName, VI_NULL, VI_NULL, &instr);
    if (status < VI_SUCCESS)
    {
        qDebug() << "Cannot open a session to the device: " << status;
        //exit(EXIT_FAILURE);
    }

    //COM port settings
    // Set the timeout to 5 seconds (5000 milliseconds)
    status = viSetAttribute(instr, VI_ATTR_TMO_VALUE, 5000);

    // Set the baud rate to 115200 (default is 9600)
    status = viSetAttribute(instr, VI_ATTR_ASRL_BAUD, BaudRate);

    // Set the number of data bits contained in each frame (from 5 to 8)
    status = viSetAttribute(instr, VI_ATTR_ASRL_DATA_BITS, 8);

    // Specify parity
    status = viSetAttribute(instr, VI_ATTR_ASRL_PARITY, VI_ASRL_PAR_NONE);

    // Specify stop bit
    status = viSetAttribute(instr, VI_ATTR_ASRL_STOP_BITS, VI_ASRL_STOP_ONE);

    // Specify that the read operation should terminate when a termination character is received
    status = viSetAttribute(instr, VI_ATTR_TERMCHAR_EN, VI_TRUE);

    // Set the termination character to 0xA
    status = viSetAttribute(instr, VI_ATTR_TERMCHAR, 0x0A);

    qDebug() << "Configuring serial port completed";
    ChartUpdateTimer->start(0);     //start updating chart
}

void MainWindow::updateChart()
{
    SerialRxData = serial->readAll();
    //serial->readLine(RxData, BUFFERSIZE);
    //SerialRxData.append(serial->readAll());
    //qDebug() << "SerialRxData size: " << SerialRxData.length();
    //qDebug() << "Data received: " << SerialRxData << " --i-- " << index++;

    //Check if the last character is a new line
    //while(SerialRxData.data()[(SerialRxData.length()-LastIndex)] != '\n')
   /*if(!SerialRxData.endsWith("\r\n"))
   {
       SerialRxData.chop(1);
       //SerialRxData.remove((SerialRxData.length()-LastIndex), 1);
       //qDebug() << "Size now: " << SerialRxData.length();
       //qDebug() << SerialRxData;
       //memcpy(RxData, SerialRxData, sizeof(SerialRxData));
       //if(SerialRxData.endsWith('\n'))
       // {
       //     break;
       // }
       //LastIndex++;
       //qDebug() << "Character found.";
   }*/
   /*else
   {
       memcpy(RxData, SerialRxData, sizeof(SerialRxData));
   }*/
   //serial->clear(QSerialPort::AllDirections);

    //qDebug() << "SerialRxData size: " << SerialRxData.length();


    //Parsing using strtok
    memcpy(RxData, SerialRxData, SerialRxData.length());    //never use sizeof() with QByteArray
    pch = strtok(RxData, "S\r\n");
    while(pch != NULL)
    {
        //qDebug() << pch;
        xVal = atof(pch);
        //qDebug() << xVal << " --c-- " <<count++;

        /*tempNum = atof(pch);
        if(index < 3)
        {
            tempNum += tempNum;
            index++;
        }
        else
        {
            xVal = tempNum/3;
            qDebug() << xVal;
            index = 0;
        }*/

        /*tempNum = tempNum + xVal;
        index++;
        //qDebug() << index;
        if(index == AvgSize)
        {
            AveragingResult = tempNum/AvgSize;
            tempNum = 0;
            index = 0;
        }*/
        pch = strtok(NULL, "S\r\n");
    }
    SerialRxData.clear();
    memset(RxData, '\0', BUFFERSIZE);

//    //Parsing using Qt regular expressions split
//    qDebug() << RxData;
//    QString data = QString::fromStdString(RxData);
//    //QString data = SerialRxData.toStdString().c_str();
//    //qDebug() << data;
//    QStringList dataList = data.split(QRegExp("(\\S|\\\n|\\\r)"), QString::SkipEmptyParts);

//    //QStringList dataList = SerialRxData.split("S\r\n");
//    /*if(index <= dataList.size())
//    {
//        //xVal = QString(dataList[index]).toDouble();
//        qDebug() << dataList[index];
//        index++;
//    }
//    else
//    {
//        index = 0;
//    }*/

//    /*for(i=0; i < dataList.size(); i++)
//    {
//        //qDebug() << i << dataList[i];
//        AnalogVal[i] = QString(dataList[i]).toDouble();
//        //qDebug() << "index: " << i << " Value: " <<AnalogVal[i];
//    }*/


    /*
    //Read from VISA serial device
    status = viRead(instr, buffer, BUFFERSIZE, &retCount);
    if (status < VI_SUCCESS)
    {
        qDebug() << "Error reading a response from the device.\n";
    }
    else
    {
        sprintf(RxData, "%s", buffer);
        //memcpy(RxData, buffer, BUFFERSIZE);
        pch = strtok(RxData, "S: \r\n");
        while(pch != NULL)
        {
            xVal = atof(pch);
            //qDebug() << xVal;
            pch = strtok(NULL, "S: \r\n");
        }
    }*/

    if(j == x1RangeMax)
    {
        j = 0;
        xAxisVar = 0.0;
        //points.clear();
        data_x.clear();
        data_y.clear();
        qwtPlot->replot();
    }
    j++;

    /*else
    {
        points.value(i)=points.value(i+1);
        j++;
    }*/

    /*point.setX(j);
    point.setY(xVal);
    points += point;*/

    /*if(count < 1024)
    {
        xValArray[count] = xVal;
        xAxisVarArray[count] = xAxisVar;
    }
    else
    {
        count = 0;
        curve->setRawSamples(xAxisVarArray, xValArray, 1024);
        curve->attach(qwtPlot);
        qwtPlot->replot();
    }
    count++;*/

    //qDebug() << AveragingResult;
    xAxisVar+=1.0;
    data_y.push_back(xVal);
    //data_y.push_back(AveragingResult);
    data_x.push_back(xAxisVar);
    curve->setRawSamples(data_x.data(),data_y.data(),data_y.size());
    qwtPlot->replot();
    qwtPlot->update();
}

void MainWindow::ConnectToCOM()
{
    /*
    //Using VISA serial
    char COMportName[5];
    char VISAportName[14] = "ASRL";//9::INSTR";
    char *pch = NULL;

    //COMportName = char(selectCOMdroplist->currentText().toStdString().c_str());
    strcpy(COMportName, selectCOMdroplist->currentText().toStdString().c_str());
    pch = strtok (COMportName,"COM");
    while (pch != NULL)
    {
        //printf ("%s\n",pch);
        strcat(VISAportName, pch);
        strcat(VISAportName, "::INSTR");
        pch = strtok (NULL, "COM");
    }
    qDebug() << "VISA port selected: " <<VISAportName;
    ConfigSerialVISA(VISAportName);
    */
    //using QSerial
    serial->close();
    serial->setPortName(selectCOMdroplist->currentText());
    //serial->setBaudRate(QSerialPort::Baud115200);
    //serial->setBaudRate(921600); //256000	// 921600
    serial->setBaudRate(selectBauddroplist->currentText().toInt());
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);
    serial->setReadBufferSize(65536);
    //serial->setReadBufferSize(0);
    serial->open(QIODevice::ReadWrite);
    //_beginthread(ReadSerial, 4096, NULL);  //here you call the thread
    //serial->open(QIODevice::ReadOnly);
    //serial->setTextModeEnabled(true);
    //LoggingTimer->start();  //for measuring sampling rate only
}

//Logging timer
void MainWindow::LogTimeCb()
{
    fclose(logFile);
    exit(0);
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.setFixedSize(850, 850);
    w.setWindowTitle("Arduino Data-Logger");   //Window title

    atexit(ExitFunc);	//Call exit function

    //Drop-down list for serial port selection
    QLabel *SerialSelect = new QLabel(&w);
    SerialSelect->setGeometry(15, 25, 120, 20);
    SerialSelect->setText("Select COM Port:");
    selectCOMdroplist = new QComboBox(&w);
    selectCOMdroplist->setGeometry(15, 55, 180, 40);

    //Baudrate drop-down list
    //Drop-down list for serial port selection
    QLabel *BaudSelect = new QLabel(&w);
    BaudSelect->setGeometry(350, 25, 120, 20);
    BaudSelect->setText("Select COM Port:");
    selectBauddroplist = new QComboBox(&w);
    selectBauddroplist->setGeometry(350, 55, 180, 40);
    selectBauddroplist->addItems(QStringList() << "4800" << "9600" << "14400" << "19200"
                                 << "38400" << "57600" << "115200" << "128000"
                                 << "230400" << "256000" << "460800" << "921600");

    //Push button
    QPushButton *ConnectButton = new QPushButton(&w);
    ConnectButton->setText("Connect");
    ConnectButton->setGeometry(220, 55, 100, 40);

    //Create serial device
    serial = new QSerialPort(&w);

    //List all available serial ports and add them to a drop-down list
    QList<QSerialPortInfo> list;
    list = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : list)
    {
        /*qDebug() << info.portName();
          qDebug() << info.description();
          qDebug() << info.productIdentifier();
          qDebug() << info.manufacturer();
          qDebug() << info.serialNumber();*/
        selectCOMdroplist->addItem(info.portName());
    }

    QColor BackGroundColor = w.palette().color(QPalette::Window);
    qwtPlot = new QwtPlot(&w);

    qwtPlot->setTitle("Serial Port Plotter");
    //qwtPlot->setCanvasBackground(Qt::white);
    qwtPlot->setCanvasBackground(BackGroundColor);
    qwtPlot->setAxisScale(QwtPlot::yLeft, y1RangeMin, y1RangeMax);
    qwtPlot->setAxisScale(QwtPlot::xBottom, x1RangeMin, x1RangeMax);
    qwtPlot->setGeometry(15, 100, 770, 680);
    qwtPlot->setAutoReplot(true);
    qwtPlot->updateAxes();

    curve = new QwtPlotCurve("Points");
    curve->setPen(Qt::blue,1);
    curve->setRenderHint(QwtPlotItem::RenderAntialiased, true);
    curve->attach(qwtPlot);

    grid = new QwtPlotGrid();
    grid->show();
    grid->attach(qwtPlot);

    //Create log.csv file
    logFile = fopen("log.csv", "w+");

    //QTimer *ChartUpdateTimer = new QTimer(&w);
    ChartUpdateTimer = new QTimer(&w);
    QObject::connect(ChartUpdateTimer, SIGNAL(timeout()), &w, SLOT(updateChart()));
    ChartUpdateTimer->setInterval(0);
    //ChartUpdateTimer->start(0);

    //create a timer and connect it to real-time slot
    //QTimer *LoggingTimer = new QTimer(&w);
    LoggingTimer = new QTimer(&w);
    QObject::connect(LoggingTimer, SIGNAL(timeout()), &w, SLOT(LogTimeCb()));
    LoggingTimer->setInterval(5000);    //Interval 0 means to refresh as fast as possible
    //LoggingTimer->start();    //Enable only when measuring sampling rate

    qwtPlot->show();
    w.show();
    QObject::connect(ConnectButton, SIGNAL(clicked()), &w, SLOT(ConnectToCOM()));
    QObject::connect(serial, SIGNAL(readyRead()), &w, SLOT(updateChart()));
    //QObject::connect(serial, SIGNAL(readChannelFinished()), &w, SLOT(updateChart()));

    return a.exec();
}
