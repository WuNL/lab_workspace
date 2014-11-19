#include <QtCore/QCoreApplication>
#include <iostream>
#include "Lms511Thread.h"

void customMessageHandler(QtMsgType type, const char *msg)
{
    QString txt;
    switch(type)
    {
    case QtDebugMsg:
        txt = QString("Debug: %l").arg(msg);
        break;
    case QtWarningMsg:
        txt=QString("Warning: %l").arg(msg);
        break;
    case QtCriticalMsg:
        txt=QString("Critical: %l").arg(msg);
        break;
    case QtFatalMsg:
        txt=QString("Fatal: %l").arg(msg);
        break;
    };
    std::cout<<txt.toStdString ()<<std::endl;
}


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    Lms511Thread lms511Thread;
    return a.exec();
}
