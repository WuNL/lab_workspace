#pragma once

//#define _DEBUG
#ifndef _DEBUG
#define QT_NO_DEBUG_OUTPUT
#endif // _DEBUG

#include <QThread>
#include <QtDebug>
#include <QTcpSocket>
#include <QtGlobal>
#include <QList>
#include <QStringList>
#include <QFile>
#include <QTextStream>
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

enum LmsMsg{RequestStartMeasurement, AnswerStartMeasurement, RequestStopMeasurement, AnswerStopMeasurement, RequestQueryStatus, AnswerQueryStatus, RequestSingleMeasurementOutput, AnswerSingleMeasurement, RequestStartContinuousMeasurementOutput, RequestStopContinuousMeasurementOutput, AnswerContinuousMeasurement, RequestReadFreqRes, AnswerReadFreqRes, RequestReadContamination, AnswerReadContamination};

class Lms511Thread :
        public QThread
{
    Q_OBJECT
public:
    Lms511Thread(void);
    ~Lms511Thread(void);
public:
    ros::NodeHandle* m_pNodeHandleLms511;
    ros::Publisher m_pubLms511;
    sensor_msgs::LaserScan m_laserScan;

    QTcpSocket m_tcpSocketLms511;
    char * m_pMsg;
    char * m_pContinuousScandataHeader;
    QByteArray m_FrameBuffer;
    QByteArray m_Frame;
    QByteArray m_DistFrame;
    QByteArray m_RssiFrame;
    QList<QByteArray> m_List;
    QList<QByteArray>::iterator m_Iterator;
    QByteArray m_TempArray;
    float m_DistScalingFactor;
    float m_RssiScalingFactor;
    int m_DistNumberData;
    std::string m_strNodeName;
    std::string m_strTopicName;
    std::string m_strFrameID;
    struct LmsMeasurements
    {
        bool bAllocated;
        float fDistScalingFactor;
        float fRssiScalingFactor;
        int nStartingAngle;
        unsigned short int nAngularStepWidth;
        unsigned short int nNumberData;
        unsigned short int * pDist;
        unsigned short int * pRssi;
    }m_lmsMeasurements;

public slots:
    void StartMeasurements();
    void ReadMsg();
    void Err();

public:
    void ProcessMsg(QByteArray msg);
    bool GetCompleteFrame();
    bool GetDistAndRssiFrame();
    bool DecomposeDistFrame();
    bool DecomposeRssiFrame();
    void PublishRosMsg();
    bool GenerateMsg(LmsMsg enumMsg, char* pMsg);

    void Deg2Rad(float deg, float *rad);
    bool Str2Int_16(QByteArray str, unsigned short int * data);
    bool Str2Real(QByteArray str, float* data);
    bool Str2Int_32(QByteArray str, int* data);
    bool Str2Uint_16(QByteArray str, unsigned short int *data);
    bool Str2Uint_32(QByteArray str, unsigned int * data);
};
