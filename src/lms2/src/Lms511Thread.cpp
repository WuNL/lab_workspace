#include "Lms511Thread.h"


/**@brief Lms511Thread constructor
*
*@note
*/
Lms511Thread::Lms511Thread(void):
    m_strNodeName("Lms2Driver"),
    m_strTopicName("lms2Scan"),
    m_strFrameID("lms2")
{
    m_pMsg = new char[50];
    m_pContinuousScandataHeader = new char[50];
    GenerateMsg(AnswerContinuousMeasurement, m_pContinuousScandataHeader);
    m_lmsMeasurements.bAllocated = false;

    int argc = 0;
    char **argv;
    argv = NULL;
    ros::init (argc, argv, m_strNodeName);
    m_pNodeHandleLms511 = new ros::NodeHandle;
    m_pubLms511 = m_pNodeHandleLms511->advertise<sensor_msgs::LaserScan>(m_strTopicName, 20);

    connect(&m_tcpSocketLms511, SIGNAL(connected()), this, SLOT(StartMeasurements()));
    connect(&m_tcpSocketLms511, SIGNAL(readyRead()), this, SLOT(ReadMsg()));
    m_tcpSocketLms511.connectToHost("192.168.200.32", 2112);

}

/**@brief Lms511Thread deconstructor
*
*@note
*/
Lms511Thread::~Lms511Thread(void) 
{
    delete m_pMsg;
}

/**@brief send start measurements cmd to the laser;

*slot function, if the connection with lms laser scanner is established,
this function is called to send request for continuous measurements output
*
*@return void;
*@note
*/
void Lms511Thread::StartMeasurements()
{
    //GenerateMsg(RequestQueryStatus, m_pMsg);
    //m_tcpSocketLms511.write(m_pMsg);
    //usleep(1000);

    //GenerateMsg(RequestReadFreqRes, m_pMsg);
    //m_tcpSocketLms511.write(m_pMsg);
    //usleep(1000);

    //GenerateMsg(RequestReadContamination, m_pMsg);
    //m_tcpSocketLms511.write(m_pMsg);
    //usleep(1000);

    GenerateMsg(RequestStartContinuousMeasurementOutput, m_pMsg);
    m_tcpSocketLms511.write(m_pMsg);
    usleep(1000);

    return;
}

/**@brief call this function to read data from the socket;
*
*slot function, if new message is ready, this function is called to read all the message, the function ProcessMsg is called to procees the message.
*@return void;
*@note
*/
void Lms511Thread::ReadMsg()
{
    QByteArray msg = this->m_tcpSocketLms511.readAll();
    //std::cout<<msg.data()<<std::endl;
    ProcessMsg(msg);

}

/**@brief
*
*process all the message received
1. append the message after the message queue
2. split the message by 0x03 to get complete message
3. call the function to dispatch message
*@param msg: the msg received;
*@return void;
*@note
*/
void Lms511Thread::ProcessMsg (QByteArray msg)
{
    if(m_FrameBuffer.size()>50000)
    {
        m_FrameBuffer.clear();
    }
    m_FrameBuffer.append(msg);

    //append to frame buffer;
    while(GetCompleteFrame())
    {
        if(GetDistAndRssiFrame())
        {
            DecomposeDistFrame();
            DecomposeRssiFrame();
            PublishRosMsg();
        }
    }
    return;
}

/**@brief try to get complete data frame from the frame buffer;
*
*@return bool: if complete data frame is get, return true; else, return false;
*@note
*/
bool Lms511Thread::GetCompleteFrame()
{
    int indexOfHead = -1;
    indexOfHead = m_FrameBuffer.indexOf(m_pContinuousScandataHeader);
    if(indexOfHead == -1)
    {
        m_FrameBuffer.clear();
        return false;
    }

    m_FrameBuffer.remove(0, indexOfHead);
    int indexOfTail = -1;
    indexOfTail = m_FrameBuffer.indexOf(char(0x03));
    if(indexOfTail == -1)
    {
        return false;
    }

    m_Frame = m_FrameBuffer.left(indexOfTail);
    m_FrameBuffer.remove(0, indexOfTail);
    return true;
}


/**@brief try to get dist and rssi frame from the data frame;
*
*@return bool: if complete dist and rssi frame is get, return true; else, return false;
*@note
*/
bool Lms511Thread::GetDistAndRssiFrame()
{
    int indexOfDist = -1;
    indexOfDist = m_Frame.indexOf("DIST");
    if(indexOfDist == -1)
    {
        m_Frame.clear();
        return false;
    }
    m_Frame.remove(0, indexOfDist);

    int indexOfRssi = -1;
    indexOfRssi = m_Frame.indexOf(" RSSI");
    if(indexOfRssi == -1)
    {
        return false;
    }

    m_DistFrame = m_Frame.left(indexOfRssi);
    m_Frame.remove(0, indexOfRssi+1);
    m_RssiFrame = m_Frame.left(m_Frame.size());
    m_Frame.clear();
    return true;
}

/**@brief decompose dist frame to get distance measurements;
*
*@return bool: if dist frame is successfully decomposed, return true; else, return false;
*@note the memory needed to store distance and intensity measurements is allocated when first dist frame is decomposed;
*/
bool Lms511Thread::DecomposeDistFrame()
{
    m_List.clear();
    m_List = m_DistFrame.split(' ');
    m_Iterator = m_List.begin();//DIST

    m_Iterator++;//scaling factor
    m_TempArray = *m_Iterator;
    if(false == Str2Real(m_TempArray, &(m_lmsMeasurements.fDistScalingFactor)))
    {
        return false;
    }

    m_Iterator++;//scaling offset;
    m_Iterator++;//starting angle;
    m_TempArray = *m_Iterator;
    if(false == Str2Int_32(m_TempArray, &(m_lmsMeasurements.nStartingAngle)))
    {
        return false;
    }

    m_Iterator++;//angular step width;
    m_TempArray = *m_Iterator;
    if(false == Str2Int_16(m_TempArray, &(m_lmsMeasurements.nAngularStepWidth)))
    {
        return false;
    }

    m_Iterator++;//number data;
    m_TempArray = *m_Iterator;
    if(false ==Str2Int_16(m_TempArray, &(m_lmsMeasurements.nNumberData)))
    {
        return false;
    }

    if(m_lmsMeasurements.bAllocated == false)//allocate memory;
    {
        m_lmsMeasurements.pDist = new unsigned short [m_lmsMeasurements.nNumberData];
        m_lmsMeasurements.pRssi = new unsigned short [m_lmsMeasurements.nNumberData];
        m_lmsMeasurements.bAllocated = true;
    }

    int i = 0;
    for(i=0; i<m_lmsMeasurements.nNumberData; i++)
    {
        m_Iterator++;
        Str2Uint_16(*m_Iterator, m_lmsMeasurements.pDist+i);
    }
}

/**@brief decompose rssi frame to get intensity measurements;
*
*@return bool: if rssi frame is successfully decomposed, return true; else, return false;
*@note
*/
bool Lms511Thread::DecomposeRssiFrame()
{
    m_List.clear();
    m_List = m_RssiFrame.split(' ');
    m_Iterator = m_List.begin();//DIST

    m_Iterator++;//scaling factor
    m_TempArray = *m_Iterator;
    if(false == Str2Real(m_TempArray, &(m_lmsMeasurements.fRssiScalingFactor)))
    {
        return false;
    }

    m_Iterator++;//scaling offset;
    m_Iterator++;//starting angle;
    m_Iterator++;//angular step width;
    m_Iterator++;//number data;
    int i = 0;
    for(i=0; i<m_lmsMeasurements.nNumberData; i++)
    {
        m_Iterator++;
        Str2Uint_16(*m_Iterator, m_lmsMeasurements.pRssi+i);
    }
}

/**@brief publish ros laser scan message
*
*@return void;
*@note
*/
void Lms511Thread::PublishRosMsg ()
{
    ros::Time scan_time = ros::Time::now();
    m_laserScan.header.stamp = scan_time;
    m_laserScan.header.frame_id = m_strFrameID;

    Deg2Rad (m_lmsMeasurements.nAngularStepWidth*1.0, &(m_laserScan.angle_increment));
    m_laserScan.angle_increment /= 10000.0;
    Deg2Rad (m_lmsMeasurements.nStartingAngle*1.0-900000.0, &(m_laserScan.angle_min));
    m_laserScan.angle_min /= 10000.0;
    m_laserScan.angle_max = m_laserScan.angle_min + (m_lmsMeasurements.nNumberData-1)*m_laserScan.angle_increment;

    m_laserScan.time_increment = 1/(100.0*m_lmsMeasurements.nNumberData);
    m_laserScan.range_min = 0.50;
    m_laserScan.range_max = 65.0;
    m_laserScan.ranges.resize (m_lmsMeasurements.nNumberData);
    m_laserScan.intensities.resize (m_lmsMeasurements.nNumberData);
    int i=0;
    for(i=0; i<m_lmsMeasurements.nNumberData; i++)
    {
        //attentation: distance = distance*scalingFactor.
        m_laserScan.ranges[i] = m_lmsMeasurements.pDist[i]*m_lmsMeasurements.fDistScalingFactor/1000.0;
        //all the remission value is between 0~255, no need to multiple scaling factor.
        m_laserScan.intensities[i] = m_lmsMeasurements.pRssi[i]*1.0;
    }
    m_pubLms511.publish(m_laserScan);
    return;
}

/**@brief generate message command and message template of the lms laser;
*
*@param enumMsg [IN]: enumerator define message type to generate;
*@param pMsg [OUT]:store the message generated;
*@return bool, if the msg type is defined return true; else, return false;
*@note the last character of the msg is '\0';
*/
bool Lms511Thread::GenerateMsg(LmsMsg enumMsg, char* pMsg)
{
    switch (enumMsg)
    {
    case RequestStartMeasurement:
        pMsg[0] = 0x02;
        pMsg[1] = 's';
        pMsg[2] = 'M';
        pMsg[3] = 'N';
        pMsg[4] = ' ';
        pMsg[5] = 'L';
        pMsg[6] = 'M';
        pMsg[7] = 'C';
        pMsg[8] = 's';
        pMsg[9] = 't';
        pMsg[10] = 'a';
        pMsg[11] = 'r';
        pMsg[12] = 't';
        pMsg[13] = 'm';
        pMsg[14] = 'e';
        pMsg[15] = 'a';
        pMsg[16] = 's';
        pMsg[17] = 0x03;
        pMsg[18] = '\0';
        break;
    case AnswerStartMeasurement:
        pMsg[0] = 0x02;
        pMsg[1] = 's';
        pMsg[2] = 'A';
        pMsg[3] = 'N';
        pMsg[4] = ' ';
        pMsg[5] = 'L';
        pMsg[6] = 'M';
        pMsg[7] = 'C';
        pMsg[8] = 's';
        pMsg[9] = 't';
        pMsg[10] = 'a';
        pMsg[11] = 'r';
        pMsg[12] = 't';
        pMsg[13] = 'm';
        pMsg[14] = 'e';
        pMsg[15] = 'a';
        pMsg[16] = 's';
        pMsg[17] = ' ';
        pMsg[18] = '\0';
        break;
    case RequestStopMeasurement:
        pMsg[0] = 0x02;
        pMsg[1] = 's';
        pMsg[2] = 'M';
        pMsg[3] = 'N';
        pMsg[4] = ' ';
        pMsg[5] = 'L';
        pMsg[6] = 'M';
        pMsg[7] = 'C';
        pMsg[8] = 's';
        pMsg[9] = 't';
        pMsg[10] = 'o';
        pMsg[11] = 'p';
        pMsg[12] = 'm';
        pMsg[13] = 'e';
        pMsg[14] = 'a';
        pMsg[15] = 's';
        pMsg[16] = 0x03;
        pMsg[17] = '\0';
        break;
    case AnswerStopMeasurement:
        pMsg[0] = 0x02;
        pMsg[1] = 's';
        pMsg[2] = 'A';
        pMsg[3] = 'N';
        pMsg[4] = ' ';
        pMsg[5] = 'L';
        pMsg[6] = 'M';
        pMsg[7] = 'C';
        pMsg[8] = 's';
        pMsg[9] = 't';
        pMsg[10] = 'o';
        pMsg[11] = 'p';
        pMsg[12] = 'm';
        pMsg[13] = 'e';
        pMsg[14] = 'a';
        pMsg[15] = 's';
        pMsg[16] = ' ';
        pMsg[17] = '\0';
        break;
    case RequestQueryStatus:
        pMsg[0] = 0x02;
        pMsg[1] = 's';
        pMsg[2] = 'R';
        pMsg[3] = 'N';
        pMsg[4] = ' ';
        pMsg[5] = 'S';
        pMsg[6] = 'T';
        pMsg[7] = 'l';
        pMsg[8] = 'm';
        pMsg[9] = 's';
        pMsg[10] = 0x03;
        pMsg[11] = '\0';
        break;
    case AnswerQueryStatus:
        pMsg[0] = 0x02;
        pMsg[1] = 's';
        pMsg[2] = 'R';
        pMsg[3] = 'A';
        pMsg[4] = ' ';
        pMsg[5] = 'S';
        pMsg[6] = 'T';
        pMsg[7] = 'l';
        pMsg[8] = 'm';
        pMsg[9] = 's';
        pMsg[10] = ' ';
        pMsg[11] = '\0';
        break;
    case RequestSingleMeasurementOutput:
        pMsg[0] = 0x02;
        pMsg[1] = 's';
        pMsg[2] = 'R';
        pMsg[3] = 'N';
        pMsg[4] = ' ';
        pMsg[5] = 'L';
        pMsg[6] = 'M';
        pMsg[7] = 'D';
        pMsg[8] = 's';
        pMsg[9] = 'c';
        pMsg[10] = 'a';
        pMsg[11] = 'n';
        pMsg[12] = 'd';
        pMsg[13] = 'a';
        pMsg[14] = 't';
        pMsg[15] = 'a';
        pMsg[16] = 0x03;
        pMsg[17] = '\0';
        break;
    case AnswerSingleMeasurement:
        pMsg[0] = 0x02;
        pMsg[1] = 's';
        pMsg[2] = 'R';
        pMsg[3] = 'A';
        pMsg[4] = ' ';
        pMsg[5] = 'L';
        pMsg[6] = 'M';
        pMsg[7] = 'D';
        pMsg[8] = 's';
        pMsg[9] = 'c';
        pMsg[10] = 'a';
        pMsg[11] = 'n';
        pMsg[12] = 'd';
        pMsg[13] = 'a';
        pMsg[14] = 't';
        pMsg[15] = 'a';
        pMsg[16] = ' ';
        pMsg[17] = '\0';
        break;
    case RequestStartContinuousMeasurementOutput:
        pMsg[0] = 0x02;
        pMsg[1] = 's';
        pMsg[2] = 'E';
        pMsg[3] = 'N';
        pMsg[4] = ' ';
        pMsg[5] = 'L';
        pMsg[6] = 'M';
        pMsg[7] = 'D';
        pMsg[8] = 's';
        pMsg[9] = 'c';
        pMsg[10] = 'a';
        pMsg[11] = 'n';
        pMsg[12] = 'd';
        pMsg[13] = 'a';
        pMsg[14] = 't';
        pMsg[15] = 'a';
        pMsg[16] = ' ';
        pMsg[17] = '1';
        pMsg[18] = 0x03;
        pMsg[19] = '\0';
        break;
    case RequestStopContinuousMeasurementOutput:
        pMsg[0] = 0x02;
        pMsg[1] = 's';
        pMsg[2] = 'E';
        pMsg[3] = 'N';
        pMsg[4] = ' ';
        pMsg[5] = 'L';
        pMsg[6] = 'M';
        pMsg[7] = 'D';
        pMsg[8] = 's';
        pMsg[9] = 'c';
        pMsg[10] = 'a';
        pMsg[11] = 'n';
        pMsg[12] = 'd';
        pMsg[13] = 'a';
        pMsg[14] = 't';
        pMsg[15] = 'a';
        pMsg[16] = ' ';
        pMsg[17] = '0';
        pMsg[18] = 0x03;
        pMsg[19] = '\0';
        break;
    case AnswerContinuousMeasurement:
        pMsg[0] = 0x02;
        pMsg[1] = 's';
        pMsg[2] = 'S';
        pMsg[3] = 'N';
        pMsg[4] = ' ';
        pMsg[5] = 'L';
        pMsg[6] = 'M';
        pMsg[7] = 'D';
        pMsg[8] = 's';
        pMsg[9] = 'c';
        pMsg[10] = 'a';
        pMsg[11] = 'n';
        pMsg[12] = 'd';
        pMsg[13] = 'a';
        pMsg[14] = 't';
        pMsg[15] = 'a';
        pMsg[16] = ' ';
        pMsg[17] = '\0';
        break;
    case RequestReadFreqRes:
        pMsg[0] = 0x02;
        pMsg[1] = 's';
        pMsg[2] = 'R';
        pMsg[3] = 'N';
        pMsg[4] = ' ';
        pMsg[5] = 'L';
        pMsg[6] = 'M';
        pMsg[7] = 'P';
        pMsg[8] = 's';
        pMsg[9] = 'c';
        pMsg[10] = 'a';
        pMsg[11] = 'n';
        pMsg[12] = 'c';
        pMsg[13] = 'f';
        pMsg[14] = 'g';
        pMsg[15] = 0x03;
        pMsg[16] = '\0';
        break;
    case AnswerReadFreqRes:
        pMsg[0] = 0x02;
        pMsg[1] = 's';
        pMsg[2] = 'R';
        pMsg[3] = 'A';
        pMsg[4] = ' ';
        pMsg[5] = 'L';
        pMsg[6] = 'M';
        pMsg[7] = 'P';
        pMsg[8] = 's';
        pMsg[9] = 'c';
        pMsg[10] = 'a';
        pMsg[11] = 'n';
        pMsg[12] = 'c';
        pMsg[13] = 'f';
        pMsg[14] = 'g';
        pMsg[15] = ' ';
        pMsg[16] = '\0';
        break;
    case RequestReadContamination:
        pMsg[0] = 0x02;
        pMsg[1] = 's';
        pMsg[2] = 'R';
        pMsg[3] = 'N';
        pMsg[4] = ' ';
        pMsg[5] = 'L';
        pMsg[6] = 'C';
        pMsg[7] = 'M';
        pMsg[8] = 's';
        pMsg[9] = 't';
        pMsg[10] = 'a';
        pMsg[11] = 't';
        pMsg[12] = 'e';
        pMsg[13] = 0x03;
        pMsg[14] = '\0';
        break;
    case AnswerReadContamination:
        pMsg[0] = 0x02;
        pMsg[1] = 's';
        pMsg[2] = 'R';
        pMsg[3] = 'A';
        pMsg[4] = ' ';
        pMsg[5] = 'L';
        pMsg[6] = 'C';
        pMsg[7] = 'M';
        pMsg[8] = 's';
        pMsg[9] = 't';
        pMsg[10] = 'a';
        pMsg[11] = 't';
        pMsg[12] = 'e';
        pMsg[13] = ' ';
        pMsg[14] = '\0';
        break;
    default:
        return false;
    }
    return true;
}

/**@brief convert an angle from degree to rad;
*
*@param deg [IN]: angle in deg;
*@param rad [OUT]: angle in rad;
*@return void;
*@note
*/
void Lms511Thread::Deg2Rad(float deg, float *rad)
{
    (*rad) = deg*3.1415926/180.0;
    return;
}

/**@brief convert QString to short int (2 bytes, 16 bits);
*
*@param str [IN]: the string to convert;
*@param data [OUT]: the result is stored in this parm;
*@return bool: if convert succeed, return true; else, return false.
*@note
*/
bool Lms511Thread::Str2Int_16(QByteArray str, unsigned short int * data)
{
    bool ok;
    *data = str.toShort(&ok, 16);
    return ok;
}

/**@brief convert QString to int (4 bytes, 32 bits);
*
*@param str [IN]: the string to convert;
*@param data [OUT]: the result is stored in this parm;
*@return bool: if convert succeed, return true; else, return false;
*@note there is a bug in qt library, so hex->int16->int32.
*/
bool Lms511Thread::Str2Int_32(QByteArray str, int* data)
{
    bool ok;
    unsigned int temp = str.toUInt(&ok, 16);
    memcpy(data, &temp, sizeof(int));
    return ok;
}

/**@brief convert QString to unsigned short int (2 bytes, 16 bits);
*
*@param str [IN]: the string to convert;
*@param data [OUT]: the result is stored in this parm;
*@return bool, if convert succeed, return true; else, return false;
*@note
*/
bool Lms511Thread::Str2Uint_16(QByteArray str, unsigned short int *data)
{
    bool ok;
    *data = str.toUShort(&ok, 16);
    return ok;
}

/**@brief convert QString to unsigned int (4 bytes, 32 bits);
*
*@param str [IN]: the string to convert;;
*@param data [OUT]: the result is stored in this parm;
*@return bool, if convert succeed, return true; else, return false;
*@note
*/
bool Lms511Thread::Str2Uint_32(QByteArray str, unsigned int * data)
{
    bool ok;
    *data = str.toUInt(&ok, 16);
    return ok;
}


/**@brief convert QString to float (4 bytes, 32 bits);
*
*@param str [IN]: the string to convert;
*@param data [OUT]: the result is stored in this parm;
*@return bool: if convert succeed, return true; else, return false;
*@note hex->decimal->float
*/
bool Lms511Thread::Str2Real(QByteArray str, float* data)
{
    bool ok = false;
    int n = str.toInt(&ok, 16);
    memcpy(data, &n, sizeof(float));
    return ok;
}
