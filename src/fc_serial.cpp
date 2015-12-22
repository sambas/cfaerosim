#include "fc_serial.h"
#include "inttypes.h"
#include "enums.h"

#include <QSerialPort>

// May be moved in GlobalVariables.h
unsigned long previous_millis_low=0;
unsigned long previous_millis_high =0;
//----------------

uint32_t modeMSPRequests;
uint32_t queuedMSPRequests;

typedef enum {
    BOXARM = 0,
    BOXANGLE,
    BOXHORIZON,
    BOXNAVALTHOLD,  // old BOXBARO
    //BOXVARIO,
    BOXMAG,
    BOXHEADFREE,
    BOXHEADADJ,
    BOXCAMSTAB,
    BOXCAMTRIG,
    BOXNAVRTH,      // old GPSHOME
    BOXNAVPOSHOLD,  // old GPSHOLD
    BOXPASSTHRU,
    BOXBEEPERON,
    BOXLEDMAX,
    BOXLEDLOW,
    BOXLLIGHTS,
    //BOXCALIB,
    BOXGOV,
    BOXOSD,
    BOXTELEMETRY,
    BOXGTUNE,
    BOXSERVO1,
    BOXSERVO2,
    BOXSERVO3,
    BOXBLACKBOX,
    BOXFAILSAFE,
    BOXNAVWP,
    BOXAIRMODE,
    CHECKBOX_ITEM_COUNT
} boxId_e;




#define SERIALBUFFERSIZE 200
static uint8_t serialBuffer[SERIALBUFFERSIZE]; // this hold the imcoming string from serial O string
static uint8_t receiverIndex;
static uint8_t dataSize;
static uint8_t cmdMSP;
static uint8_t rcvChecksum;
static uint8_t readIndex;


uint32_t  FC_Serial::read32() {
  uint32_t t = (uint32_t)read16();
  t |= (uint32_t)read16()<<16;
  return t;
}

uint16_t  FC_Serial::read16() {
  uint16_t t = (uint16_t)read8();
  t |= (uint16_t)read8()<<8;
  return t;
}

uint8_t  FC_Serial::read8()  {
  return serialBuffer[readIndex++];
}

// --------------------------------------------------------------------------------------
// Here are decoded received commands from MultiWii
void  FC_Serial::serialMSPCheck()
{
//    qDebug() << "M: " << cmdMSP;
  readIndex = 0;

  if (cmdMSP==MSP_IDENT)
  {
    telem.MwVersion= read8();                             // MultiWii Firmware version
    modeMSPRequests &=~ REQ_MSP_IDENT;
  }
  if (cmdMSP==MSP_MIXER)
  {
    telem.Mixer= read8();                             // mixermode
    modeMSPRequests &=~ REQ_MSP_MIXER;
  }


  if (cmdMSP==MSP_STATUS)
  {
      telem.cycleTime=read16();
      telem.I2CError=read16();
    telem.MwSensorPresent = read16();
    telem.MwSensorActive = read32();
    telem.armed = (telem.MwSensorActive & 1);
    read8();
  }

  if (cmdMSP==MSP_RAW_IMU)
  {
      uint8_t i;
    for(i=0;i<3;i++)
        telem.MwAccSmooth[i] = read16();
  }
  if (cmdMSP==MSP_RC)
  {
      uint8_t i;
    for(i=0;i<8;i++)
        telem.input[i] = read16();
  }
  if (cmdMSP==MSP_HIL_STATE)
  {
    telem.input[0] = read16();
    telem.input[1] = read16();
    telem.input[2] = -read16(); // yaw is inverted here for some reason
    telem.input[3] = read16();
  }

  if (cmdMSP==MSP_RAW_GPS)
  {
      telem.GPS_fix=read8();
      telem.sats=read8();
      telem.lat = DEG2RAD*(((int32_t)read32())/10.0e6);
      telem.lon = DEG2RAD*(((int32_t)read32())/10.0e6);
      telem.galt = (int16_t)read16();

#if defined I2CGPS_SPEED
      telem.GPS_speed = read16()*10;
//    gpsfix(); untested
#else
    telem.gpsspeed = read16()/100.0;
#endif
    telem.MwHeading = ((int16_t)read16())/10.0;
  }

  if (cmdMSP==MSP_COMP_GPS)
  {
      telem.GPS_distanceToHome=read16();
#ifdef I2CGPS_DISTANCE
    gpsdistancefix();
#endif

    telem.GPS_directionToHome=read16();
    read8(); //missing
#ifdef GPSTIME
    GPS_time = read32();        //local time of coord calc - haydent
#endif
  }

  if (cmdMSP==MSP_NAV_STATUS)
  {
     read8();
     read8();
     read8();
     telem.GPS_waypoint_step=read8();
     read8();
     read8();
     read8();
  }

  if (cmdMSP==MSP_ATTITUDE)
  {
    //telem.Roll = ((int16_t)read16())/10.0f;
    //telem.Pitch = ((int16_t)read16())/-10.0f;
    //telem.Yaw = ((int16_t)read16());
#ifdef HEADINGCORRECT
    if (telem.MwHeading >= + 180) telem.MwHeading -= 360;
#endif
  }

  if (cmdMSP==MSP_DEBUG)
  {
      uint8_t i;
    for(i=0;i<3;i++)
      telem.debug[i] = read16();
  }

  if (cmdMSP==MSP_ALTITUDE)
  {
      telem.baroaltitude = ((int32_t)read32())/100.0f;
      telem.vD = ((int16_t)read16())/-100.0f;
  }

  if (cmdMSP==MSP_ANALOG)
  {
      telem.battery=read8()*100;
      telem.pMeterSum=read16();
      telem.MwRssi = read16();
      telem.MWAmperage = read16();
 #ifdef AMPERAGECORRECT
      telem.MWAmperage = telem.MWAmperage * 10;
#endif
 }

#ifdef BOXNAMES
  if(cmdMSP==MSP_BOXNAMES) {
    uint32_t bit = 1;
    uint8_t remaining = dataSize;
    uint8_t len = 0;
    char firstc, lastc;

    mode.armed = 0;
    mode.stable = 0;
    mode.baro = 0;
    mode.mag = 0;
    mode.gpshome = 0;
    mode.gpshold = 0;
    mode.llights = 0;
    mode.camstab = 0;
    mode.osd_switch = 0;

    while(remaining > 0) {
      char c = read8();
      if(len == 0)
        firstc = c;
      len++;
      if(c == ';') {
        // Found end of name; set bit if first and last c matches.
        if(firstc == 'A') {
          if(lastc == 'M') // "ARM;"
            mode.armed |= bit;
          if(lastc == 'E') // "ANGLE;"
            mode.stable |= bit;
        }
        if(firstc == 'H' && lastc == 'N') // "HORIZON;"
          mode.horizon |= bit;
        if(firstc == 'M' && lastc == 'G') // "MAG;"
           mode.mag |= bit;
        if(firstc == 'B' && lastc == 'O') // "BARO;"
          mode.baro |= bit;
        if(firstc == 'L' && lastc == 'S') // "LLIGHTS;"
          mode.llights |= bit;
        if(firstc == 'C' && lastc == 'B') // "CAMSTAB;"
          mode.camstab |= bit;
        if(firstc == 'G') {
          if(lastc == 'E') // "GPS HOME;"
            mode.gpshome |= bit;
          if(lastc == 'D') // "GPS HOLD;"
            mode.gpshold |= bit;
        }
        if(firstc == 'P' && lastc == 'U')  mode.passthru |= bit; // "Passthru;"
        if(firstc == 'O' && lastc == 'W') // "OSD SW;"
          mode.osd_switch |= bit;

        len = 0;
        bit <<= 1L;
      }
      lastc = c;
      --remaining;
    }
    modeMSPRequests &=~ REQ_MSP_BOX;
  }
#else
  if(cmdMSP==MSP_BOXIDS) {
    uint32_t bit = 1;
    uint8_t remaining = dataSize;

    /*mode.armed = 0;
    mode.stable = 0;
    mode.horizon = 0;
    mode.baro = 0;
    mode.mag = 0;
    mode.gpshome = 0;
    mode.gpshold = 0;
    mode.gpsmission = 0;
    mode.gpsland = 0;
    mode.llights = 0;
    mode.passthru = 0;
    mode.osd_switch = 0;
    mode.camstab = 0;

    while(remaining > 0) {
      char c = read8();
      switch(c) {
      case 0:
        mode.armed |= bit;
        break;
      case 1:
        mode.stable |= bit;
        break;
      case 2:
        mode.horizon |= bit;
        break;
      case 3:
        mode.baro |= bit;
        break;
      case 5:
        mode.mag |= bit;
        break;
      case 8:
        mode.camstab |= bit;
        break;
      case 10:
        mode.gpshome |= bit;
        break;
      case 11:
        mode.gpshold |= bit;
        break;
      case 12:
        mode.passthru  |= bit;
        break;
      case 16:
        mode.llights |= bit;
        break;
      case 19:
        mode.osd_switch |= bit;
        break;
      case 20:
        mode.gpsmission |= bit;
        break;
      case 21:
        mode.gpsland |= bit;
        break;
      }
      bit <<= 1;
      --remaining;
    }*/
    modeMSPRequests &=~ REQ_MSP_BOX;
  }
#endif
}
// End of decoded received commands from MultiWii
// --------------------------------------------------------------------------------------

void  FC_Serial::serialMSPreceive(uint8_t c)
{
  static enum _serial_state {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  } c_state = IDLE;

  {
    if (c_state == IDLE)
    {
      c_state = (c=='$') ? HEADER_START : IDLE;
    }
    else if (c_state == HEADER_START)
    {
      c_state = (c=='M') ? HEADER_M : IDLE;
    }
    else if (c_state == HEADER_M)
    {
      c_state = (c=='>') ? HEADER_ARROW : IDLE;
    }
    else if (c_state == HEADER_ARROW)
    {
      if (c > SERIALBUFFERSIZE)
      {  // now we are expecting the payload size
        c_state = IDLE;
      }
      else
      {
        dataSize = c;
        c_state = HEADER_SIZE;
        rcvChecksum = c;
      }
    }
    else if (c_state == HEADER_SIZE)
    {
      c_state = HEADER_CMD;
      cmdMSP = c;
      rcvChecksum ^= c;
      receiverIndex=0;
    }
    else if (c_state == HEADER_CMD)
    {
      rcvChecksum ^= c;
      if(receiverIndex == dataSize) // received checksum byte
      {
        if(rcvChecksum == 0) {
            serialMSPCheck();
        }
        c_state = IDLE;
      }
      else
        serialBuffer[receiverIndex++]=c;
    }
  }
}

QByteArray FC_Serial::blankserialRequest(uint8_t requestMSP)
{
    QByteArray data;
  data.append(headSerialRequest());
  data.append((char)0x00);
  data.append(requestMSP);
  data.append(requestMSP);
  return data;
}

QByteArray FC_Serial::headSerialRequest (void) {
    QByteArray data;
    data.append('$');
    data.append('M');
    data.append('<');
    return data;
}


///*
// * Transmitter thread.
// */
QByteArray FC_Serial::generateMSP() {


  {

      //---------------  Start Timed Service Routines  ---------------------------------------
      unsigned long currentMillis = time->elapsed();

      if((currentMillis - previous_millis_low) >= lo_speed_cycle)  // 10 Hz (Executed every 100ms)
      {
        previous_millis_low = previous_millis_low+lo_speed_cycle;
        return blankserialRequest(MSP_STATUS);

      }  // End of slow Timed Service Routine (100ms loop)

      if((currentMillis - previous_millis_high) >= hi_speed_cycle)  // 20 Hz (Executed every 50ms)
      {
        previous_millis_high = previous_millis_high+hi_speed_cycle;

          uint8_t MSPcmdsend=0;
          if(queuedMSPRequests == 0)
            queuedMSPRequests = modeMSPRequests;
          uint32_t req = queuedMSPRequests & -queuedMSPRequests;
          queuedMSPRequests &= ~req;
          //qDebug() << "C:" << req;
          switch(req) {
          case REQ_MSP_IDENT:
           MSPcmdsend = MSP_IDENT;
            break;
          case REQ_MSP_STATUS:
            MSPcmdsend = MSP_STATUS;
            break;
          case REQ_MSP_RAW_IMU:
            MSPcmdsend = MSP_RAW_IMU;
            break;
          case REQ_MSP_RC:
            MSPcmdsend = MSP_RC;
            break;
          case REQ_MSP_RAW_GPS:
            MSPcmdsend = MSP_RAW_GPS;
            break;
          case REQ_MSP_COMP_GPS:
            MSPcmdsend = MSP_COMP_GPS;
            break;
          case REQ_MSP_ATTITUDE:
            MSPcmdsend = MSP_ATTITUDE;
            break;
          case REQ_MSP_ALTITUDE:
            MSPcmdsend = MSP_ALTITUDE;
            break;
          case REQ_MSP_DEBUG:
             MSPcmdsend = MSP_DEBUG;
             break;
          case REQ_MSP_HIL_STATE:
             MSPcmdsend = MSP_HIL_STATE;
             break;
          case REQ_MSP_NAV_STATUS:
    //           if(MwSensorActive&mode.gpsmission)
             MSPcmdsend = MSP_NAV_STATUS;
          break;
        }
          return blankserialRequest(MSPcmdsend);
      }  // End of fast Timed Service Routine (50ms loop)
    return 0;
  }
}


void FC_Serial::setMspRequests() {
    modeMSPRequests =
      //REQ_MSP_IDENT|
      //REQ_MSP_STATUS|
      REQ_MSP_HIL_STATE
      //REQ_MSP_COMP_GPS|
      //REQ_MSP_ATTITUDE|
      //REQ_MSP_ALTITUDE|REQ_MSP_RAW_IMU
      //      REQ_MSP_DEBUG
            ;

      //modeMSPRequests |= REQ_MSP_RC;

  //modeMSPRequests |= REQ_MSP_ANALOG;


  // so we do not send requests that are not needed.
  queuedMSPRequests &= modeMSPRequests;
  qDebug() << "setMspRequests" << modeMSPRequests;
}

void  FC_Serial::serialize8(uint8_t a)
{
    requestData.append(a);
    checksum ^= a;
}

void  FC_Serial::serialize16(uint16_t a)
{
    serialize8((uint8_t)(a >> 0));
    serialize8((uint8_t)(a >> 8));
}

void  FC_Serial::serialize32(uint32_t a)
{
    serialize16((uint16_t)(a >> 0));
    serialize16((uint16_t)(a >> 16));
}


void  FC_Serial::headSerialResponse(uint8_t err, uint8_t responseBodySize, uint8_t cmdMSP)
{
    requestData.clear();
    serialize8('$');
    serialize8('M');
    serialize8(err ? '!' : '<');
    checksum = 0;               // start calculating a new checksum
    serialize8(responseBodySize);
    serialize8(cmdMSP);
}

void FC_Serial::headSerialReply(uint8_t responseBodySize, uint8_t cmdMSP)
{
    headSerialResponse(0, responseBodySize,cmdMSP);
}
void FC_Serial::headSerialError(uint8_t responseBodySize, uint8_t cmdMSP)
{
    headSerialResponse(1, responseBodySize,cmdMSP);
}

void  FC_Serial::tailSerialReply(void)
{
    serialize8(checksum);
}


QByteArray FC_Serial::mspProcessReceivedCommand(uint8_t cmdMSP) {
    if (!(processOutCommand(cmdMSP) )) {
        headSerialError(0,cmdMSP);
    }
    tailSerialReply();
    return requestData;
}

// into FC
bool FC_Serial::processOutCommand(uint8_t cmdMSP)
{
    switch (cmdMSP) {
    case MSP_SET_HIL_STATE:
        headSerialReply(16,cmdMSP);
        serialize16(telem.Roll);
        serialize16(telem.Pitch);
        serialize16(telem.Yaw);
        serialize32(telem.baroaltitude);
        serialize16(telem.X);
        serialize16(telem.Y);
        serialize16(telem.Z);
        break;

    case MSP_SET_RAW_GPS:
        headSerialReply(14,cmdMSP);
        serialize8(1);
        serialize8(telem.GPS_numSat);
        serialize32(telem.GPS_latitude);
        serialize32(telem.GPS_longitude);
        serialize16(telem.GPS_altitude);
        serialize16(telem.GPS_speed);
        break;

    case MSP_SET_RAW_RC:
        headSerialReply(16,cmdMSP);
        for(int i=0;i<8;i++)
        {
            serialize16(telem.MwRcData[i]);
        }
        break;

    default:
        return false;
    }
    return true;
}

FC_Serial::FC_Serial(const QList<quint8> map,
                     const QList<quint8> mapout,
                     bool isTX,
                     QObject *parent)
    : QThread(parent)
{
    stopped  = false;
    time = new QTime();
    time->start();

    qDebug() << this << "FC_Serial::FC_Serial thread:" << thread() << " ID: "<< QThread::currentThreadId();
    for (int i = 0; i < 8; ++i) {
        channels << 0.0;
    }
    channelsMapIn   = map;
    channelsMapOut   = mapout;
    takeFromTX    = isTX;
    packetsSended = 0;
    qDebug() << "channelmapin: "<< channelsMapIn;
    qDebug() << "channelmapout: "<< channelsMapOut;
}

FC_Serial::~FC_Serial()
{
}

// public
void FC_Serial::init(const QString &Cport)
{
    qDebug() << this << "FC_Serial::init";
    qDebug() << "init ID: "<< QThread::currentThreadId();


    port = new QSerialPort(Cport);
    connect(port, SIGNAL(readyRead()), this, SLOT(readReady()));


    port->setBaudRate(QSerialPort::Baud115200);
    port->setFlowControl(QSerialPort::NoFlowControl);
    port->setParity(QSerialPort::NoParity);
    port->setDataBits(QSerialPort::Data8);
    port->setStopBits(QSerialPort::OneStop);

    bool res = false;
    res = port->open(QIODevice::ReadWrite);
    timer = new QTimer();
    if(res){
       qDebug() << "Opened: " << Cport;
       setMspRequests();
       QByteArray data = blankserialRequest(MSP_IDENT);
       if(port->isOpen() && port->isWritable()){
           int err = port->write(data);
           port->waitForBytesWritten(-1);
           //qDebug() << "write err: " << err;
//               QString line;
//               for(int i=0;i<data.length();i++)
//                   line += QString::number((uint8_t)data.at(i)).append(" - ");
//               qDebug() << line;
       }
       // read mixer type
       data = blankserialRequest(MSP_MIXER);
       if(port->isOpen() && port->isWritable()){
           int err = port->write(data);
           port->waitForBytesWritten(-1);
           //qDebug() << "write err: " << err;
//               QString line;
//               for(int i=0;i<data.length();i++)
//                   line += QString::number((uint8_t)data.at(i)).append(" - ");
//               qDebug() << line;
       }
       connect(timer, SIGNAL(timeout()), this, SLOT(mspevent()));
       timer->start(10);
       qDebug("Timer started");

    }else{
       qDebug() << "Connection failed:" << port->errorString();
    }

}

void FC_Serial::mspevent(){
    //QMutexLocker locker(&mutex);
    //qDebug() << "mspevent";
    QByteArray data = generateMSP();
    if(data.length() == 0)
        return;
    QString line;
    for(int i=0;i<data.length();i++)
        line += QString::number((uint8_t)data.at(i)).append(" - ");
    //qDebug() << line;

    if(port->isOpen() && port->isWritable()){
        int err = port->write(data, data.length());
        port->waitForBytesWritten(-1);
        //qDebug() << "mspevent write: " << err;
        //qDebug() << data;
    }else{
        timer->stop();
        qDebug() << "Connection is not open for writing." << port->error();
    }

}


void FC_Serial::SendPacket(uint8_t cmdMSP)
{
    //QMutexLocker locker(&mutex);
    //qDebug() << "SendPacket ID: "<< QThread::currentThreadId();
    QByteArray data = mspProcessReceivedCommand(cmdMSP);
    if(port->isOpen() && port->isWritable()){
        int err = port->write(data);
        port->waitForBytesWritten(-1);
        //qDebug() << "write err: " << err;
            QString line;
            for(int i=0;i<data.length();i++)
                line += QString::number((uint8_t)data.at(i)).append(" - ");
            //qDebug() << line;
    }else{
        //timer->stop();
        qDebug() << "Connection is not open for writing." << port->error();
    }
}
void FC_Serial::FeedBytes(QByteArray data)
{
    for(int i=0;i<data.length();i++)
        serialMSPreceive(data[i]);
}

float wrap_3600(float angle)
{
    if (angle > 3600)
        angle -= 3600;
    if (angle < 0)
        angle += 3600;
    return angle;
}

void FC_Serial::sendDatagram(const simToPlugin *stp)
{
    QMutexLocker locker(&mutex);
    //qDebug() << "sendDatagram ID: "<< QThread::currentThreadId();

    // channels
    for (int i = 0; i < 8; ++i) {
        quint8 mapTo = channelsMapOut.at(i);
        if (mapTo == 255) { // unused channel
        } else if (takeFromTX) { // use values from simulators transmitter
            //qDebug() << "simtx: " << i << " " << stp->chSimTX[mapTo];
            telem.MwRcData[i] = (stp->chSimTX[mapTo]*500)+1500;
        } else { // direct use values from ESC/motors/ailerons/etc
            //qDebug() <<  "simrx: " << i << " " << stp->chSimRX[mapTo];
            telem.MwRcData[i] = (stp->chSimRX[mapTo]*500)+1500;
        }
    }

    // Rotate gravity to body frame and account for different axis convention in CF
    telem.X =  stp->accelYm * 100.0 + 981 * stp->axisYz;
    telem.Y = -stp->accelXm * 100.0 - 981 * stp->axisXz;
    telem.Z =  stp->accelZm * 100.0 + 981 * stp->axisZz;
    telem.Roll = -stp->roll*RAD2DEG*10;
    telem.Pitch = -stp->pitch*RAD2DEG*10;
    telem.Yaw = wrap_3600(stp->heading*RAD2DEG*10);
    telem.baroaltitude = stp->AGL*100;
    telem.GPS_latitude = stp->latitude*10e6;
    telem.GPS_longitude = stp->longitude*10e6;
    telem.GPS_altitude = stp->AGL;
    telem.GPS_speed = sqrt(pow(stp->velX,2) + pow(stp->velY,2))*100;
    telem.MwHeading = wrap_3600(stp->heading*RAD2DEG*10);
    telem.GPS_numSat = 10;


/*    telem.Pitch= (int)(telem.Pitch+1)%900;
    telem.Roll= (int)(telem.Roll+1)%900;
    telem.Yaw= (int)(telem.Yaw+1)%90;
    telem.X = 0;
    telem.Y = 0;
    telem.Z = 0;
*/

   /* qDebug() << "X " << telem.X;
    qDebug() << "Y " << telem.Y;
    qDebug() << "Z " << telem.Z;

    qDebug() << "rX " << telem.Roll;
    qDebug() << "rY " << telem.Pitch;
    qDebug() << "rZ " << telem.Yaw;

    qDebug() << "telem.GPS_latitude " << telem.GPS_latitude;
    qDebug() << "telem.GPS_longitude " << telem.GPS_longitude;*/

    SendPacket(MSP_SET_HIL_STATE);
    SendPacket(MSP_SET_RAW_GPS);

    // use simulator tx to fc
    if (takeFromTX)
    {
        SendPacket(MSP_SET_RAW_RC);
    }

    ++packetsSended;
}

void FC_Serial::run()
{
    qDebug() << "run ID: "<< QThread::currentThreadId();
    qDebug() << this << "FC_Serial::run start";
    while (!stopped) {
        //onReadyRead();
    }
    qDebug() << this << "FC_Serial::run ended";
}

void FC_Serial::stop()
{
    qDebug() << this << "FC_Serial::stop";
    stopped = true;
}

void FC_Serial::setChannels(pluginToSim *pts)
{
    QMutexLocker locker(&mutex);

    for (int i = 0; i < 10; i++) {
        quint8 mapTo = channelsMapIn.at(i);
        if (mapTo != 255) {
            int input = telem.input[i];
            //qDebug() << "i" << i << " " << input;
            float inp;
            if(i==3)
                inp = (input - 1500)/500.0;
            else
                inp = (input)/500.0;
            inp=inp/0.7;
            float channelValue = qBound(-1.0f, inp, 1.0f);
            //if(telem.Mixer!=MIXER_FLYING_WING && telem.Mixer!=MIXER_AIRPLANE) // multi
            {
                // replace simulators transmitter
                pts->chNewTX[mapTo]  = channelValue;
                pts->chOverTX[mapTo] = true;
            }/*
            else
            {
                // direct connect to ESC/motors/ailerons/etc
                pts->chNewRX[mapTo]  = channelValue;
                pts->chOverRX[mapTo] = true;
            }*/
        }
    }
}

void FC_Serial::getFlightStatus(quint8 &arm, quint32 &mod, quint8 &mixer)
{
    QMutexLocker locker(&mutex);

    arm = telem.armed==1?2:0;
    mod = 0;

    //qDebug() << telem.MwSensorActive << " " << (1<<BOXANGLE) << " " << (1<<BOXNAVPOSHOLD) << " " << (1<<BOXNAVRTH);
    if(telem.MwSensorActive & (1<<BOXANGLE))
    {
       mod |= 1;
    }
    if(telem.MwSensorActive & (1<<BOXNAVPOSHOLD))
    {
       mod |= 2;
    }
    if(telem.MwSensorActive & (1<<BOXNAVRTH))
    {
       mod |= 4;
    }
    //mod = telem.MwSensorActive;
    mixer = telem.Mixer;
}

// Parse the frame and send it
// for processing.
void FC_Serial::readReady(){
    //qDebug() << "readReady1";
    //QSerialPort *port = qobject_cast<QSerialPort *> (sender());
    // Read into a byte array.
    QByteArray data = port->readAll();

    //qDebug() << "received: " + data.length();
    FeedBytes(data);
}

