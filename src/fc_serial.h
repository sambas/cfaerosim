#ifndef FC_SERIAL_H
#define FC_SERIAL_H
#include "inttypes.h"

#include <QCoreApplication>
#include <QByteArray>
#include <QSerialPort>
#include <QTime>
#include "aerosimrcdatastruct.h"

const float RAD2DEG = (float)(180.0 / M_PI);
const float DEG2RAD = (float)(M_PI / 180.0);

typedef struct TELEM{
    volatile uint16_t   battery; // mV
    volatile uint16_t   bec; // mV
    volatile float		baroaltitude; // m
    volatile double homelat; // rad
    volatile double homelon; // rad
    volatile float galt; // m
    volatile double lat;	// rad
    volatile double lon;	// rad
    volatile int sats;
    volatile float gpsspeed; // from ned m/s
    volatile float X;
    volatile float Y;
    volatile float Z;
    volatile float Roll;	// deg
    volatile float Pitch;	// deg
    volatile float Yaw;		// deg
    volatile float vN; // m/s
    volatile float vE; // m/s
    volatile float vD; // m/s
    volatile double homedistance; // m
    volatile double homebearing; // deg
    volatile uint8_t status1;
    volatile uint8_t status2;
    volatile uint8_t status3;
    volatile uint8_t fmode;
    volatile int16_t input[20];
    volatile int16_t th_raw;
    volatile int16_t roll_raw;
    volatile int16_t pitch_raw;
    volatile int16_t yaw_raw;

    volatile uint16_t  MwSensorPresent;
    volatile uint32_t  MwSensorActive;
    volatile uint8_t MwVersion;
    volatile uint8_t MwVBat;
    volatile int16_t MwVario;
    volatile uint8_t armed;
    volatile uint8_t previousarmedstatus;  // for statistics after disarming
    volatile uint16_t armedangle;           // for capturing direction at arming
    volatile uint16_t GPS_distanceToHome;
    volatile uint8_t GPS_fix;
    volatile int32_t GPS_latitude;
    volatile int32_t GPS_longitude;
    volatile int16_t GPS_altitude;
    volatile uint16_t GPS_speed;
    volatile uint16_t old_GPS_speed;
    volatile int16_t GPS_directionToHome;
    volatile uint8_t GPS_numSat;
    volatile uint8_t GPS_waypoint_step;
    //volatile uint16_t I2CError=0;
    //volatile uint16_t cycleTime=0;
    volatile uint16_t pMeterSum;	//mAh
    volatile uint16_t MwRssi;
    volatile uint32_t GPS_time;        //local time of coord calc - haydent

    volatile uint16_t cycleTime;
    volatile uint16_t I2CError;
    volatile uint16_t Mixer;

    volatile uint16_t  MwAccSmooth[3];       // Those will hold Accelerator data
    volatile int32_t  MwAltitude;                         // This hold barometric value
    volatile int32_t  old_MwAltitude;                     // This hold barometric value


    volatile int MwAngle[2];           // Those will hold Accelerator Angle
    volatile uint16_t MwRcData[8];   // This hold receiver pulse signal
    volatile int16_t MwHeading;
    volatile uint16_t MWAmperage; // 1/100 A
    volatile uint16_t debug[4];
} TELEMETRY;

extern TELEMETRY telem;


// Note: this is called MultiType/MULTITYPE_* in baseflight.
typedef enum mixerMode
{
    MIXER_TRI = 1,
    MIXER_QUADP = 2,
    MIXER_QUADX = 3,
    MIXER_BICOPTER = 4,
    MIXER_GIMBAL = 5,
    MIXER_Y6 = 6,
    MIXER_HEX6 = 7,
    MIXER_FLYING_WING = 8,
    MIXER_Y4 = 9,
    MIXER_HEX6X = 10,
    MIXER_OCTOX8 = 11,
    MIXER_OCTOFLATP = 12,
    MIXER_OCTOFLATX = 13,
    MIXER_AIRPLANE = 14,        // airplane / singlecopter / dualcopter (not yet properly supported)
    MIXER_HELI_120_CCPM = 15,
    MIXER_HELI_90_DEG = 16,
    MIXER_VTAIL4 = 17,
    MIXER_HEX6H = 18,
    MIXER_PPM_TO_SERVO = 19,    // PPM -> servo relay
    MIXER_DUALCOPTER = 20,
    MIXER_SINGLECOPTER = 21,
    MIXER_ATAIL4 = 22,
    MIXER_CUSTOM = 23,
    MIXER_CUSTOM_AIRPLANE = 24,
    MIXER_CUSTOM_TRI = 25
} mixerMode_e;

#define VERSION 231


// ---------------------------------------------------------------------------------------
// Defines imported from Multiwii Serial Protocol MultiWii_shared svn r1337
#define MSP_VERSION              0

//to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
#define MSP_API_VERSION                 1    //out message
#define MSP_FC_VARIANT                  2    //out message
#define MSP_FC_VERSION                  3    //out message
#define MSP_BOARD_INFO                  4    //out message
#define MSP_BUILD_INFO                  5    //out message

#define MSP_SET_HIL_STATE               11    // in message  roll , pitch , trueHeading, baroAltitude, accel forward, right, up
#define MSP_HIL_STATE                   12    // out message

//
// MSP commands for Cleanflight original features
//
#define MSP_MODE_RANGES                 34    //out message         Returns all mode ranges
#define MSP_SET_MODE_RANGE              35    //in message          Sets a single mode range

#define MSP_FEATURE                     36
#define MSP_SET_FEATURE                 37

#define MSP_BOARD_ALIGNMENT             38
#define MSP_SET_BOARD_ALIGNMENT         39

#define MSP_CURRENT_METER_CONFIG        40
#define MSP_SET_CURRENT_METER_CONFIG    41

#define MSP_MIXER                       42
#define MSP_SET_MIXER                   43

#define MSP_RX_CONFIG                   44
#define MSP_SET_RX_CONFIG               45

#define MSP_LED_COLORS                  46
#define MSP_SET_LED_COLORS              47

#define MSP_LED_STRIP_CONFIG            48
#define MSP_SET_LED_STRIP_CONFIG        49

#define MSP_RSSI_CONFIG                 50
#define MSP_SET_RSSI_CONFIG             51

#define MSP_ADJUSTMENT_RANGES           52
#define MSP_SET_ADJUSTMENT_RANGE        53

// private - only to be used by the configurator, the commands are likely to change
#define MSP_CF_SERIAL_CONFIG            54
#define MSP_SET_CF_SERIAL_CONFIG        55

#define MSP_VOLTAGE_METER_CONFIG        56
#define MSP_SET_VOLTAGE_METER_CONFIG    57

#define MSP_SONAR_ALTITUDE              58 //out message get sonar altitude [cm]

#define MSP_PID_CONTROLLER              59
#define MSP_SET_PID_CONTROLLER          60

#define MSP_ARMING_CONFIG               61 //out message         Returns auto_disarm_delay and disarm_kill_switch parameters
#define MSP_SET_ARMING_CONFIG           62 //in message          Sets auto_disarm_delay and disarm_kill_switch parameters

#define MSP_DATAFLASH_SUMMARY           70 //out message - get description of dataflash chip
#define MSP_DATAFLASH_READ              71 //out message - get content of dataflash chip
#define MSP_DATAFLASH_ERASE             72 //in message - erase dataflash chip

#define MSP_LOOP_TIME                   73 //out message         Returns FC cycle time i.e looptime parameter
#define MSP_SET_LOOP_TIME               74 //in message          Sets FC cycle time i.e looptime parameter

#define MSP_FAILSAFE_CONFIG             75 //out message         Returns FC Fail-Safe settings
#define MSP_SET_FAILSAFE_CONFIG         76 //in message          Sets FC Fail-Safe settings

#define MSP_RXFAIL_CONFIG               77 //out message         Returns RXFAIL settings
#define MSP_SET_RXFAIL_CONFIG           78 //in message          Sets RXFAIL settings

//
// Baseflight MSP commands (if enabled they exist in Cleanflight)
//
#define MSP_RX_MAP                      64 //out message get channel map (also returns number of channels total)
#define MSP_SET_RX_MAP                  65 //in message set rx map, numchannels to set comes from MSP_RX_MAP

// FIXME - Provided for backwards compatibility with configurator code until configurator is updated.
// DEPRECATED - DO NOT USE "MSP_BF_CONFIG" and MSP_SET_BF_CONFIG.  In Cleanflight, isolated commands already exist and should be used instead.
#define MSP_BF_CONFIG                   66 //out message baseflight-specific settings that aren't covered elsewhere
#define MSP_SET_BF_CONFIG               67 //in message baseflight-specific settings save

#define MSP_REBOOT                      68 //in message reboot settings

// DEPRECATED - Use MSP_BUILD_INFO instead
#define MSP_BF_BUILD_INFO               69 //out message build date as well as some space for future expansion

//
// Multwii original MSP commands
//

// DEPRECATED - See MSP_API_VERSION and MSP_MIXER
#define MSP_IDENT                100    //out message         mixerMode + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes
#define MSP_NAV_STATUS           121   //out message	     Returns navigation status

#define MSP_CELLS                130   //out message         FrSky SPort Telemtry

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction

#define MSP_BIND                 240   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4
// End of imported defines from Multiwii Serial Protocol MultiWii_shared svn r1333
// ---------------------------------------------------------------------------------------

// Private MSP for use with the GUI
#define MSP_OSD                  220   //in message          starts epprom send to OSD GUI
// Subcommands
#define OSD_NULL                 0
#define OSD_READ_CMD             1
#define OSD_WRITE_CMD            2
#define OSD_GET_FONT             3
#define OSD_SERIAL_SPEED         4
#define OSD_RESET                5
#define OSD_DEFAULT              6
#define OSD_SENSORS              7
// End private MSP for use with the GUI



#define REQ_MSP_IDENT     (1 <<  0)
#define REQ_MSP_STATUS    (1 <<  1)
#define REQ_MSP_RAW_IMU   (1 <<  2)
#define REQ_MSP_RC        (1 <<  3)
#define REQ_MSP_RAW_GPS   (1 <<  4)
#define REQ_MSP_COMP_GPS  (1 <<  5)
#define REQ_MSP_ATTITUDE  (1 <<  16)
#define REQ_MSP_ALTITUDE  (1 <<  17)
#define REQ_MSP_ANALOG    (1 <<  8)
#define REQ_MSP_RC_TUNING (1 <<  9)
#define REQ_MSP_PID       (1 << 10)
#define REQ_MSP_BOX       (1 << 11)
#define REQ_MSP_FONT      (1 << 12)
#define REQ_MSP_DEBUG     (1 << 13)
#define REQ_MSP_CELLS     (1 << 14)
#define REQ_MSP_NAV_STATUS  32768 //(1 << 15)
#define REQ_MSP_HIL_STATE (1 << 6)
#define REQ_MSP_MIXER       (1 << 7)



#define hi_speed_cycle  20
#define lo_speed_cycle  1000




class FC_Serial: public QThread {
    Q_OBJECT

public:
    FC_Serial(const QList<quint8> map, const QList<quint8> mapout, bool isTX, QObject *parent = 0);
    ~FC_Serial();
    void sendDatagram(const simToPlugin *stp);
    quint32 pcks()
    {
        return packetsSended;
    }

    void init(const QString &Cport);
    void run();
    void stop();
    // function getChannels for other threads
    void setChannels(pluginToSim *pts);
    void getFlightStatus(quint8 &arm, quint32 &mod, quint8 &mixer);
    quint8 getArmed()
    {
        return armed;
    }
    quint8 getMode()
    {
        return mode;
    }
    quint32 pckr()
    {
        return packetsRecived;
    }

private:
    QSerialPort * port;
    QTimer *timer;
    QTime* time;
    QByteArray requestData;
    char checksum;
    TELEMETRY telem;
    quint16 outPort;
    QList<float> channels;
    QList<quint8> channelsMapIn;
    QList<quint8> channelsMapOut;
    bool takeFromTX;
    quint32 packetsSended;

    volatile bool stopped;
    QMutex mutex;
    bool sendToRX;
    quint8 armed;
    quint8 mode;
    quint32 packetsRecived;

    void FeedBytes(QByteArray data);
    void setMspRequests();
    QByteArray generateMSP();
    QByteArray blankserialRequest(uint8_t requestMSP);
    QByteArray headSerialRequest (void);
    void SendPacket(uint8_t cmdMSP);
    bool processOutCommand(uint8_t cmdMSP);
    QByteArray mspProcessReceivedCommand(uint8_t cmdMSP);

    uint32_t  read32();
    uint16_t  read16();
    uint8_t  read8();
    void serialMSPCheck();
    void serialMSPreceive(uint8_t c);
    void serialize8(uint8_t a);
    void serialize16(uint16_t a);
    void serialize32(uint32_t a);
    void headSerialResponse(uint8_t err, uint8_t responseBodySize, uint8_t cmdMSP);
    void headSerialReply(uint8_t responseBodySize, uint8_t cmdMSP);
    void headSerialError(uint8_t responseBodySize, uint8_t cmdMSP);
    void tailSerialReply(void);

public slots:
    void readReady();
    void mspevent();

};

#endif // FC_SERIAL_H
