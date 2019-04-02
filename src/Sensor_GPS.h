#ifndef SENSOR_GPS_H
#define SENSOR_GPS_H

#include "t_serial.h"
#include "CoordinateConv.h"
#include "stdio.h"

#define INIT 0
#define PREV 1
#define CURR 2

class Sensor_GPS
{

public:
    Sensor_GPS();
    virtual ~Sensor_GPS();

    t_serial serial;

    bool data_acquisition;

    bool m_boolUpdateDataFlag;
    void SetStatusUpdateData(bool boolOnOff) { m_boolUpdateDataFlag = boolOnOff; }
    bool GetStatusUpdateData(void) { return m_boolUpdateDataFlag; }

    bool detect_GGA;
    bool detect_RMC;

    unsigned int m_dwordCounterChecksumPass;
    unsigned int m_dwordCounterChecksumFail;

    bool m_boolHeaderDetectFlag;
    void SetStatusHeaderDetect(bool boolOnOff) { m_boolHeaderDetectFlag = boolOnOff; }
    bool GetStatusHeaderDetect(void) { return m_boolHeaderDetectFlag; }

    int m_iRawDataIndex;

    bool HandlingDataGPS();

    int m_iTimeHour;
    int m_iTimeMinute;
    double m_dTimeSecond;

    int m_iQualityIndicator;
    int m_iNumSatelliteinUse;
    double m_dHDOP;

    double m_adLatitude[3];
    double m_adLongitude[3];

    char m_acLatitudeDirection[3];
    char m_acLongitudeDirection[3];

    double m_adAntennaAltitude[3];
    double m_adGeoidalSeparation[3];
    double m_adAltitude[3];

    double m_adLatitudeDelta[3];
    double m_adLongitudeDelta[3];
    double m_adDistanceDelta[3];

    BYTE m_abyteRawData[90];
    char m_acCopiedRawData[90];

    int iChecksum ;
    int iCandisum ;

    void Initialize(void);

    void InterpretGeneral(void);
    BOOL CalcCheckSum(void);
    void ExtractData(unsigned char byteCurrent);

    //QString m_sPacket;

    DWORD CheckBYTEXOR(BYTE byteCompared1, BYTE byteCompared2);
    void SetInitial(void);
    void ShiftData(void);
    void ManipulateData(void);

public:
    // Variable for GxRMC
    int m_iTimeHourRMC;
    int m_iTimeMinuteRMC;
    double m_dTimeSecondRMC;

    char m_acRMCStatus;
    double m_adLatitudeRMC[3];
    double m_adLongitudeRMC[3];

    char m_acLatitudeDirectionRMC[3];
    char m_acLongitudeDirectionRMC[3];

    double m_adSpeed_Knots;
    double m_adCourseOverGround_deg;


public:
    // Variable for coordinate conversion
    double m_dGPS_Lat;
    double m_dGPS_Long;

    double m_dGPS_UTM_X;
    double m_dGPS_UTM_Y;

    double m_dGPS_UTM_X_Init;
    double m_dGPS_UTM_Y_Init;

    double m_dGPS_UTM_X_Rel;
    double m_dGPS_UTM_Y_Rel;

    CoordinateConv m_CoordConv;

    void CoordinateConv_GPS(void);

    void SetCurrentGPSLat(double dGPS) { m_dGPS_Lat = dGPS; }
    void SetCurrentGPSLong(double dGPS) { m_dGPS_Long = dGPS; }
    double GetCurrentGPSLat(void) { return m_dGPS_Lat; }
    double GetCurrentGPSLong(void) { return m_dGPS_Long; }

    double GetCurrentGPS_X(void) { return m_dGPS_UTM_X; }
    double GetCurrentGPS_Y(void) { return m_dGPS_UTM_Y; }

    void SetInitGPS_UTM_X(double dUTM) { m_dGPS_UTM_X_Init = dUTM; }
    void SetInitGPS_UTM_Y(double dUTM) { m_dGPS_UTM_Y_Init = dUTM; }
    double GetInitGPS_UTM_X(void) { return m_dGPS_UTM_X_Init; }
    double GetInitGPS_UTM_Y(void) { return m_dGPS_UTM_Y_Init; }

    double GetGPS_UTM_X_Rel(void) { return m_dGPS_UTM_X_Rel; }
    double GetGPS_UTM_Y_Rel(void) { return m_dGPS_UTM_Y_Rel; }


};

#endif // SENSOR_GPS_H
