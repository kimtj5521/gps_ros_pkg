#include "Sensor_GPS.h"

Sensor_GPS::Sensor_GPS()
{
    Initialize();
    SetInitial();
}

Sensor_GPS::~Sensor_GPS()
{
}

void Sensor_GPS::Initialize(void)
{
    m_dwordCounterChecksumPass = 0;
    m_dwordCounterChecksumFail = 0;

    m_iTimeHour = 0;
    m_iTimeMinute = 0;
    m_dTimeSecond = 0.0;

    m_iQualityIndicator = 0;
    m_iNumSatelliteinUse = 0;
    m_dHDOP = 0.0;

    memset(m_adLatitude,0,sizeof(m_adLatitude));
    memset(m_adLongitude,0,sizeof(m_adLongitude));

    memset(m_acLatitudeDirection,'x',sizeof(m_acLatitudeDirection));
    memset(m_acLongitudeDirection,'x',sizeof(m_acLongitudeDirection));

    memset(m_adAntennaAltitude,0,sizeof(m_adAntennaAltitude));
    memset(m_adGeoidalSeparation,0,sizeof(m_adGeoidalSeparation));
    memset(m_adAltitude,0,sizeof(m_adAltitude));

    memset(m_adLatitudeDelta,0,sizeof(m_adLatitudeDelta));
    memset(m_adLongitudeDelta,0,sizeof(m_adLongitudeDelta));
    memset(m_adDistanceDelta,0,sizeof(m_adDistanceDelta));

    memset(m_abyteRawData,0,sizeof(m_abyteRawData));
    memset(m_acCopiedRawData,0,sizeof(m_acCopiedRawData));

    m_dGPS_UTM_X_Init = 0.0;
    m_dGPS_UTM_Y_Init = 0.0;

    m_dGPS_UTM_X_Rel = 0.0;
    m_dGPS_UTM_Y_Rel = 0.0;

    //m_sPacket = "";

    SetStatusHeaderDetect(false);
    data_acquisition = false;
    SetStatusUpdateData(false);
    detect_GGA = false;
    detect_RMC = false;

    m_iRawDataIndex = 0;
}

void Sensor_GPS::ExtractData(unsigned char byteCurrent)
{
    if(GetStatusHeaderDetect() == false){
        if(byteCurrent == 0x24){	// mark $
            SetStatusHeaderDetect(true);
            memset(m_abyteRawData,0,sizeof(m_abyteRawData));
            m_abyteRawData[0] = byteCurrent;
            m_iRawDataIndex = 1;
        }
    }
    else{
        m_abyteRawData[m_iRawDataIndex++] = byteCurrent;
        if(byteCurrent == 0x0D){ // 0x0d = CR, 0x0A = LF
            for(int i = 0; i < m_iRawDataIndex; i++) m_acCopiedRawData[i] = m_abyteRawData[i];
            data_acquisition = true;
            SetStatusHeaderDetect(false);
        }
        if(m_iRawDataIndex > 90) SetStatusHeaderDetect(false);
    }
}

bool Sensor_GPS::HandlingDataGPS(){
    if(CalcCheckSum() == TRUE){
        InterpretGeneral();
        SetStatusUpdateData(true);
        m_dwordCounterChecksumPass++;
    }
    else {
        m_dwordCounterChecksumFail++;
        return false;
    }
    return true;
}

void Sensor_GPS::InterpretGeneral(void)
{
    //QString sData;
    //for(int i=0; i<m_iRawDataIndex; i++){
    //    sData= QString("%1").arg(m_acCopiedRawData[i]);
    //    m_sPacket += sData;
    //}

    ShiftData();

    char cBranch;
    double dTime;
    char cTemp;

    sscanf(&m_acCopiedRawData[3],"%c",&cBranch);
    if(cBranch == 'G'){
        sscanf(&m_acCopiedRawData[7],"%lf,%lf,%c,%lf,%c,%d,%d,%lf,%lf,%c,%lf,%c",
            &dTime,
            &m_adLatitude[CURR], &m_acLatitudeDirection[CURR],
            &m_adLongitude[CURR], &m_acLongitudeDirection[CURR],
            &m_iQualityIndicator, &m_iNumSatelliteinUse,
            &m_dHDOP,
            &m_adAntennaAltitude[CURR], &cTemp,
            &m_adGeoidalSeparation[CURR], &cTemp);

        m_iTimeHour = (int) (dTime / 10000);
        m_iTimeMinute = (int) ((dTime - m_iTimeHour * 10000) / 100);
        m_dTimeSecond = (double) (dTime - m_iTimeHour * 10000 - m_iTimeMinute * 100);
    }
    else if(cBranch == 'R'){
        sscanf(&m_acCopiedRawData[7],"%lf,%c,%lf,%c,%lf,%c,%lf,%lf",
            &dTime,
            &m_acRMCStatus,
            &m_adLatitudeRMC[CURR], &m_acLatitudeDirectionRMC[CURR],
            &m_adLongitudeRMC[CURR], &m_acLongitudeDirectionRMC[CURR],
            &m_adSpeed_Knots, &m_adCourseOverGround_deg);

        m_iTimeHourRMC = (int) (dTime / 10000);
        m_iTimeMinuteRMC = (int) ((dTime - m_iTimeHourRMC * 10000) / 100);
        m_dTimeSecondRMC = (double) (dTime - m_iTimeHourRMC * 10000 - m_iTimeMinuteRMC * 100);
    }

    ManipulateData();

    SetStatusUpdateData(true);
}

BOOL Sensor_GPS::CalcCheckSum(void)
{
    sscanf(&m_acCopiedRawData[m_iRawDataIndex-3],"%x",&iChecksum);

    for(int i = 1; i < m_iRawDataIndex - 5; i++){
        if(i == 1) iCandisum = CheckBYTEXOR(m_acCopiedRawData[i], m_acCopiedRawData[i+1]);
        else iCandisum = CheckBYTEXOR(iCandisum, m_acCopiedRawData[i+1]);
    }

    if(iCandisum == iChecksum) return true;
    else return false;
}

DWORD Sensor_GPS::CheckBYTEXOR(BYTE byteCompared1, BYTE byteCompared2)
{
    BYTE byteMask = 0x80;
    BYTE byteResult = 0;

    for(int i = 0; i < 8; i++){
        byteResult <<= 1;

        if((byteCompared1 & byteMask) == (byteCompared2 & byteMask)) byteResult |= 0x0;
        else byteResult |= 0x1;

        byteMask >>= 1;
    }

    return byteResult;
}

void Sensor_GPS::SetInitial(void)
{
    m_adLatitude[INIT] = m_adLatitude[CURR];
    m_acLatitudeDirection[INIT] = m_acLatitudeDirection[CURR];

    m_adLongitude[INIT] = m_adLongitude[CURR];
    m_acLongitudeDirection[INIT] = m_acLongitudeDirection[CURR];

    m_adAntennaAltitude[INIT] = m_adAntennaAltitude[CURR];
    m_adGeoidalSeparation[INIT] = m_adGeoidalSeparation[CURR];
    m_adAltitude[INIT] = m_adAltitude[CURR];

    m_adLatitudeDelta[INIT] = m_adLatitudeDelta[CURR];
    m_adLongitudeDelta[INIT] = m_adLongitudeDelta[CURR];
    m_adDistanceDelta[INIT] = m_adDistanceDelta[CURR];
}

void Sensor_GPS::ShiftData(void)
{
    m_adLatitude[PREV] = m_adLatitude[CURR];
    m_acLatitudeDirection[PREV] = m_acLatitudeDirection[CURR];

    m_adLongitude[PREV] = m_adLongitude[CURR];
    m_acLongitudeDirection[PREV] = m_acLongitudeDirection[CURR];

    m_adAntennaAltitude[PREV] = m_adAntennaAltitude[CURR];
    m_adGeoidalSeparation[PREV] = m_adGeoidalSeparation[CURR];
    m_adAltitude[PREV] = m_adAltitude[CURR];

    m_adLatitudeDelta[PREV] = m_adLatitudeDelta[CURR];
    m_adLongitudeDelta[PREV] = m_adLongitudeDelta[CURR];
    m_adDistanceDelta[PREV] = m_adDistanceDelta[CURR];
}

void Sensor_GPS::ManipulateData(void)
{
    m_adLatitudeDelta[CURR] = m_adLatitude[INIT] - m_adLatitude[CURR];
    m_adLongitudeDelta[CURR] = m_adLongitude[INIT] - m_adLongitude[CURR];
}

void Sensor_GPS::CoordinateConv_GPS(void)
{
    double dMDVGPS_DDLAT;
    double dMDVGPS_DDLON;

    // DM -> DD
    m_CoordConv.GPSWGS84_DM2DD(m_dGPS_Lat, m_dGPS_Long);
    dMDVGPS_DDLAT = m_CoordConv.dWGS84_DDLAT;
    dMDVGPS_DDLON = m_CoordConv.dWGS84_DDLON;

    // DD -> UTM
    m_CoordConv.WGS2UTM(dMDVGPS_DDLAT, dMDVGPS_DDLON);
    m_dGPS_UTM_X = m_CoordConv.dUTM_X;
    m_dGPS_UTM_Y = m_CoordConv.dUTM_Y;

    m_dGPS_UTM_X_Rel = m_dGPS_UTM_X - m_dGPS_UTM_X_Init;
    m_dGPS_UTM_Y_Rel = m_dGPS_UTM_Y - m_dGPS_UTM_Y_Init;
}
