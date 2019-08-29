#include <ros/ros.h>
#include "t_serial.h"
#include "Sensor_GPS.h"
#include "std_msgs/Float64MultiArray.h"
#include <iostream>
#include <iomanip>

using namespace std;

Sensor_GPS m_gps;

ros::Publisher gps_pub;
std_msgs::Float64MultiArray gps_data;

int m_iGpsIndex;

void OnReceiveGPS(void)
{
    int n = m_gps.serial.GetLength();
    unsigned char *pBuffer = m_gps.serial.GetBuffer();

    if(n>=90){
        for(int i=0; i<n; ++i){
            if(pBuffer[i]=='$' && pBuffer[i+3]=='G' && pBuffer[i+4]=='G' && pBuffer[i+5]=='A'){
                m_gps.detect_GGA = true;
                m_iGpsIndex = 0;
            }
            if(m_gps.detect_GGA) {
                m_gps.ExtractData(pBuffer[i]);
                if(m_gps.data_acquisition){
                    m_gps.serial.Reset();
                    m_gps.HandlingDataGPS();
                    m_gps.data_acquisition = false;
                    m_gps.detect_GGA = false;
                    break;
                }
                ++m_iGpsIndex;
                if(m_iGpsIndex>90) {
                    m_gps.detect_GGA = false;
                    m_gps.serial.Reset();
                }
            }
        }
        for(int j=0; j<n; ++j){
            if(pBuffer[j]=='$' && pBuffer[j+3]=='R' && pBuffer[j+4]=='M' && pBuffer[j+5]=='C'){
                m_gps.detect_RMC = true;
                m_iGpsIndex = 0;
            }
            if(m_gps.detect_RMC){
                m_gps.ExtractData(pBuffer[j]);
                if(m_gps.data_acquisition){
                    m_gps.serial.Reset();
                    m_gps.HandlingDataGPS();
                    m_gps.data_acquisition = false;
                    m_gps.detect_RMC = false;
                    break;
                }
                ++m_iGpsIndex;
                if(m_iGpsIndex>90){
                    m_gps.detect_RMC = false;
                    m_gps.serial.Reset();
                }
            }
        }
    }
}

void publishGpsData(void)
{
    gps_data.data.clear();
    if(!m_gps.data_acquisition){
        int int_tmp;
        double double_tmp;
        char char_tmp;

        int_tmp = m_gps.m_iTimeHour;
        gps_data.data.push_back(int_tmp); // at(0) : hour
        int_tmp = m_gps.m_iTimeMinute;
        gps_data.data.push_back(int_tmp); // at(1) : minute
        double_tmp = m_gps.m_dTimeSecond;
        gps_data.data.push_back(double_tmp); // at(2) : second

        double_tmp = m_gps.m_adLatitude[CURR];
        gps_data.data.push_back(double_tmp); // at(3) : latitude
        char_tmp = m_gps.m_acLatitudeDirection[CURR];
        gps_data.data.push_back(char_tmp); // at(4) : latitude direction

        double_tmp = m_gps.m_adLongitude[CURR];
        gps_data.data.push_back(double_tmp); // at(5) : longitude
        char_tmp = m_gps.m_acLongitudeDirection[CURR];
        gps_data.data.push_back(char_tmp); // at(6) : longitude direction

        int_tmp = m_gps.m_iQualityIndicator;
        gps_data.data.push_back(int_tmp); // at(7) : mode indicator (1: single, 2: dgps, 4: RTK fix, 5: RTK float
        int_tmp = m_gps.m_iNumSatelliteinUse;
        gps_data.data.push_back(int_tmp); // at(8) : # of using satellite

	double_tmp = m_gps.m_dHDOP;
	gps_data.data.push_back(double_tmp); // at(9) : Horizontal dillusion of position (deviation)
	double_tmp = m_gps.m_adAntennaAltitude[CURR];
	gps_data.data.push_back(double_tmp); // at(10) : height (at virtual sea level)
	double_tmp = m_gps.m_adGeoidalSeparation[CURR];
	gps_data.data.push_back(double_tmp); // at(11) : height (difference between real sea level and virtual sea level)

        int_tmp = m_gps.m_iTimeHourRMC;
        gps_data.data.push_back(int_tmp); // at(12) : RMC hour
        int_tmp = m_gps.m_iTimeMinuteRMC;
        gps_data.data.push_back(int_tmp); // at(13) : RMC minute
        double_tmp = m_gps.m_dTimeSecondRMC;
        gps_data.data.push_back(double_tmp); // at(14) : RMC second

        char_tmp = m_gps.m_acRMCStatus;
        gps_data.data.push_back(char_tmp); // at(15) : RMC available status

        double_tmp = m_gps.m_adLatitudeRMC[CURR];
        gps_data.data.push_back(double_tmp); // at(16) : RMC latitude
        char_tmp = m_gps.m_acLatitudeDirectionRMC[CURR];
        gps_data.data.push_back(char_tmp); // at(17) : RMC latitude direction

        double_tmp = m_gps.m_adLongitudeRMC[CURR];
        gps_data.data.push_back(double_tmp); // at(18) : RMC longitude
        char_tmp = m_gps.m_acLongitudeDirectionRMC[CURR];
        gps_data.data.push_back(char_tmp); // at(19) : RMC longitude direction

        double_tmp = m_gps.m_adSpeed_Knots *0.51444444;
        gps_data.data.push_back(double_tmp); // at(20) : RMC speed (knots. 1 knots = 0.51444444 m/s)
        double_tmp = m_gps.m_adCourseOverGround_deg;
        gps_data.data.push_back(double_tmp); // at(21) : RMC heading degree (north base clock-wise direction 0~360 deg)

        gps_pub.publish(gps_data);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gps");
    ros::NodeHandle nh;

    std::string port;
    int baudrate;

    nh.param<std::string>("port", port, "/dev/ttyACM0");
    nh.param("baudrate", baudrate, 115200);

    if(!m_gps.serial.Open(const_cast<char*>(port.c_str()), baudrate)){
        cout << "device is not opened! " << endl;
        return 0;
    }    

    gps_pub = nh.advertise<std_msgs::Float64MultiArray>("GpsData", 1);

    ros::Rate loop_rate(4);

    while(ros::ok()){
        OnReceiveGPS();
        publishGpsData();

        loop_rate.sleep();     
    }

    return 0;
}
