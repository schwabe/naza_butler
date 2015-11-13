/**
MinimOSD standard configuration
https://code.google.com/p/minimosd-extra/wiki/APM

- 2hz for waypoints, GPS raw, fence data, current waypoint, etc
- 5hz for attitude and simulation state
- 2hz for VFR_Hud data 
- 3hz for AHRS, Hardware Status, Wind 
- 2hz for location data 
- 2hz for raw imu sensor data 
- 5hz for radio input or radio output data 

**/


#include "config.h"
#include "NazaDecoderLib.h"
#define DEBUG_GPS


#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include "GCS_MAVLink.h"
#include "../GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h"
mavlink_system_t mavlink_system;


#if defined(HOME_SET_PRECISION)
float home_set_precision = HOME_SET_PRECISION;
#else
float home_set_precision = 0.2;
#endif
void sendMavlinkMessages() {
  unsigned long currtime=millis();
  
  static unsigned long heartbeat_1hz=0;
  static unsigned long heartbeat_2hz=0;
  static unsigned long heartbeat_3hz=0;
  static unsigned long heartbeat_5hz=0;
  static unsigned long dimming = 0;
  static float alt_MSL_m_last=0;


  mavlink_system.sysid = 100; // System ID, 1-255
  mavlink_system.compid = 50; // Component/Subsystem ID, 1-255
  
  //Home can be set when GPS is 3D fix, and altitude is not changing (within 20cm) for more than 10s
  if(  (gpsFix < 3) || (abs(alt_MSL_m_last - alt_MSL_m) > home_set_precision ) ) {
    fix_time = currtime + 10000;
    alt_MSL_m_last = alt_MSL_m;
  } 
  
  #if defined(HOME_SET_AUTO_TIMEOUT)
  if( (currtime/1000) > HOME_SET_AUTO_TIMEOUT ) {
    fix_time = 0;
  }
  #endif
  
  //Set home altitude if not already set, and 3D fix and copter moving for more than 500ms
  if( (home_set == 0) && (currtime > fix_time) ) {
    alt_Home_m = NazaDecoder.getGPSalt();
    home_set = 1;
  } 
  
  #if defined(LIPO_CAPACITY_MAH_MULTI)
    static unsigned long lasttime=0;
  
    if( throttlepercent > 30 && currtime > 5000) {
      flight_time += (currtime - lasttime);
    } else {
      if( flight_time < BATTERY_DISPLAY_FTIME) flight_time = 0;
    }
    lasttime = currtime;
  #endif
  
  //1hz for mavlink heart beat
  if(currtime >= heartbeat_1hz ){
    sendHeartBeat();
    heartbeat_1hz = currtime + 1000;
    dimming = currtime;
  } 
  
  //2hz for waypoints, GPS raw, fence data, current waypoint, etc
  //2hz for VFR_Hud data 
  if(currtime >= heartbeat_2hz ){
    sendGpsData();
    sendVfrHud();
    sendGlobalPosition();
    heartbeat_2hz = currtime + 500;
    dimming = currtime;
  } 
  
  //3hz for AHRS, Hardware Status, Wind 
  if(currtime >= heartbeat_3hz){
    sendSystemStatus();
    heartbeat_3hz = currtime + 333;
    dimming = currtime;
  } 
  
  //5hz for attitude and simulation state
  //5hz for radio input or radio output data 
  if(currtime >= heartbeat_5hz ){
    sendAttitude();
    sendRawChannels();
    heartbeat_5hz = currtime + 200;
    dimming = currtime;
  } 
  
  if( currtime - dimming < 15 ) { //LED is on for 30ms when a message is sent
    LEDPIN_ON
  } else {
    LEDPIN_OFF
  }
}


//MAVLINK_MSG_ID_GLOBAL_POSITION_INT
/**
 * Send a global_position_int message
 * chan MAVLink channel to send the message
 *
 * time_boot_ms Timestamp (milliseconds since system boot)
 * lat Latitude, expressed as * 1E7
 * lon Longitude, expressed as * 1E7
 * alt Altitude in meters, expressed as * 1000 (millimeters), above MSL
 * relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * vx Ground X Speed (Latitude), expressed as m/s * 100
 * vy Ground Y Speed (Longitude), expressed as m/s * 100
 * vz Ground Z Speed (Altitude), expressed as m/s * 100
 * hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 */
void sendGlobalPosition() {
    mavlink_msg_global_position_int_send(
        MAVLINK_COMM_0,
        millis(), 
        (long)(lat*10000000), 
        (long)(lon*10000000), 
        (long)(alt_MSL_m*1000.0), 
        (long)((alt_MSL_m - alt_Home_m)*1000.0), 
        0, //Unknown, so not set
        0, //Unknown, so not set 
        0, //Unknown, so not set
        heading_d * 100.0);
}

//Send MAVLink Heartbeat, 1hz 
//MAVLINK_MSG_ID_HEARTBEAT

// type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
// autopilot Autopilot type / class. defined in MAV_AUTOPILOT ENUM
// base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
// custom_mode A bitfield for use for autopilot-specific flags.
// system_status System status flag, see MAV_STATE ENUM

void sendHeartBeat() {
  //static inline void mavlink_msg_heartbeat_send(mavlink_channel_t chan, uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status)
  //flightmode 0 : Stabilize
  //flightmode 1 : Acrobatic
  //flightmode 2 : Alt Hold
  //flightmode 3 : Auto
  //flightmode 4 : Guided
  //flightmode 5 : Loiter
  //flightmode 6 : Return to Launch
  //flightmode 7 : Circle
  //flightmode 8 : Position
  //flightmode 9 : Land
  //flightmode 10 : OF_Loiter
  
  uint8_t mav_base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
  //Custom flag used to indicate home altitude is set
  if( home_set == 1 ) {
    mav_base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
  }
  //Consider copter armed when there's some throttle
  if( (throttlepercent > 10) ) {
    mav_base_mode = mav_base_mode | MAV_MODE_FLAG_SAFETY_ARMED;
    isArmed = 1;
  } else {
    isArmed = 0;
  }
  
  mavlink_msg_heartbeat_send(
    MAVLINK_COMM_0, 
    MAV_TYPE_QUADROTOR, 
    MAV_AUTOPILOT_GENERIC, 
    mav_base_mode, 
    flightmode, 
    MAV_STATE_STANDBY);
}



//Send GPS data
//MAVLINK_MSG_ID_GPS_RAW_INT

// gps_utc_time_second Timestamp (microseconds since UNIX epoch or microseconds since system boot)
// gpsFix 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
// lat Latitude in 1E7 degrees
// lon Longitude in 1E7 degrees
// alt_MSL Altitude in 1E3 meters (millimeters) above MSL
// eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
// epv GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
// ground_speed GPS ground speed (m/s * 100). If unknown, set to: 65535
// cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
// numsats Number of satellites visible. If unknown, set to 255

void sendGpsData() {
//mavlink_msg_gps_raw_int_send(MAVLINK_COMM_0,gps_utc_time_second, gpsFix, (long)(lat*10000000), (long)(lon*10000000), alt_MSL (in mm), eph, epv, ground_speed, cog, numsats);

  mavlink_msg_gps_raw_int_send(
    MAVLINK_COMM_0,
    millis(), 
    gpsFix, 
    (long)(lat*10000000), 
    (long)(lon*10000000), 
    (long)(alt_MSL_m*1000.0), 
    eph_cm,
    epv_cm, 
    ground_speed_ms * 100.0, 
    cog_cd, 
    numsats);
}



//Send horizon position
//MAVLINK_MSG_ID_ATTITUDE

// time_boot_ms Timestamp (milliseconds since system boot)
// roll Roll angle (rad)
// pitch Pitch angle (rad)
// yaw Yaw angle (rad)
// rollspeed Roll angular speed (rad/s)
// pitchspeed Pitch angular speed (rad/s)
// yawspeed Yaw angular speed (rad/s)

void sendAttitude() {
	pitch_rad = (pitch_pwm - PITCH_LEVEL) * PI/500.0 * PITCH_GAIN / 10.0; ///12.00;      //500 is the difference between vertical and level
	#if defined(PITCH_INVERT)
          pitch_rad = pitch_rad * -1.0;
        #endif
	//need to scale up or down the pitch and roll - a delta of 500 is correct if pitch = 12 and roll is 7.6.  So scale the inputted values by that?
	//Pitch correction factor = configured pitch gain /12.00
	//Roll correction factor = configured roll gain / 7.60

	roll_rad = (roll_pwm - ROLL_LEVEL) * PI/500.0 * ROLL_GAIN / 10.0; ///7.60;
        #if defined(ROLL_INVERT)
          roll_rad = roll_rad * -1.0;
        #endif
	
	if(pitch_rad<0.05 && pitch_rad>-0.05) pitch_rad=0;
	if(roll_rad<0.05 && roll_rad > -0.05) roll_rad=0;
	if(pitch_rad>5) pitch_rad=0;
	if(roll_rad>5) roll_rad=0;
	if(pitch_rad<-5) pitch_rad=0;
	if(roll_rad<-5) roll_rad=0;
	mavlink_msg_attitude_send(
          MAVLINK_COMM_0, 
          millis(), 
          roll_rad, 
          pitch_rad,
          0.0, 
          0, 
          0, 
          0);
}



//Send speed, orientation, altitude and throttle percent
//MAVLINK_MSG_ID_VFR_HUD

// airspeed Current airspeed in m/s
// groundspeed Current ground speed in m/s
// heading Current heading in degrees, in compass units (0..360, 0=north)
// throttle Current throttle setting in integer percent, 0 to 100
// alt Current altitude (MSL), in meters
// climb Current climb rate in meters/second

void sendVfrHud() {
	//mavlink_msg_vfr_hud_send(mavlink_channel_t chan, float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb)
	mavlink_msg_vfr_hud_send(
          MAVLINK_COMM_0,
          ground_speed_ms,
          ground_speed_ms, 
          heading_d, 
          throttlepercent, 
          alt_m, 
          -climb); 
}



//Sensor information : current, voltage
//MAVLINK_MSG_ID_SYS_STATUS

// onboard_control_sensors_present Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
// onboard_control_sensors_enabled Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
// onboard_control_sensors_health Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
// load Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
// voltage_battery Battery voltage, in millivolts (1 = 1 millivolt)
// current_battery Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
// battery_remaining Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
// drop_rate_comm Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
// errors_comm Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
// errors_count1 Autopilot-specific errors
// errors_count2 Autopilot-specific errors
// errors_count3 Autopilot-specific errors
// errors_count4 Autopilot-specific errors

void sendSystemStatus() {
	//current is in ma.  Function needs to send in ma/10.
	#if ESTIMATE_BATTERY_REMAINING == ENABLED
            battery_remaining_A = estimatepower();
            ampbatt_A = 0;
	#else
            battery_remaining_A = capacity/battery_capacity*100.0;
            ampbatt_A = IFinal;
        #endif
        mavlink_msg_sys_status_send(MAVLINK_COMM_0,0,0,0,0,long(VFinal*1000.0),ampbatt_A*100.0,battery_remaining_A,0,0,0,0,0,0);
}



//Send RAW channels data and RSSI
//MAVLINK_MSG_ID_RC_CHANNELS_RAW

// time_boot_ms Timestamp (milliseconds since system boot)
// port Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
// chan1_raw RC channel 1 value, in microseconds
// chan2_raw RC channel 2 value, in microseconds
// chan3_raw RC channel 3 value, in microseconds
// chan4_raw RC channel 4 value, in microseconds
// chan5_raw RC channel 5 value, in microseconds
// chan6_raw RC channel 6 value, in microseconds
// chan7_raw RC channel 7 value, in microseconds
// chan8_raw RC channel 8 value, in microseconds
// rssi Receive signal strength indicator, 0: 0%, 255: 100%

void sendRawChannels() {
	mavlink_msg_rc_channels_raw_send(
        MAVLINK_COMM_0,
        millis(),
        0,        // port 0
        roll_pwm,
        pitch_pwm,
        #if defined(RC_PPM_MODE)
            rcDataPPM[YAW_PPM],       // Yaw
            throttle_pwm,  // Throttle
            rcDataPPM[X1_PPM],        // X1
            rcDataPPM[X2_PPM],        // X2
        #else 
            1500,
            throttle_pwm,
            1500,
            1500,
       #endif
        rcDataSTD[FMODE_STD],     // Flight mode
        panel_pwm,       // Aux
        receiver_rssi);
}

