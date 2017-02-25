/* USC RPL HAMSTER v2.2
 * Authors: Carter Allen, Jack Nelson, Adam Rising
    Contributors: Skye Mceowen, Ian Heidenberger, Vivek Ramachandran,
    Conor Hayes, James Hightower
 */

// Core libraries
#include "mbed.h"
#include <string>
#include <cerrno>
#include <cstdio>
#include <math.h>
#include "semihost_api.h"
#include "MovingAverage.h"
#include "MovingQuadReg.h"

//Peripheral device libraries
#include "ADXL345.h"
// #include "GPS.h"
#include "LM75B.h"
#include "SDFileSystem.h"
#include "MS5607Base.h"
#include "MS5607SPI.h"
#include "MAX3107.h"
#include "FastPWM.h"
#include "BNO055.h"

enum State {
    Default,
    Initial,
    Standby,
    StatusCheck,
    DetectLaunch,
    Ascent,
    Recovery
};

/* Pin Definitions */
//Pyro pin definitions.
DigitalOut pyro_fire_1(p22); //Main
DigitalOut pyro_fire_0(p21); //Drogue
AnalogIn pyro_stat_1(p18); //Main
AnalogIn pyro_stat_0(p19); //Drogue

// Raven Disarm Pin
DigitalOut disarm_ravens(p20);

//Set up serial devices
//Serial pc(USBTX, USBRX);//DEBUG PC serial
Serial pc(p9, p10); //XTEND serial

//SPI bus devices
// v2.1 SPI bus configuration:
// mosi - p5
// miso - p6
// sclk - p7
ADXL345 accelerometer(p5, p6, p7, p8); //v2.1 pins
MS5607SPI altimeter(p5, p6, p7, p15); //mosi, miso, sckl, cs. v2.1 board pins
MAX3107 max_gps(p5, p6, p7, p24, /*p23,*/ p29); //mosi, miso, sclk, cs, irq, rst
// InterruptIn maxInterrupt(p23);
bool maxInterruptPending = false;
FastPWM maxClock(p26);

//I2C bus
LM75B temp_sensor(p28,p27);
BNO055 imu(p28, p27); //SDA -> Orange, SCLK -> Yellow
DigitalOut imuReset(p30); // Active low
DigitalOut imubootload(p17);
//functions
void drogue_fire();
void main_fire();

//Flight constants
// TODO - These should be in a separate flight config file
const int maxlength = 100; //for GPS checksum
const int DROGUE_FIRE_WINDOW_START = 90; //drogues can't fire before this time
const int DROGUE_FIRE_WINDOW_END = 130;   //drogues automatically fire if not fired already
const int MAIN_FIRE_WINDOW_START = 230; // main can't fire before this time
const int FLIGHT_EXIT_TIME = 130;//Ascent time (time to apogee)
const int PRE_LAUNCH_TIMEOUT = 1200;
const int DESCENT_TIME = 300; //Time under parachute from apogee
const int HEARTBEAT_TIME_INTERVAL_STANDBY = 60; // DEBUG! ACTUALLY SET TO 60!
const int HEARTBEAT_TIME_INTERVAL_PRELAUNCH = 5;
//const int GPS_BAUD = 115200;
//const int GPS_MAX_BYTES = 80;
const int offsets[3] = {0, 0, 0};
const int MAVDEPTH = 10;
//Timers
Timer heartbeatTimer;
Timer launch_timer;
Timer flight_exit_timer;
Timer RecoveryTimer; //for looping and printing things
Timer flight_lock_write_timer;  // used to periodically write to flight lock file in main while loop
FILE *flightLockFile;
int exitTimerRollovers = 0;
/*Flight Lock File stuff*/
const int flight_lock_write_period = 1; // determines how frequently to update flight lock file

int launch_time = -1;   // GPS time of launch, given in seconds
float launch_timer_offset = 0;  // used to offset timer tracking total time in flight
int current_gps_time = 0;

//Setup File Systems
LocalFileSystem local("local"); //Local File System
//TODO - Whole program crashes on startup if there isn't an SD card. Need to handle that error
// SDFileSystem sd(p11, p12, p13, p14, "sd"); //Init for v2.1 board

// String buffers for flash files
#define GPS_BUFFER_LENGTH 400
#define GPS_MAX_LINE_LENGTH 150
#define ACCEL_BUFFER_LENGTH 10000
#define ACCEL_MAX_LINE_LENGTH 150
#define IMU_BUFFER_LENGTH 10000
#define IMU_MAX_LINE_LENGTH 150
#define PC_BAUD 115200

char gpsFileBuffer[GPS_BUFFER_LENGTH];
int gpsFileCursor;
char accelFileBuffer[ACCEL_BUFFER_LENGTH];
int accelFileCursor;
char imuFileBuffer[IMU_BUFFER_LENGTH];
int imuFileCursor;
bool MAINFIRED_FLAG = 0; // tells us if we have fired the mains ourselves yet.
// String buffer for reading from the MAX3107
char maxGPSBuffer[200];

// Global variables to store the most recent GPS data
float latitude_g, longitude_g, altitude_g;
int numberOfSatellites_g, fixQuality_g;
extern "C" void mbed_reset();
MovingQuadReg altitude_qreg(10); //10 deep for altitude
#define LAUNCH_THRESHOLD 80 //ft/s^2
#define FLIGHT_LOCK_PATH "/local/lock.txt"

size_t flushGPSBuffer() {
    // FILE *gpsFileID = fopen("/local/gps.txt", "a");
    // size_t bytesWritten = fwrite((const char *)gpsFileBuffer, sizeof(char), gpsFileCursor, gpsFileID);
    // fclose(gpsFileID);
    // gpsFileCursor = 0;
    // return bytesWritten;
    FILEHANDLE gpsFileID = semihost_open("gps.txt", 8);
    size_t bytesNotWritten = semihost_write(gpsFileID, (const unsigned char *)gpsFileBuffer, sizeof(char) * gpsFileCursor, 0);
    semihost_close(gpsFileID);
    gpsFileCursor = 0;
    return bytesNotWritten;
}

size_t flushAccelBuffer() {
    FILE *accelFileID = fopen("/local/accel.txt", "a");
    size_t bytesWritten = fwrite((const char *)accelFileBuffer, sizeof(char), accelFileCursor, accelFileID);
    fclose(accelFileID);
    accelFileCursor = 0;
    return bytesWritten;
}

size_t flushIMUBuffer() {
    // FILE *imuFileID = fopen("/local/imu.txt", "a");
    // size_t bytesWritten = fwrite((const char *)imuFileBuffer, sizeof(char), imuFileCursor, imuFileID);
    // fclose(imuFileID);
    // imuFileCursor = 0;
    // return bytesWritten;
    FILEHANDLE imuFileID = semihost_open("imu.txt", 8);
    size_t bytesNotWritten = semihost_write(imuFileID, (const unsigned char *)imuFileBuffer, sizeof(char) * imuFileCursor, 0);
    semihost_close(imuFileID);
    imuFileCursor = 0;
    return bytesNotWritten;
}

void resetGVals() {
  latitude_g = 0;
  longitude_g = 0;
  altitude_g = 0;
  numberOfSatellites_g = 0;
  fixQuality_g = 0;
}

bool checksum(char* gpsData){
   int tmpChecksum;
   char tmpGpsChecksum[3];
   tmpChecksum = 0;
   int i = 1;
   //computes your checksum
   while(gpsData[i] != '*' && i<maxlength){
      tmpChecksum ^= gpsData[i];
      i++;}
   //accounts for mutilated packet (i.e. no '*')
   if(i>=maxlength){
      pc.printf("Error: packet exceeds maxL");
      return false;}
   //Finds gpsChecksum
   i++; //goes past '*'
   //Gives tmpGpsChecksum in hex
   for(int j = i;j<(i+2);j++){
      tmpGpsChecksum[j-i] = gpsData[j];
   }
   tmpGpsChecksum[2] = '\0';
   int gpsChecksum = strtol(tmpGpsChecksum,NULL,16);
   if(tmpChecksum == gpsChecksum){return true;}
   else{return false;}
}

double convert_lat_coord(char *s, char north_south) {
    int deg, min, sec;
    double fsec, val;

    deg  = ( (s[0] - '0') * 10) + s[1] - '0';
    min  = ( (s[2] - '0') * 10) + s[3] - '0';
    sec  = ( ((s[5] - '0') * 1000) + ((s[6] - '0') * 100) + ((s[7] - '0') * 10) + (s[8] - '0'));
    fsec = (double)((double)sec /10000.0);
    val  = (double)deg + ((double)((double)min/60.0)) + (fsec/60.0);
    if (north_south == 'S') { val *= -1.0; }
    latitude_g = val;
    return val;
}

double convert_lon_coord(char *s, char east_west) {
    int deg, min, sec;
    double fsec, val;

    deg  = ( (s[0] - '0') * 100) + ((s[1] - '0') * 10) + (s[2] - '0');
    min  = ( (s[3] - '0') * 10) + s[4] - '0';
    sec  = ( ((s[6] - '0') * 1000) + ((s[7] - '0') * 100) + ((s[8] - '0') * 10) + (s[9] - '0'));
    fsec = (double)((double)sec /10000.0);
    val  = (double)deg + ((double)((double)min/60.0)) + (fsec/60.0);
    if (east_west == 'W') { val *= -1.0; }
    longitude_g = val;
    return val;

}

double convert_height(char *s) {
    double val = (double)(atof(s) / 1000.0);
    altitude_g = val;
    return val;
}

//converts the string containing the time to an integer.  The string is formatted weird,
//but the format is here: http://aprs.gids.nl/nmea/#gga
//It's basically just hrs:mins:secs format without the colons.
int convert_GPStime(char* input) {
    int rawtime = atoi(input);
    int secs = 0;
    secs += rawtime%100;
    rawtime /= 100;
    secs += (rawtime%100)*60;
    rawtime /= 100;
    secs += rawtime*3600;
    return secs;
}

/**
    Reads the launch time (in GPS time format) from the flight-lock file
    Returns true if file exists
    Stores gps time in gps_flight_time, and old launch timer value in launch_timer_offset
    Should be used exactly once in event of power loss
    By James
**/
bool readFlightLockFile() {
//    pc.printf("opened flightLockFile\r\n");
    flightLockFile = fopen(FLIGHT_LOCK_PATH, "r");
    if (flightLockFile == NULL) {
//        pc.printf("flightLockFile DNE\r\n");
        return false;   // file does not exist
    }
//    pc.printf("flightLockFile exists\r\n");
    // search for gps time data
    fscanf(flightLockFile, "%d", &launch_time);
    if (feof(flightLockFile)) {
//        pc.printf("flightLockFile is empty\r\n");
        fclose(flightLockFile);
        return false;
    }  // reached end of file unexpectedly, data DNE, but file exists
    fscanf(flightLockFile, "%f", &launch_timer_offset);// retrieve launch_timer value, if it exists. store it in launch_timer_offset
    if (feof(flightLockFile)) {
//        pc.printf("flightLockFile contains launch_time but not launch_timer_offset\r\n");
        fclose(flightLockFile);
        return false;
    }  // reached end of file unexpectedly, data DNE, but file exists
//    pc.printf("flightLockFile contains launch_time and launch_timer_offset\r\n");
    fclose(flightLockFile);
    return true;
}

/**
    Update flight lock file by writing launch time in # seconds, then writing value of launch timer
    Format is: gps time of launch on first line,
    By James
**/
void updateFlightLockFile() {
    if (flight_lock_write_timer.read() < flight_lock_write_period) return;  // do nothing, haven't reached period
    flight_lock_write_timer.stop();
    flight_lock_write_timer.reset();
//    pc.printf("opening flightLockFile\r\n");
    flightLockFile = fopen(FLIGHT_LOCK_PATH, "w");
    if (flightLockFile == NULL) {
//        pc.printf("failed to open flightLockFile\r\n");
        return; // flightLockFile should not be null
    }
 //   pc.printf("writing launch_time\r\n");
    fprintf(flightLockFile, "%d ", launch_time);
//    pc.printf("writing launch_timer.read()+launch_timer_offset\r\n");
    fprintf(flightLockFile, "%f", (flight_exit_timer.read()+launch_timer_offset));
//    pc.printf("value written: %f \n\r", flight_exit_timer.read() + launch_timer_offset);
    fclose(flightLockFile);
//    pc.printf("closed flightLockFile\r\n");
    flight_lock_write_timer.start();
}

void initFlightLockFile() {
    flight_lock_write_timer.stop();
    flight_lock_write_timer.reset();
    FILE *flightLockFile = fopen("/local/lock.txt", "w");
    fclose(flightLockFile);
}

void writelog(string s, float t) {
    FILE *fp = fopen("/local/log.log", "a");
    fprintf(fp, "%s %f\n", s, t);
    fclose(fp);
}


void processPendingMaxInterrupt() {
    // Check for the special character interrupt flag
    uint8_t isr = max_gps.readRegister(MAX3107::ISR);
    if (isr & MAX3107::SpCharInt) {
        // Read the special character interrupt to clear it out
        max_gps.readRegister(MAX3107::SpclCharInt);
        // Find out how long the sentence we received was
        uint8_t fifoLevel = max_gps.getRxFifoLevel();
        // Read that number of characters from the FIFO
        max_gps.receiveCharacters((uint8_t *)maxGPSBuffer, fifoLevel);
        // // Add a null byte to the buffer at the end of the string so we know where it ends
        maxGPSBuffer[fifoLevel+1] = '\0';
        // Print out the sentence
        // pc.printf("%s", maxGPSBuffer);
        if (strncmp(maxGPSBuffer, "$GPGGA", 6)) {
            pc.printf("GPS string is not GGA, sentence:\n\r%s\n\r", maxGPSBuffer);
            if (flushGPSBuffer()<0) {
              pc.printf("Failed to flush GPS buffer");
            }
            resetGVals();
            return;
        }
        //checks to see if the checksum is what we expect. if not, exit and we have a bad packet.
        if(!checksum(maxGPSBuffer)){
            pc.printf("Bad GPS checksum, sentence:\n\r%s\n\r", maxGPSBuffer);
            if (flushGPSBuffer()<0) pc.printf("Failed to flush GPS buffer");
            resetGVals();
            return;
        }

        // The following code was taken from the MODGPS library, Copyright (c) 2010 Andy Kirkham
        char *token;
        int  token_counter = 0;
        char *GPStimestr= (char *)NULL;
        char *latitude  = (char *)NULL;
        char *longitude = (char *)NULL;
        char *lat_dir   = (char *)NULL;
        char *lon_dir   = (char *)NULL;
        char *qual      = (char *)NULL;
        char *altitude  = (char *)NULL;
        char *sats      = (char *)NULL;



        token = strtok(maxGPSBuffer, ",");
        while (token) {
            switch (token_counter) {
                    case 1:  GPStimestr= token; break;
                    case 2:  latitude  = token; break;
                    case 4:  longitude = token; break;
                    case 3:  lat_dir   = token; break;
                    case 5:  lon_dir   = token; break;
                    case 6:  qual      = token; break;
                    case 7:  sats      = token; break;
                    case 9:  altitude  = token; break;
            }
            token = strtok((char *)NULL, ",");
            token_counter++;
        }

        // If the fix quality is valid set our location information.
        if (latitude && longitude && altitude && sats) {
            latitude_g = convert_lat_coord(latitude,  lat_dir[0]);
            longitude_g = convert_lon_coord(longitude, lon_dir[0]);
            altitude_g = convert_height(altitude);
            numberOfSatellites_g = atoi(sats);
            fixQuality_g = atoi(qual);
            current_gps_time = convert_GPStime(GPStimestr);

            // if the time is right, update the flight lock file
            if (flight_lock_write_timer.read() >= flight_lock_write_period) {
                updateFlightLockFile();
                flight_lock_write_timer.reset();
            }
            //keep track of altitude in MovingQuadReg class
            int GPStime = convert_GPStime(GPStimestr);
            altitude_qreg.addPoint(GPStime, altitude_g);

        }
        else {
            fixQuality_g = 0;
        }

        // Clear out the GPS buffer
        memset((void*)maxGPSBuffer, 0x00, fifoLevel+1);
    }
}


void configureMAX3107AndGPS() {

    // The MAX3107 needs a clock source, for which we are using a PWM from the Mbed.
    // To get a 2MHz PWM, you need to use the FastPWM library.
    maxClock.period_us(0.5); // 2MHz wave
    maxClock.write(0.5); // 50% duty cycle (square wave)

    // Configuring the MAX3107 baud rate generation is tricky, so Maxim provides a spreadsheet.
    // Except the spreadsheet is for the MAX3108. But they're pretty similar. I used the spreadsheet
    // to get these hex values.
    max_gps.writeRegister(MAX3107::PLLConfig, 0x42);
    wait_us(50);
    max_gps.writeRegister(MAX3107::BRGConfig, 0x08);
    wait_us(50);
    max_gps.writeRegister(MAX3107::DIVLSB, 0x38);
    wait_us(50);
    max_gps.writeRegister(MAX3107::DIVMSB, 0x01);
    wait_us(50);
    max_gps.writeRegister(MAX3107::CLKSource, 0x94);
    wait_us(50);

    // The GPS uses standard "8N1" serial (8 bit word, 1 stop bit). The LCR register controls that
    // in the MAX3107, and oddly enough, 8N1 isn't the default! So we have to set it here.
    max_gps.writeRegister(MAX3107::LCR, 0x03); // 8 bit word length, 1 stop bit
    wait_us(50);

    // This is a useful block to check to make sure all the clocking/format registers got saved.
    // Un-#if 0 it for debugging.
#if 0
    pc.printf("MAX3107 clocking configured as follows:\n");
    pc.printf("PLLConfig = 0x%02x\n", max_gps.readRegister(MAX3107::PLLConfig));
    pc.printf("BRGConfig = 0x%02x\n", max_gps.readRegister(MAX3107::BRGConfig));
    pc.printf("DIVLSB    = 0x%02x\n", max_gps.readRegister(MAX3107::DIVLSB));
    pc.printf("DIVMSB    = 0x%02x\n", max_gps.readRegister(MAX3107::DIVMSB));
    pc.printf("CLKSource = 0x%02x\n", max_gps.readRegister(MAX3107::CLKSource));
    pc.printf("LCR       = 0x%02x\n", max_gps.readRegister(MAX3107::LCR));
#endif

    // Tell the GPS to only send GGA sentences
    // The GPS uses the MT3339 chipset, and all the commands are documented here:
    // https://www.adafruit.com/datasheets/PMTK_A11.pdf
    // You have to add a checksum at the end (that's after the asterisk).
    // There's a site that can calculate it for you: http://www.hhhh.org/wiml/proj/nmeaxor.html
    max_gps.sendString("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n");

    // Set an interrupt on the MAX for the line break character
    // First enable the XOFF1 special character interrupt (low-level)
    // max_gps.writeRegister(MAX3107::SpclChrIntEn, MAX3107::XOFF1IntEn);
    max_gps.writeRegister(MAX3107::SpclChrIntEn, 0xFF);
    wait_us(50);
    // Then enable special character detection in the mode register
    max_gps.writeRegister(MAX3107::MODE2, MAX3107::SpecialChr);
    wait_us(50);
    // Set the character in the XOFF1 register to a line feed
    max_gps.writeRegister(MAX3107::XOFF1, '\n');
    wait_us(50);
    // Enable special character detection in the flow control register
    max_gps.writeRegister(MAX3107::FlowCtrl, 0x30);
    wait_us(50);
    // Enable routing of special character interrupts to the IRQ pin
    max_gps.writeRegister(MAX3107::IRQEn, MAX3107::SpclChrIEn);
    wait_us(50);
    // Finally, enable the IRQ pin itself
    max_gps.writeRegister(MAX3107::MODE1, MAX3107::IRQSel);

    // // Set our handler function on the falling edge of the IRQ line
    // maxInterrupt.fall(&maxInterruptHandler);
    // maxInterrupt.mode(PullUp);
    // maxInterrupt.enable_irq();

}

/*
  Function to handle reading input in recovery. Returns true if recovery needs
  to break.
*/
void readRecoveryInput(char* input, float timestamp, State& state, int i) {
  if (input[i-1] == 13){
      if ( strcmp(input,"STANDBY\r") == 0 ) {
          pc.printf("\nTransitioning to standby state\n\r");
          writelog("To STANDBY from RECOVERY via command", timestamp);
          heartbeatTimer.start();
          state = Standby;
          i = 0;
          input[0] = 13;
          disarm_ravens = 1;
          flight_exit_timer.stop(); flight_exit_timer.reset();
          remove("/local/lock.txt");
          // break;
      }
      else if ( strcmp(input, "DROGUEDEPLOY\r") == 0) {
          writelog("Drogue deployed via command in RECOVERY state ", timestamp);
          drogue_fire();
          pc.printf("Drogues deployed in RECOVERY state\n");
          writelog("Drogue deploy did not brown out ", timestamp);
      }
      else if (strcmp(input, "MAINDEPLOY\r") == 0) {
          writelog("Main deployed via command in RECOVERY state ", timestamp);
          main_fire();
          pc.printf("Main deployed in RECOVERY state\n");
          writelog("Main deploy did not brown out ", timestamp);
          remove("/local/lock.txt");
      }
      //Invalid command. Empty buffer and try again.
      else {
          i = 0;
          input[0] = 13;
          pc.printf("\nInvalid Command\n\r");
      }
  }
}

int main() {

    State initialState = Initial;

    //Create flight data files on SD card and write to them
    writelog("Initial Power On at ", 0);
    FILE *accel_file = fopen("/local/accel.txt", "a");//sensor file
    fprintf(accel_file, "---power on---\n\n");
    fclose(accel_file);
    FILE *gps_file = fopen("/local/gps.txt", "a");//gps file
    FILE *imu_file = fopen("/local/imu.txt", "a"); // IMU readings file
    FILE *temp_file = fopen("/local/temp.txt", "a");//temperature file
    fprintf(gps_file, "---power on---\n\n");
    fprintf(imu_file, "---power on---\n\n");
    fprintf(temp_file, "--power on---\n\ntime temp\n");
    fclose(gps_file);
    fclose(imu_file);
    fclose(temp_file);

    //GPS serial creation
    // gps.baud(GPS_BAUD);
    // GPS_Geodetic gpsGeo;

    Timer refresh_Timer;//for gps
    State state = initialState; //initial
    char input[15] = "";//input string, emulate XTEND input
    bool launch_flag = false;
    int i=0; int k = 0; int j = 0;
    int adxl_readings[3];//x,y,z
    int adxl_offsets[3];
//    MovingAverage adxlMAVx(MAVDEPTH);
//    MovingAverage adxlMAVy(MAVDEPTH);
    MovingAverage adxlMAVz(MAVDEPTH);
    // GPS data
    // float latitude, longitude, altitude;
    float max_altitude = 0;
    pc.baud(PC_BAUD);

    // Check if the flight-in-progress file exists
    flight_lock_write_timer.stop();
    flight_lock_write_timer.reset();
    flight_lock_write_timer.start();
    bool brownedOut = readFlightLockFile();
    if (brownedOut) {   // brown-out has occurred because flight lock file exists. need to determine current state.
        // readFlightLockFile should have stored launch time in brownedOut
        pc.printf("BROWNOUT DETECTED at: %d", current_gps_time);
        disarm_ravens = 0;
        // pc.printf("Time: %02d:%02d:%02d\n\r", gpsTime.hour, gpsTime.minute, gpsTime.second);
        // pc.printf("Date: %d/%d/%d\n\r", gpsTime.day, gpsTime.month, gpsTime.year);
        // pc.printf("Fix Quality: %d\n\r", gps.getGPSquality());
        pc.printf("launch time: %f  timer offset: %f\n\r", launch_time, launch_timer_offset);
        state = Ascent;
        flight_exit_timer.start();
        int deltaT = current_gps_time - launch_time;
        if (deltaT >= DROGUE_FIRE_WINDOW_END || launch_timer_offset >= DROGUE_FIRE_WINDOW_END) {   //
            state = Recovery;
            RecoveryTimer.stop(); RecoveryTimer.reset();
            RecoveryTimer.start(); flight_exit_timer.start();
        }

    }
    else { //no brownout. we will create flight lock file later, when we go into ascent.
        //Turn backplane off by turning pin high
        disarm_ravens = 1;
    }

    // Setup MAX3107 to read gps data
    pc.printf("Waiting for MAX3107 to startup...");
    bool maxStartedUp = max_gps.waitForStartup(60000);
    if (maxStartedUp) {
        pc.printf("done.\n");
    }
    else {
        pc.printf("timed out.\n");
        writelog ("MAX 3107 Timed Out ", -1);
    }
    uint8_t revID = max_gps.readRegister(MAX3107::RevID);
    pc.printf("MAX3107 rev ID: %d\n", revID);

    // Test the barometer
    float baroAlt = altimeter.getAltitude();
    pc.printf("Barometric alt: %f\n", baroAlt);

    configureMAX3107AndGPS();
//    wait(4);
    imuReset = 1;
    imubootload = 1;


// Reset the BNO055
    imu.reset();
    wait(1);

    if (!imu.check()){
        pc.printf("IMU not connected!\n");
        writelog("IMU not connected ", 0);
    }
    imu.setmode(OPERATION_MODE_NDOF);
/*    MovingAverage imuMAVx(10);
    MovingAverage imuMAVy(10); //10-deep moving average filter for IMU
    MovingAverage imuMAVz(10);
    MovingAverage imuMAVr(10);
    MovingAverage imuMAVp(10);
    MovingAverage imuMAVyaw(10);
    MovingAverage tMAV(10); //for time
*/

    while (1) {
        processPendingMaxInterrupt();
        switch (state)
        {
        ////////////////////////
        // INITIAL STATE
        ////////////////////////
        // Initial startup routine when the mbed powers on.

        case Initial: //initial
        {
            //state functions
            pc.printf("\n===HAMSTER Power On===\n\r");
            pc.printf("ADXL Offsets: ");
            for ( int i = 0; i < 3; i++) {
                accelerometer.setOffset(i, offsets[i]);
                pc.printf("%d ", accelerometer.getOffset(i));
            }
            pc.printf("\n");
            //state transitions
            pc.printf("Going to Standby\n\r");
            writelog("To Standby from Initial ", -1);
            heartbeatTimer.start();
            state = Standby;
            break;
        }

        ////////////////////////
        // STANDBY STATE
        ////////////////////////
        // Default "safe" state

        case Standby://standby
        {
            //state functions
            disarm_ravens = 1;

            //Standby Heartbeat
            if (heartbeatTimer.read() >= HEARTBEAT_TIME_INTERVAL_STANDBY) {
                // GPS_Time gpsTime;
                // gps.timeNow(&gpsTime);
                // gps.geodetic(&gpsGeo);
                const char *broken = "";
                if (!fixQuality_g)
                    broken = "/";
                // pc.printf("<%s3: Standby: %02d:%02d:%02d %d %f %f %f\n", broken, gpsTime.hour, gpsTime.minute, gpsTime.second, gpsQuality, gpsGeo.lat, gpsGeo.lon, gpsGeo.alt);
                pc.printf("<%s3: Standby: %d %f %f %f\n", broken, fixQuality_g, latitude_g, longitude_g, altitude_g);
                heartbeatTimer.reset();
                heartbeatTimer.start();
            }


            //state transitions
            if (pc.readable()) {
                input[i] = pc.getc();
                pc.printf("%c", input[i]);
                input[++i] = '\0';
            }

            if (input[i-1] == 13) {
                if ( strcmp(input,"LAUNCH\r") == 0 ) {
                    pc.printf("\nTransitioning to status check state\n\r");
                    writelog("To Status Check from Standby with LAUNCH command ", -1);
                    state = StatusCheck;
                    launch_flag = true;
                    i = 0;
                    input[0] = 13;
                }
                if ( strcmp(input, "+++WIPE+++\r") == 0) {
                    pc.printf("\n\rAre you sure you want to wipe all the memory? Y/N: ");
                    char c = pc.getc();
                    pc.printf("%c\n", c);
                    if (c == 'Y' || c == 'y') {
                        //WIPE the memory files
                        accel_file = fopen("/local/accel.txt", "w");//sensor file
                        fprintf(accel_file, "---memory wiped---\n\n");
                        fclose(accel_file);
                        gps_file = fopen("/local/gps.txt", "w");//gps file
                        imu_file = fopen("/local/imu.txt", "w"); // IMU readings file
                        temp_file = fopen("/local/temp.txt", "w");//temperature file
                        fprintf(gps_file, "---memory wiped---\n\n");
                        fprintf(imu_file, "---memory wiped---\n\n");
                        fprintf(temp_file, "--memory wiped---\n\n\n");
                        fclose(gps_file);
                        fclose(imu_file);
                        fclose(temp_file);
                        pc.printf("\nMemory Wiped.\n\r");
                        writelog("Memory Wiped ", -1);
                    }
                    i = 0;
                    input[0] = 13;
                }
                else if ( strcmp(input, "+++WIPELOG+++\r") == 0) {
                    pc.printf("\n\rAre you sure you want to wipe the log file? Y/N: ");
                    char c = pc.getc();
                    pc.printf("%c\n", c);
                    if (c == 'Y' || c == 'y') {
                        //WIPE the memory files
                        FILE *fp = fopen("/local/log.log", "w");//sensor file
                        fclose(fp);
                        writelog("LOG wiped via COMMAND ", 0);
                    }
                    i = 0;
                    input[0] = 13;
                    pc.printf("LOG file wiped\n\r");
                }
                else if ( strcmp(input, "DUMP\r") == 0 ) {
                    writelog("DUMP commissioned ", -1);
                    const char *names[] = {"/local/gps.txt", "/local/log.log", "/local/imu.txt", "/local/temp.txt"};
                    char line[256];
                    for (int jj = 0; jj < sizeof(names)/sizeof(*names); jj++) {
                        pc.printf("\n====== FILE START ======\n");
                        FILE *file = fopen(names[jj], "r");
                        while (fgets(line, sizeof(line), file)) {
                            pc.printf("%s", line);
                        }
                        fclose(file);
                    }
                    writelog("DUMP completed ", -1);
                    pc.printf("DUMP completed\n\r");
                }
                else if ( strcmp(input,"CHECK\r") == 0 ) {
                    pc.printf("\nTransitioning to status check state\n\r");
                    state = StatusCheck;
                    i = 0;
                    input[0] = 13;
                    writelog("To Status Check from STANDBY ", -1);
                }
                else if ( strcmp(input,"RESET\r") == 0) {
                    writelog("RESET in STANDBY ",-1);
                    mbed_reset();
                }
                else if ( strcmp(input, "DROGUEDEPLOY\r") == 0) {
                    pc.printf("Are you sure you want to deploy the Drogues? Y/N\n\r");
                    char c = pc.getc();
                    if (c == 'Y' || c == 'y') {
                        drogue_fire();
                        pc.printf("Drogues deployed in STANDBY state.\n");
                        writelog("Manual Drogue Deploy in STANDBY ", -1);
                    }
                    else {
                        pc.printf("Drogues NOT deployed in STANDBY state.\n");
                        writelog("Manual Drogue NOT deployed in STANDBY ", -1);
                    }
                }
                else if (strcmp(input, "MAINDEPLOY\r") == 0) {
                    pc.printf("Are you sure you want to deploy the Main? Y/N\n\r");
                    char c = pc.getc();
                    if (c == 'Y' || c == 'y') {
                        main_fire();
                        pc.printf("Main deployed in STANDBY state.\n");
                        writelog("Manual Main Deploy in STANDBY ", -1);
                    }
                    else {
                        pc.printf("Main NOT deployed in STANDBY state.\n");
                        writelog("Manual Main NOT deployed in STANDBY ", -1);
                    }
                }
                // TODO - Make this into a separate function outside main
                // ADXL375 Calibration routine
                else if ( strcmp(input, "ACAL\r") == 0 ) {
                    i = 0;
                    input[0] = 13;

                    int continue_flag = 0;
                    int reading_counter = 0;
                    Timer reading_timer;
                    int16_t sums[3] = {0, 0, 0};
                    double avgs[3] = {0,0,0};

                    pc.printf("\nADXL345 Calibration\nPlace the accelerometer with the x-axis up. Enter 'GO' when ready to calibrate.\n\r");

                    while(continue_flag != 1){
                        if(pc.readable()){
                            input[i] = pc.getc();
                            pc.printf("%c", input[i]);
                            input[++i] = '\0';
                        }

                        if (strcmp(input,"GO\r") == 0){
                             continue_flag = 1;
                             i = 0;
                             input[0] = 13;
                        }

                        wait(.001);//This will set the speed of the loop
                        if (i == 15 || (i != 0 && input[i-1] == 13)) {
                            //pc.printf("\nInvalid Command\n\r");
                            i = 0;
                            input[0] = 13;
                        }


                    }
                    pc.printf("\nReading accelerometer values for 500ms.\n\r");
                    continue_flag = 0;
                    //get sample adxl readings
                    reading_timer.start();

                    while(reading_timer.read_ms() < 500){
                        accelerometer.getOutput(adxl_readings);
                        pc.printf("Accel: %d %d %d\n\r", (int16_t)adxl_readings[0], (int16_t)adxl_readings[1], (int16_t)adxl_readings[2]);
                        sums[0] += adxl_readings[0];
                        sums[1] += adxl_readings[1];
                        sums[2] += adxl_readings[2];

                        reading_counter++;
                    }

                    pc.printf("Count = %d\n", reading_counter);
                    // Average the values for each axis
                    for (int i = 0; i < 3; i++) {
                        avgs[i] = (double)sums[i];
                        pc.printf("avgs[%d] = %f\n", i, avgs[i]);
                        avgs[i] = avgs[i]/(double)reading_counter;
                        adxl_offsets[i] = (int16_t)ceil(avgs[i]);
                    }
                    pc.printf("averaged values: %d %d %d\n", adxl_offsets[0], adxl_offsets[1], adxl_offsets[2]);

                    // Calculate the X offset (subtract the 1g gravity field)
                    adxl_offsets[0] = adxl_offsets[0] - 21;

                   pc.printf("\nOffset values:\nx = %d\ny = %d\nz = %d\n\r", adxl_offsets[0], adxl_offsets[1], adxl_offsets[2]);

                }
                //Invalid command. Empty buffer and try again.
                else {
                    i = 0;
                    input[0] = 13;
                    pc.printf("\nInvalid Command\n\r");
                }
            }
            break;
        }
        //////////////////
        // STATUS CHECK
        //////////////////
        // Check the sensor and pyro statuses

        case StatusCheck://status check
        {
            int err = 0; // Number of errors encountered
            pc.printf("\nRunning status check...\n\r");
            //Pyro continuity checks
            pc.printf("Pyro 0: %f Pyro 1: %f\n\r", pyro_stat_0.read(), pyro_stat_1.read());
            if (pyro_stat_0.read() > .5) //Garbage threshold, don't know if this is good yet.
                pc.printf("Pyro 0 appears to be connected.\n\r");
            else {
                pc.printf("Pyro 0 does not appear to be connected.\n\r");
                err++;
            }
            if (pyro_stat_1.read() > .5)
                pc.printf("Pyro 1 appears to be connected.\n\r");
            else {
                pc.printf("Pyro 1 does not appear to be connected.\n\r");
                err++;
            }
            //barometer
            float baroAlt = altimeter.getAltitude();
            pc.printf("Barometric alt: %f\n", baroAlt);
            //accelerometer setup
            int devID = accelerometer.getDevId();
            pc.printf("Accel dev id: %d\n", devID);
            //Go into standby mode to configure the device.
            accelerometer.setPowerControl(0x00);
            //Full resolution, +/-16g, 4mg/LSB.
            accelerometer.setDataFormatControl(0x0B);
            //3.2kHz data rate.
            accelerometer.setDataRate(ADXL345_3200HZ);
            //Measurement mode.
            accelerometer.setPowerControl(0x08);
            //get sample adxl readings
            for (int j = 0; j < 3; j++) {
                accelerometer.getOutput(adxl_readings);
                pc.printf("Accel x: %i, y: %i, z: %i\n\r", (int16_t)adxl_readings[0], (int16_t)adxl_readings[1], (int16_t)adxl_readings[2]);
            }
            // Get current IMU readings
            imu.get_calib();
            imu.get_angles();
            imu.get_lia();
            pc.printf("IMU (cal,r,p,y,ax,ay,az):\n0x%02x %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f\n\r",
                imu.calib,imu.euler.roll,imu.euler.pitch,imu.euler.yaw,
                imu.lia.x,imu.lia.y,imu.lia.z
                );
            // Get temperature
            temp_sensor.open();
            pc.printf("Temp: %4.1f\n\r", (float) temp_sensor);
            // Current GPS reading
            // GPS_Time gpsTime;
            // GPS_VTG gpsVTG;
            // gps.timeNow(&gpsTime);
            //             pc.printf("Time: %02d:%02d:%02d\n\r", gpsTime.hour, gpsTime.minute, gpsTime.second);
            //             pc.printf("Date: %d/%d/%d\n\r", gpsTime.day, gpsTime.month, gpsTime.year);
            //             pc.printf("Fix Quality: %d\n\r", gps.getGPSquality());
            pc.printf("Fix Quality: %d\n\r", fixQuality_g);
            if (fixQuality_g) {
                pc.printf("Location: %f, %f\n\r", latitude_g, longitude_g);
                pc.printf("Altitude: %f\n\r", altitude_g);
                pc.printf("Satellites: %d\n\r", numberOfSatellites_g);
            }
            //             if (gps.getGPSquality()) {
            //                 pc.printf("Location: %f, %f\n\r", gps.latitude(), gps.longitude());
            //                 pc.printf("Altitude: %f\n\r", gps.altitude());
            //  pc.printf("Satellites: %d\n\r", gps.numOfSats());
            //  gps.vtg(&gpsVTG);
            //                 pc.printf("Velocity: %f knots\n\r", gpsVTG.velocity_knots());
            //                 pc.printf("Track (deg true): %f\n\r", gpsVTG.track_true());
            //             }
            //
            //             if (!gps.getGPSquality())//increase the error if no fix
            //                 err++;
            if (!fixQuality_g)
                err++; //increase the error if we don't have a GPS fix

            //int err = pyrochecks();
            if (err > 0 && launch_flag) {
                pc.printf("Error count = %d, are you sure you want to continue? Y/N or R for retry\n\r", err);
                char c = pc.getc();
                pc.printf("%c", c);
                if (c == 'N' || c == 'n') {
                    launch_flag = 0;
                    writelog("to STANDBY via STATUS CHECK FAIL 'n' ", -1);
                }
                if (c == 'r' || c == 'R') {
                    launch_flag = 1;//repeat status check
                    break;
                }
                if (c == 'Y' || c == 'y'){
                    launch_flag = 1;
                    writelog("to LAUNCH DETECT via STATUS CHECK ERROR IGNORE ", -1);
                }
                else {
                    pc.printf("Not a valid input. Returning to Standby\n\r");
                    pc.printf("%d", c);
                    launch_flag = 0;
                    state = Standby;
                    writelog("to STANDBY via STATUS CHECK FAIL bad char ", -1);
                }
            }

            //TODO - Is it necessary to get a char from pc here?
            if (pc.readable()) {
                input[i] = pc.getc();
                pc.printf("%c", input[i]);
                input[++i] = '\0';
            }
            //state transitions
            if ( launch_flag == 0 ) {
                pc.printf("\nTransitioning to standby state\n\r");
                state = Standby;
                heartbeatTimer.start();
                i = 0;
                input[0] = 13;
                writelog("To STANDBY from Status Check ", -1);
            }
            else {
                launch_flag = 0;
                pc.printf("\nTransitioning to Detect Launch state and turning on backplane...\n\r");
                disarm_ravens = 0;
                launch_timer.reset();
                wait(2);//raven power up, 2 seconds
                launch_timer.start();//start launch timer, will default exit after some time
                heartbeatTimer.stop(); heartbeatTimer.reset();
                heartbeatTimer.start();
                state = DetectLaunch;
                i = 0;
                input[0] = 13;
                writelog("To DETECT LAUNCH from Status Check ", -1);
            }
            break;
        }
        ////////////////////////
        // LAUNCH DETECT STATE
        ///////////////////////
        // Prepare for launch

        case DetectLaunch://detect launch
        {
            /*Launch detection algorithm - look at IMU and ADXL, although we only use IMU for algorithm*/
            wait(.01);
            /*imu.get_lia();
            imu.get_calib();
            imu.get_angles();
            imuMAVx << imu.lia.x;
            imuMAVy << imu.lia.y; //the IMU's y axis points up, so is the axis we care about
            imuMAVz << imu.lia.z;

            imuMAVr << imu.euler.roll;
            imuMAVp << imu.euler.pitch;
            imuMAVyaw << imu.euler.yaw;*/
            accelerometer.getOutput(adxl_readings);
            //adxlMAVx << (int16_t)adxl_readings[0]; //the ADXL's x axis points up, so is the axis we care about
            //adxlMAVy << (int16_t)adxl_readings[1];
            adxlMAVz << (int16_t)adxl_readings[2];
            if (adxlMAVz.getAvg() > LAUNCH_THRESHOLD) {
                pc.printf("LAUNCH DETECTED\n\r");
                writelog("To ASCENT: LAUNCH DETECTED in Launch Detect State ", 0);
                writelog("Come fly with me, let's fly, let's fly away...", 0);
                //We change state to ASCENT!
                flight_exit_timer.start();//flight timer
                pc.printf("\nTransitioning to ascent state\n\r");
                state = Ascent;
                i = 0;
                input[0] = 13;
                // GPS_Time gpsTime;
                // gps.timeNow(&gpsTime);
                time_t seconds = time(NULL);
                // time_t seconds = 1;
                accel_file = fopen("/local/accel.txt", "a");
                fprintf(accel_file, "ASCENT BEGINNING @ %s\ntime x y z\n", ctime(&seconds));
                fclose(accel_file);
                gps_file = fopen("/local/gps.txt", "a");
                imu_file = fopen("/local/imu.txt", "a");
                fprintf(gps_file, "ASCENT BEGINNING @ %s\n", ctime(&seconds));
                fprintf(imu_file, "ASCENT BEGINNING @ %s\ntime r p y ax ay az\n", ctime(&seconds));
                fclose(gps_file);
                fclose(imu_file);
                //Create flight lock file - we were going to fly!
                flight_lock_write_timer.start();
                //Flush all the data points into file before switching
                /*FILE *imu_file = fopen("/local/imu.txt", "a");
                FILE *accel_file = fopen("/local/accel.txt", "a");

                for (int s = 0; s < MAVDEPTH; s++) {
                    //write all the elements in queues in MAVs
                    //IMU:
                    fprintf(imu_file, "%f %02x %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f\n", tMAV.elements.front() - launch_timer.read(),
                        imu.calib,imuMAVr.elements.front(),imuMAVp.elements.front(),imuMAVyaw.elements.front(),
                        imuMAVx.elements.front(),imuMAVy.elements.front(),imuMAVz.elements.front());
                    imuMAVr.elements.pop(); imuMAVp.elements.pop(); //must pop elements manually
                    imuMAVyaw.elements.pop(); imuMAVx.elements.pop();
                    imuMAVy.elements.pop(); imuMAVz.elements.pop();

                    //accel:
                    fprintf(accel_file, "%f %d %d %d\n", tMAV.elements.front() - launch_timer.read(), (int16_t)adxlMAVx.elements.front(), (int16_t)adxlMAVy.elements.front(),
                        (int16_t)adxlMAVz.elements.front());
                    tMAV.elements.pop(); adxlMAVx.elements.pop();
                    adxlMAVy.elements.pop(); adxlMAVz.elements.pop();
                    //update timer value into tMAV
                    tMAV << launch_timer.read();
                }
                fclose(imu_file);
                fclose(accel_file); */
                //reset launch timer
                launch_timer.stop();
                launch_timer.reset();
                launch_timer.start(); //timer gets started right here
            }
            //state transitions
            if (pc.readable()) {
                input[i] = pc.getc();
                pc.printf("%c", input[i]);
                input[++i] = '\0';
            }

            if (input[i-1] == 13){
                if ( strcmp(input,"FLIGHT\r") == 0 ) {
                    flight_exit_timer.start();//flight timer
                    pc.printf("\nTransitioning to ascent state\n\r");
                    writelog("To ASCENT from DETECT LAUNCH via FLIGHT command at ", 0);
                    writelog("Come fly with me, let's fly, let's fly away...", 0);
                    state = Ascent;
                    i = 0;
                    input[0] = 13;
                    // GPS_Time gpsTime;
                    // gps.timeNow(&gpsTime);
                    time_t seconds = time(NULL);
                    // time_t seconds = 1;
                    accel_file = fopen("/local/accel.txt", "a");
                    fprintf(accel_file, "ASCENT BEGINNING @ %s\ntime x y z\n", ctime(&seconds));
                    fclose(accel_file);
                    gps_file = fopen("/local/gps.txt", "a");
                    imu_file = fopen("/local/imu.txt", "a");
                    fprintf(gps_file, "ASCENT BEGINNING @ %s\n", ctime(&seconds));
                    fprintf(imu_file, "ASCENT BEGINNING @ %s\ntime r p y ax ay az\n", ctime(&seconds));
                    fclose(gps_file);
                    fclose(imu_file);
                    //Create flight lock file - we were going to fly!
                    flight_lock_write_timer.start();
                }
                else if ( strcmp(input,"DISARM\r") == 0 || launch_timer.read() >= PRE_LAUNCH_TIMEOUT) {
                    if (launch_timer.read() >= PRE_LAUNCH_TIMEOUT) {
                        pc.printf("Pre-Launch Timeout. ");
                        writelog("to STANDBY from DETECT LAUNCH via TIMEOUT ", -1);
                    }
                    else {
                        writelog("to STANDBY  from DETECT LAUNCH via DISARM command ", -1);
                    }
                    pc.printf("\nDisarmed, transitioning to standby state\n\r");
                    launch_timer.stop();
                    launch_timer.reset();
                    heartbeatTimer.stop();
                    state = Standby;
                    heartbeatTimer.start();
                    i = 0;
                    input[0] = 13;
                }
                else if ( strcmp(input, "DROGUEDEPLOY\r") == 0) {
                    pc.printf("Are you sure you want to deploy the Drogues? Y/N\n\r");
                    char c = pc.getc();
                    if (c == 'Y' || c == 'y') {
                        drogue_fire();
                        pc.printf("Drogues deployed in LAUNCH DETECT state.\n");
                        writelog("Drogues deployed in LAUNCH DETECT state ", -1);
                    }
                    else {
                        pc.printf("Drogues NOT deployed in LAUNCH DETECT state.\n");
                        writelog("Drogues NOT deployed in LAUNCH DETECT state ", -1);
                    }
                }
                else if (strcmp(input, "MAINDEPLOY\r") == 0) {
                    pc.printf("Are you sure you want to deploy the Drogues? Y/N\n\r");
                    char c = pc.getc();
                    if (c == 'Y' || c == 'y') {
                        main_fire();
                        writelog("Main deployed in LAUNCH DETECT state ", -1);
                        pc.printf("Main deployed in LAUNCH DETECT state.\n");
                    }
                    else {
                        writelog("Main NOT deployed in LAUNCH DETECT state ", -1);
                        pc.printf("Main NOT deployed in LAUNCH DETECT state.\n");
                    }
                }
                //Invalid command. Empty buffer and try again.
                else {
                    i = 0;
                    input[0] = 13;
                    pc.printf("\nInvalid Command\n\r");
                }
            }

            // Heartbeat message
            if (heartbeatTimer.read() >= HEARTBEAT_TIME_INTERVAL_PRELAUNCH) {
                // int gpsQuality = gps.getGPSquality();
                // GPS_Time gpsTime;
                // gps.timeNow(&gpsTime);
                // gps.geodetic(&gpsGeo);
                const char *broken = "";
                if (!fixQuality_g)
                    broken = "/";
                int time_remaining = PRE_LAUNCH_TIMEOUT - launch_timer.read();
                // pc.printf("<%s3: Launch Detect: %02d:%02d:%02d Timeout T-%d: %d %f %f %f\n", broken, gpsTime.hour, gpsTime.minute, gpsTime.second, time_remaining, gpsQuality, gpsGeo.lat, gpsGeo.lon, gpsGeo.alt);
                pc.printf("<%s3: Launch Detect: Timeout T-%d: %d %f %f %f\n\r", broken, time_remaining, fixQuality_g, latitude_g, longitude_g, altitude_g);
                heartbeatTimer.reset();
                heartbeatTimer.start();
            }
            break;
        }
        ///////////////////////////
        // ASCENT STATE
        //////////////////////////
        // Take data as fast as possible
        case Ascent://ascent state
        {
            updateFlightLockFile();
            // State functions
            // Save GPS and barometer data every 200 cycles
            if (j > 200) {
                // gps.geodetic(&gpsGeo);

                // int gpsQuality = gps.getGPSquality();
                //Write GPS data directly to file on SD card
                /*gps_file = fopen("/local/gps.txt", "a");
                fprintf(gps_file, "%4.4f %d %f %f %f\n", timestamp, fixQuality_g, latitude_g, longitude_g, altitude_g);
                fclose(gps_file);

                if ((altitude_g > max_altitude) && (fixQuality_g == 1)) {
                    max_altitude = altitude_g;
                }*/

                //Write buffered data to the on-board flash using the flushGPSBuffer() function above
                int gpsBufferSpaceRemaining = GPS_BUFFER_LENGTH - gpsFileCursor;
                if (gpsBufferSpaceRemaining < GPS_MAX_LINE_LENGTH) {
                    int bytesWritten = flushGPSBuffer(); //TODO returns a size_t of how many bytes written. Need a better way to determine an unsuccessful flush
                    if (bytesWritten < 0)
                        pc.printf("DEBUG: GPS flush failed to write. Error code: %d.\n", bytesWritten);
                    bytesWritten = flushAccelBuffer();
                    if (bytesWritten < 0)
                        pc.printf("DEBUG: Accel flush failed to write. Error code: %d.\n", bytesWritten);
                    bytesWritten = flushIMUBuffer();
                    if (bytesWritten < 0)
                        pc.printf("DEBUG: IMU flush failed to write.Error code: %d.\n", bytesWritten);
                    gpsBufferSpaceRemaining = GPS_BUFFER_LENGTH - gpsFileCursor;
                }
                double timestamp = flight_exit_timer.read() + launch_timer_offset;
                float baroAlt = altimeter.getAltitude();
                gpsFileCursor += snprintf(gpsFileBuffer+gpsFileCursor, gpsBufferSpaceRemaining, "%4.4f %d %f %f %f %f\n", timestamp, fixQuality_g, latitude_g, longitude_g, altitude_g, baroAlt);
                if ((altitude_g > max_altitude) && (fixQuality_g == 1)) {
                    max_altitude = altitude_g;
                }

                //Transmit the telemetry
                pc.printf("%4.4f %d %f %f %f %f %f\n\r", timestamp, fixQuality_g, latitude_g, longitude_g, altitude_g, baroAlt, max_altitude);
                j = 0;
            }
            j++;

            // Save accelerometer every cycle
            accelerometer.getOutput(adxl_readings);

            //Write buffered data to on-board flash memory
            int accelBufferSpaceRemaining = ACCEL_BUFFER_LENGTH - accelFileCursor;
            if (accelBufferSpaceRemaining < ACCEL_MAX_LINE_LENGTH) {
                int bytesWritten = flushGPSBuffer(); //TODO returns a size_t of how many bytes written. Need a better way to determine an unsuccessful flush
                if (bytesWritten < 0)
                    pc.printf("DEBUG: GPS flush failed to write. Error code: %d.\n", bytesWritten);
                bytesWritten = flushAccelBuffer();
                if (bytesWritten < 0)
                    pc.printf("DEBUG: Accel flush failed to write.Error code: %d.\n", bytesWritten);
                bytesWritten = flushIMUBuffer();
                if (bytesWritten < 0)
                    pc.printf("DEBUG: IMU flush failed to write.Error code: %d.\n", bytesWritten);
                accelBufferSpaceRemaining = ACCEL_BUFFER_LENGTH - accelFileCursor;
            }
            double timestamp = flight_exit_timer.read() + launch_timer_offset;
            accelFileCursor += snprintf(accelFileBuffer+accelFileCursor, accelBufferSpaceRemaining, "%f %d %d %d\n", timestamp, (int16_t)adxl_readings[0], (int16_t)adxl_readings[1], (int16_t)adxl_readings[2]);

            // Save IMU data every cycle
            int imuBufferSpaceRemaining = IMU_BUFFER_LENGTH - imuFileCursor;
            if (imuBufferSpaceRemaining < IMU_MAX_LINE_LENGTH) {
                int bytesWritten = flushGPSBuffer(); //TODO returns a size_t of how many bytes written. Need a better way to determine an unsuccessful flush
                if (bytesWritten < 0)
                    pc.printf("DEBUG: GPS flush failed to write. Error code: %d.\n", bytesWritten);
                bytesWritten = flushAccelBuffer();
                if (bytesWritten < 0)
                    pc.printf("DEBUG: Accel flush failed to write.Error code: %d.\n", bytesWritten);
                bytesWritten = flushIMUBuffer();
                if (bytesWritten < 0)
                    pc.printf("DEBUG: IMU flush failed to write.Error code: %d.\n", bytesWritten);
                imuBufferSpaceRemaining = IMU_BUFFER_LENGTH - imuFileCursor;
            }
            imu.get_calib();
            imu.get_angles();
            imu.get_lia();
            timestamp = flight_exit_timer.read() + launch_timer_offset;
            imuFileCursor += snprintf(imuFileBuffer+imuFileCursor, imuBufferSpaceRemaining,
                "%f %02x %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f\n", timestamp,
                imu.calib,imu.euler.roll,imu.euler.pitch,imu.euler.yaw,
                imu.lia.x,imu.lia.y,imu.lia.z);

            // Save temperature data every 500 cycles
            if (k > 500) {
                temp_file = fopen("/local/temp.txt", "a");
                fprintf(temp_file, "%f %4.1f\n", flight_exit_timer.read() + launch_timer_offset, (float)temp_sensor);
                fclose(temp_file);
                k = 0;
            }
            k++;

            //state transitions
            if (pc.readable()) {
                input[i] = pc.getc();
                pc.printf("%c", input[i]);
                input[++i] = '\0';
            }
            if (input[i-1] == 13){
                if ( strcmp(input,"DISARM\r") == 0 ) {
                    pc.printf("\nDisarmed, transitioning to standby state\n\r");
                    writelog("to STANDBY from ASCENT mode via command after ", flight_exit_timer.read());
                    flight_exit_timer.stop();
                    flight_exit_timer.reset();
                    heartbeatTimer.start();
                    state = Standby;
                    disarm_ravens = 1;
                    i = 0;
                    input[0] = 13;
                    remove("/local/lock.txt");
                }
                //just in case things go very wrong
                else if ( strcmp(input, "DROGUEDEPLOY\r") == 0) {
                    writelog("Drogues deployed in ASCENT state via command at ", flight_exit_timer.read() + launch_timer_offset);
                    drogue_fire();
                    pc.printf("Drogues deployed in ASCENT state, transitioning to recovery state\n\r");
                    writelog("To RECOVERY state from ASCENT via MANUAL DROGUE DEPLOY ", flight_exit_timer.read() + launch_timer_offset);
                    state = Recovery;
                    RecoveryTimer.start();
                }
                else if (strcmp(input, "MAINDEPLOY\r") == 0) {
                    writelog("Mains deployed in ASCENT state via command at ", flight_exit_timer.read() + launch_timer_offset);
                    main_fire();
                    pc.printf("Main chutes deployed in ASCENT state, transitioning to recovery state\n\r");
                    state = Recovery;
                    pc.printf("Transitioning to RECOVERY state\n\r");
                    writelog("To RECOVERY state from ASCENT via MANUAL MAIN DEPLOY ", flight_exit_timer.read() + launch_timer_offset);
                    RecoveryTimer.start();
                }
                //Invalid command. Empty buffer and try again.
                else {
                    i = 0;
                    input[0] = 13;
                    pc.printf("\nInvalid Command\n\r");
                }
            }
            //DROGUE FIRE LOGIC///////////////////////
            //if the drogue fire window has been reached, and the GPS indicates we're at apogee,
            //fire pyros and move to recovery state
            if ( (flight_exit_timer.read() + launch_timer_offset) >= DROGUE_FIRE_WINDOW_START && altitude_qreg.getRSQ() > .8 && altitude_qreg.getVelocity() <= -0.01) {
                writelog("Attempt drogue auto-deploy via GPS at ", timestamp);
                drogue_fire();
                pc.printf("Drogues auto-deployed by GPS in ASCENT state ");
                pc.printf("with flight_exit_timer at %f secs\n\r", timestamp);
                writelog("To RECOVERY from ASCENT via drogue auto-deploy ", timestamp);
                //below is copied directly from the FLIGHT_EXIT_TIME conditional
                pc.printf("\nTransitioning to recovery state\n\r");
                //flight_exit_timer.stop();
                //flight_exit_timer.reset();
                //Restart the timer for the descent phase
                //flight_exit_timer.start();
                state = Recovery;
                RecoveryTimer.start();
                i = 0;
                input[0] = 13;
                int bytesWritten = flushGPSBuffer(); //TODO returns a size_t of how many bytes written. Need a better way to determine an unsuccessful flush
                if (bytesWritten < 0)
                    pc.printf("DEBUG: GPS flush failed to write %d bytes.\n");
                bytesWritten = flushAccelBuffer();
                if (bytesWritten < 0)
                    pc.printf("DEBUG: Accel flush failed to write %d bytes.\n");
                bytesWritten = flushIMUBuffer();
                if (bytesWritten < 0)
                    pc.printf("DEBUG: IMU flush failed to write %d bytes.\n");
            }
            // Fires drogues automatically if DROGUE_FIRE_WINDOW_END has been reached
            // not sure if this makes the FLIGHT_EXIT_TIME conditional transition irrelevant or not
            // if it doesn't, then this should be moved to the recovery state
            timestamp = flight_exit_timer.read() + launch_timer_offset;
            if (timestamp >= DROGUE_FIRE_WINDOW_END) {
                writelog("Drogue auto-deploy attempt by timer in ASCENT at ", timestamp);
                drogue_fire();
                pc.printf("Drogues auto-deployed by timer in ASCENT state ");
                pc.printf("with flight_exit_timer at %f secs\n\r", timestamp);
                writelog("To RECOVERY from ASCENT via drogue timer auto-deploy ", timestamp);
                //below is copied directly from the FLIGHT_EXIT_TIME conditional
                pc.printf("\nTransitioning to recovery state\n\r");
                //flight_exit_timer.stop();
                //flight_exit_timer.reset();
                //Restart the timer for the descent phase
                //flight_exit_timer.start();
                state = Recovery;
                RecoveryTimer.start();
                i = 0;
                input[0] = 13;
                int bytesWritten = flushGPSBuffer(); //TODO returns a size_t of how many bytes written. Need a better way to determine an unsuccessful flush
                if (bytesWritten < 0)
                    pc.printf("DEBUG: GPS flush failed to write %d bytes.\n");
                bytesWritten = flushAccelBuffer();
                if (bytesWritten < 0)
                    pc.printf("DEBUG: Accel flush failed to write %d bytes.\n");
                bytesWritten = flushIMUBuffer();
                if (bytesWritten < 0)
                    pc.printf("DEBUG: IMU flush failed to write %d bytes.\n");
            }
            //In case drogues STILL don't fire, the Raven fires shortly after the DROGUE_FIRE_WINDOW_END time
            //END DROGUE FIRE LOGIC/////////////
            break;
        }
        ////////////////////////////
        // RECOVERY STATE
        ///////////////////////////
        // Continue taking GPS and IMU readings, transmit position.
        case Recovery://recovery state
        {
            updateFlightLockFile();
            float timestamp = flight_exit_timer.read() + 1800 * exitTimerRollovers + launch_timer_offset;
            //only consider main pyro logic if past a certain time window
            //make sure time is absolute time of flight, although we shouldn't ever roll over
            //in the ideal case.
            if ((timestamp > MAIN_FIRE_WINDOW_START) && !MAINFIRED_FLAG) {
                //Main chute fire logic
                //if GPS altitude < 5000ft OR pressure sensor altitde < 5000ft, fire mains
                if ((altitude_g < 2.92 && (fixQuality_g > 1)) || altimeter.getAltitude() < 2920) {
                    if (altitude_g < 2.92 && (fixQuality_g > 1)) {
                        pc.printf("GPS: ");
                        writelog("Main auto-deploy in RECOVERY from GPS at ", timestamp);
                    }
                    if (altimeter.getAltitude() < 2920) {
                        pc.printf("Baro: ");
                        writelog("Main auto-deploy in RECOVERY from altimeter at ", timestamp);
                    }
                    main_fire();
                    MAINFIRED_FLAG = 1; //We have fired! don't fire again.
                    pc.printf("Mains auto-deployed in RECOVERY state ");
                    pc.printf("with flight_exit_timer at %f secs\n\r", timestamp);
                    writelog("Main auto-deploy in RECOVERY did not brown out ", timestamp);
                    remove("/local/lock.txt");
                }   //if this doesn't deploy the main, the Raven will deploy it
            }
            // State transitions
            while (pc.readable()) {
                input[i] = pc.getc();
                pc.printf("%c", input[i]);
                input[++i] = '\0';
            }

            readRecoveryInput(input, timestamp, state, i);

            // Timers overflow after ~33 mins. Reset timer and increment rollover counter.
            if (flight_exit_timer.read() >= 1800) {
                flight_exit_timer.reset();
                exitTimerRollovers++;
            }

            // Read, transmit, and save GPS pings as we descend
            timestamp = flight_exit_timer.read() + 1800 * exitTimerRollovers + launch_timer_offset;
            float baroAlt = altimeter.getAltitude();
            // int gpsQuality = gps.getGPSquality();
            //want to read a timer so we wait a second to print our altitude.
            if (RecoveryTimer.read() > 1) {
                RecoveryTimer.stop(); RecoveryTimer.reset();
                RecoveryTimer.start();
                if (fixQuality_g) {
                //  gps.geodetic(&gpsGeo);
                    if (altitude_g > max_altitude) {
                        max_altitude = altitude_g;
                    }
                    pc.printf("%4.4f %d %f %f %f %f %f\n\r", timestamp, fixQuality_g, latitude_g, longitude_g, altitude_g, baroAlt, max_altitude);
                }
                else {
                    pc.printf("</3: %4.4f %f %f %f %f %f\n\r", timestamp, latitude_g, longitude_g, altitude_g, baroAlt, max_altitude);
                }
            }
            if (timestamp < DESCENT_TIME) {
                imu_file = fopen("/local/imu.txt", "a");
                refresh_Timer.start();
                while (refresh_Timer.read() <= 1) {
                    timestamp = flight_exit_timer.read() + FLIGHT_EXIT_TIME + 1800 * exitTimerRollovers + launch_timer_offset;
                    imu.get_calib();
                    imu.get_angles();
                    imu.get_lia();
                    fprintf(imu_file, "%f %02x %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f\n", timestamp,
                        imu.calib,imu.euler.roll,imu.euler.pitch,imu.euler.yaw,
                        imu.lia.x,imu.lia.y,imu.lia.z);
                    wait_ms(50);
                }
                refresh_Timer.stop(); refresh_Timer.reset();
                gps_file = fopen("/local/gps.txt", "a");
                timestamp = flight_exit_timer.read() + FLIGHT_EXIT_TIME + 1800 * exitTimerRollovers + launch_timer_offset;
                fprintf(gps_file, "%4.4f %d %f %f %f %f\n", timestamp, fixQuality_g, latitude_g, longitude_g, altitude_g, baroAlt);
                fclose(gps_file);
                fclose(imu_file);
            }
            else {
                for (char i = 0; i < 20; i++) {
                  readRecoveryInput(input, timestamp, state, i);
                  wait_ms(50); //this causes us to not read commands well.
                }
            }
            break;
        }
        /////////////////
        // DEFAULT STATE
        ////////////////
        // Handles any weird unspecified states and returns to standby
        case Default:
        default:
        {
            pc.printf("\nERROR: In an unspecified state. Switching to Standby.\n\r");
            heartbeatTimer.start();
            state = Standby;
            writelog("Error: in unspecified state, switching to STANDBY ", -999);
            break;
        }

        }
        wait_us(4000); //This will set the speed of the loop
    if (i == 15 || (i != 0 && input[i-1] == 13)) {
        //pc.printf("\nInvalid Command\n\r");
        i = 0;
        input[0] = 13;
    }
    }
}


/*Functions to fire drogue and main parachutes*/
void drogue_fire() {
    pyro_fire_0 = 1;
    wait(.25);
    pyro_fire_0 = 0;
}

void main_fire() {
    pyro_fire_1 = 1;
    wait(.25);
    pyro_fire_1 = 0;
}
