/* Satellite Orbit Tracking Pointer
Grady Hillhouse 2015
*/

#include "mbed.h"
#include "sgp4ext.h"
#include "sgp4unit.h"
#include "sgp4io.h"
#include "sgp4coord.h"
#include "Adafruit_MotorShield.h"
#include "Servo.h"

Serial pc(SERIAL_TX, SERIAL_RX);
Servo EL_SERVO(D9);



//Function prototypes
float Convert_El_to_Servo(float elevation);


int main()
{
    //INITIALIZE STEPPER MOTOR
    Adafruit_MotorShield AFMS = Adafruit_MotorShield();
    Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 1);
    AFMS.begin(2400);  // create with the frequency 2.4KHz
    myMotor->setSpeed(1000);  // rpm (this has an upper limit way below 1000 rpm)
    myMotor->release();
    
    //SET UP SOME VARIABLES
    double ro[3];
    double vo[3];
    double recef[3];
    double vecef[3];
    char typerun, typeinput, opsmode;
    gravconsttype  whichconst;
    
    double sec, secC, jd, jdC, tsince, startmfe, stopmfe, deltamin;
    double tumin, mu, radiusearthkm, xke, j2, j3, j4, j3oj2;
    double latlongh[3]; //lat, long in rad, h in km above ellipsoid
    double siteLat, siteLon, siteAlt, siteLatRad, siteLonRad;
    double razel[3];
    double razelrates[3];
    int  year; int mon; int day; int hr; int min;
    int yearC; int monC; int dayC; int hrC; int minC;
    typedef char str3[4];
    str3 monstr[13];
    elsetrec satrec;
    double steps_per_degree = 1.38889; //Stepper motor steps per degree azimuth
    float elevation;
    
    //VARIABLES FOR STEPPER CALCULATIONS
    float azimuth; //-180 to 0 to 180
    float prevAzimuth;
    float cAzimuth; //From 0 to 359.99
    float prevcAzimuth;
    bool stepperRelative = 0; //Has the stepper direction been initialized?
    float azimuthDatum = 0;
    int stepsFromDatum = 0;
    int stepsNext = 0;
    int dirNext = 1;
    int totalSteps = 0;
    int prevDir = 3; //Initialize at 3 to indicate that there is no previous direction yet (you can't have a "previous direction" until the third step)
    double azError = 0;
    
    //SET REAL TIME CLOCK (Set values manually using custom excel function until I find a way to do it automatically)    
    set_time(1440763200);
    
    //SET VARIABLES
    opsmode = 'i';
    typerun = 'c';
    typeinput = 'e';
    whichconst = wgs72;
    getgravconst( whichconst, tumin, mu, radiusearthkm, xke, j2, j3, j4, j3oj2 );
    strcpy(monstr[1], "Jan");
    strcpy(monstr[2], "Feb");
    strcpy(monstr[3], "Mar");
    strcpy(monstr[4], "Apr");
    strcpy(monstr[5], "May");
    strcpy(monstr[6], "Jun");
    strcpy(monstr[7], "Jul");
    strcpy(monstr[8], "Aug");
    strcpy(monstr[9], "Sep");
    strcpy(monstr[10], "Oct");
    strcpy(monstr[11], "Nov");
    strcpy(monstr[12], "Dec");
    
    //ENTER TWO-LINE ELEMENT HERE
    char longstr1[] = "1 25544U 98067A   15239.40934558  .00012538  00000-0  18683-3 0  9996";
    char longstr2[] = "2 25544  51.6452  88.4122 0001595  95.9665 324.8493 15.55497522959124";
    
    //ENTER SITE DETAILS HERE
    siteLat = 30.25; //+North (Austin)
    siteLon = -97.75; //+East (Austin)
    siteAlt = 0.15; //km (Austin)
    siteLatRad = siteLat * pi / 180.0;
    siteLonRad = siteLon * pi / 180.0;
    
    //FREEDOM OF MOVEMENT CHECKS
    
    for (int i = 0; i < 500; i = i + 10) {
        EL_SERVO = Convert_El_to_Servo(-90.0 + 180.0 * i / 500.0);
    }
    wait(1);

    for (int i = 500; i > 0; i = i - 10) {
        EL_SERVO = Convert_El_to_Servo(-90.0 + 180.0 * i / 500.0);
    }    
    wait(1);
    
    /*
    //FREEDOM OF MOVEMENT CHECKS STEPPER
    myMotor->step(500, FORWARD, SINGLE);
    myMotor->step(500, BACKWARD, SINGLE);
    */
    
    //INITIALIZE SATELLITE TRACKING    
    //pc.printf("Initializing satellite orbit...\n");
    twoline2rv(longstr1, longstr2, typerun, typeinput, opsmode, whichconst, startmfe, stopmfe, deltamin, satrec );
    //pc.printf("twoline2rv function complete...\n");
    //Call propogator to get initial state vector value
    sgp4(whichconst, satrec, 0.0, ro, vo);
    //pc.printf("SGP4 at t = 0 to get initial state vector complete...\n"); 
    jd = satrec.jdsatepoch;    
    
    invjday(jd, year, mon, day, hr, min, sec);
    pc.printf("Scenario Epoch   %3i %3s%5i%3i:%2i:%12.9f \n", day, monstr[mon], year, hr, min, sec);
    jdC = getJulianFromUnix(time(NULL));
    invjday( jdC, yearC, monC, dayC, hrC, minC, secC);
    pc.printf("Current Time    %3i %3s%5i%3i:%2i:%12.9f \n", dayC, monstr[monC], yearC, hrC, minC, secC);
    //pc.printf("            Time            PosX            PosY            PosZ              Vx              Vy              Vz\n");
    //pc.printf("            Time             Lat            Long          Height           Range         Azimuth       Elevation\n");
    
    //BEGIN SATELLITE TRACKING
    while(1)
    {
        
        //RUN SGP4 AND COORDINATE TRANSFORMATION COMPUTATIONS
        jdC = getJulianFromUnix(time(NULL));
        tsince = (jdC - jd) * 24.0 * 60.0;
        sgp4(whichconst, satrec, tsince, ro, vo);
        teme2ecef(ro, vo, jdC, recef, vecef);
        ijk2ll(recef, latlongh);
        rv2azel(ro, vo, siteLatRad, siteLonRad, siteAlt, jdC, razel, razelrates);
        
        //CHECK FOR ERRORS
        if (satrec.error > 0)
        {
            pc.printf("# *** error: t:= %f *** code = %3d\n", satrec.t, satrec.error);
        }
        else
        {
            azimuth = razel[1]*180/pi;
            if (azimuth < 0) {
                cAzimuth = 360.0 + azimuth;
            }
            else {
                cAzimuth = azimuth;
            }      
            elevation = razel[2]*180/pi;
            
            //pc.printf("%16.8f%16.8f%16.8f%16.8f%16.8f%16.8f%16.8f\n", satrec.t, recef[0], recef[1], recef[2], vecef[0], vecef[1], vecef[2]);
            //pc.printf("%16.8f%16.8f%16.8f%16.8f%16.8f%16.8f%16.8f\n", satrec.t, latlongh[0]*180/pi, latlongh[1]*180/pi, latlongh[2], razel[0], razel[1]*180/pi, razel[2]*180/pi);
            
            //For first step, initialize the stepper direction assuming its initial position is true north
            if (stepperRelative == 0){
                stepsNext = int(cAzimuth * steps_per_degree);
                dirNext = 2;                
                myMotor->step(stepsNext, dirNext, MICROSTEP); //Turn stepper clockwise to approximate initial azimuth
                stepperRelative = 1;
                azimuthDatum = stepsNext / steps_per_degree;
                prevAzimuth = azimuth;
                prevcAzimuth = cAzimuth;
                
                pc.printf("             Azimuth       Azimuth Datum    Steps from Datum         Total Steps          Steps Next           Direction          Az. Error\n");
            }
            else {
                
                //Determine direction of rotation (note this will be incorrect if azimuth has crossed true north since previous step - this is dealt with later)
                if ( cAzimuth < prevcAzimuth ) {
                    dirNext = 1; //CCW
                }
                else {
                    dirNext = 2; //CW
                }
                
                
                //Check if azimuth has crossed from 360 to 0 degrees or vice versa
                if (abs( (azimuth - prevAzimuth) - (cAzimuth - prevcAzimuth) ) > 0.0001) {
                    
                    //Recalculate direction of rotation
                    if ( cAzimuth > prevcAzimuth ) {
                        dirNext = 1; //CCW
                    }
                    else {
                        dirNext = 2; //CW
                    }
                    
                    //Reset the azimuth datum
                    if (dirNext == 1) {
                        azimuthDatum = cAzimuth + azError + prevcAzimuth;
                    }
                    else {
                        azimuthDatum = cAzimuth - azError + (prevcAzimuth - 360);
                    }
                    
                    //Reset totalSteps
                    totalSteps = 0;
                }
                
                
                //Check if azimuth rate has changed directions
                if (prevDir != 3) { //prevDir of 3 means there is no previous direction yet
                    
                    if (prevDir != dirNext) {
                        
                        //Reset totalSteps
                        totalSteps = 0;
                        
                        //Reset azimuth datum
                        if (dirNext == 1) {
                            azimuthDatum = prevcAzimuth + azError;
                        }
                        else {
                            azimuthDatum = prevcAzimuth - azError;
                        }
                        
                    }
                    
                }
                
                
                stepsFromDatum = int( abs(cAzimuth - azimuthDatum) * steps_per_degree );
                stepsNext = stepsFromDatum - totalSteps;
                totalSteps += stepsNext;
                azError = abs(cAzimuth - azimuthDatum) - (totalSteps / steps_per_degree);
                                
                pc.printf("%20.2f%20.2f%20d%20d%20d%20d%20.2f\n", cAzimuth, azimuthDatum, stepsFromDatum, totalSteps, stepsNext, dirNext, azError);
                
                if (stepsNext > 250) {
                
                    pc.printf("something's probably wrong... too many steps\n\n\n\n");
                    while(1){} // pause
                    
                }
                
                myMotor->step(stepsNext, dirNext, MICROSTEP);               
            }
                        
            EL_SERVO = Convert_El_to_Servo(elevation);
            prevAzimuth = azimuth;
            prevcAzimuth = cAzimuth;
            prevDir = dirNext;
        }
        
        wait(1);
        
    } //indefinite loop
    
}

float Convert_El_to_Servo(float elevation) {
    
    float servo_90     = 0.71;     //Calibrated to servo
    float servo_0      = 0.445;    //Calibrated to servo
    float servo_min    = 0.186;
    float servo_ratio;
    
    //Interpolate servo ratio
    servo_ratio = servo_0 + (servo_90 - servo_0) * (elevation) / (90);
    if (servo_ratio < servo_min) {
        return servo_min;
    }
    else {
        return servo_ratio;
    }
}