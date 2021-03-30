#include <math.h>

//Likely User Modified Variables ******************************

unsigned int deltaT = 1000;         // Sample period in microseconds.
int angleAverages = 5;
int past_size = 3;                 // interval for delta, larger=less noise, more delay.
unsigned long transferDt = 20000; // usecs between host updates  
                                 
// ***************************************************************

// Teensy definitions
#define angleSensorPin  A9
#define pwmVoltagePin A0
#define motorVoltagePin  A1
#define motorOutPWM  4
#define monitorPin 2  
#define dacRes 12   // Teensy 12-bit dac resolution, max val 4095
#define dacMax 4095
#define adcRes 14
#define adcCenter 8192 //Teensy ADC set to 14 bits, 16384/2 = 8192
#define adcMax 16383

// Circuit Specific Definitions.
#define scaleVadc 5.0 // All signals 0->5 volts.
#define vDrive 5.0 // Driver peak voltage.
#define rShunt 0.5 // Resistor from motor to ground.
#define rMotor 0.5 // Motor series resistance.

// Variables for Frequency Sweep
#define deg2rad 0.0175    
#define twopi 6.2831853  
#define MAX_ANG 5000
#define MAX_FREQ_PTS 40
#define firstFreq 0.25
#define lastFreq 4.0

float angles[MAX_ANG]; // storage for the responses
float motorCmds[MAX_ANG];
float freqs[MAX_FREQ_PTS];
float Fmultiplier;
float newFreq;
boolean newFreqFlg;
int freqIndex;
int periodInDts;
int numDts;
float sa, saOld, ca, caOld, sc, scOld, cc, ccOld; 

//Rest of Setup:
bool first_time;
String config_message_30_bytes = "&T~Freq~F4~0~10&T~Mag~F4~0~0.5&T~Phase~F4~-200~200&T~FreqCmd~F4~0~10&T~MagCmd~F4~0~1&T~PhaseCmd~F4~-200~200&D~100&H~4&";
String config_message = config_message_30_bytes;

float rad2deg = 1.0/deg2rad;        // 180/pi
 
float dTsec = 1.0e-6*deltaT;       // The period in seconds. 
float scaleD = 1.0/(dTsec*past_size); // Divide deltas by interval.    
                             
float errorVintegral;                // Variable for integrating angle errors.
float integralMax = 200;            // Maximum value of the integral

int loopCounter;

// Storage for past values.
float pastAngleV[20];  // Should be larger array than past_size.
char buf[60];  // 

// Variables for loop control
uint32_t loop_counter;
int numSkip;  // Number of loops to skip between host updates.
elapsedMicros loopTime; // Create elapsed time variable to ensure regular loops.
unsigned int headroom;  // Headroom is time left before loop takes too long.
boolean switchFlag;

// Initializes past values.
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  // Set up inputs
  analogReadResolution(adcRes);
  pinMode(motorVoltagePin, INPUT);
  pinMode(pwmVoltagePin, INPUT);
  pinMode(angleSensorPin, INPUT);

  // Set up output
  analogWriteResolution(dacRes);
  pinMode(motorOutPWM, OUTPUT);
  analogWriteFrequency(motorOutPWM, 23437.5); // Teensy 3.0 pin 3 also changes to 23437.5 kHz
  analogWrite(motorOutPWM, LOW);
  
  // Frequency Monitor
  pinMode(monitorPin, OUTPUT);

  // Number of loops between transfers to host.
  numSkip = max(int((transferDt+100)/deltaT),1); 

  first_time = false;

}

void loop() {  // Main code, runs repeatedly
  
  // Reinitializes loop if host disconnects.
  startup();
  
  // Make sure loop starts deltaT microsecs since last start
  unsigned int newHeadroom = max(int(deltaT) - int(loopTime), 0);
  headroom = min(headroom, newHeadroom);
  
  while (int(loopTime) < int(deltaT)) {};
  loopTime = 0;
  
  // Monitor Output should be a square wave with frequency = 1/(2*deltaT) 
  switchFlag = !switchFlag;
  digitalWrite(monitorPin, switchFlag);

  if(newFreqFlg) { // Initialize for next frequency point, make sure equals even number of dTsecs.
    newFreqFlg = false;
    periodInDts = 2*int(0.5/(dTsec*newFreq));
    if ((periodInDts < 10) || (freqIndex == MAX_FREQ_PTS)) {
      newFreq = firstFreq;
      freqIndex = 0; 
      periodInDts = 2*int(0.5/(dTsec*newFreq));
    }
    newFreq = float(1.0)/(dTsec*float(periodInDts)); // Make sure the period is even number dTsec's
    numDts = 0;
    sc = 0; cc = 0; scOld = 0; ccOld = 0;
    sa = 0; ca = 0; saOld = 0; caOld = 0;
  } else {
    numDts += 1;
  }

  float arg = (twopi*float(numDts))/float(periodInDts);
  float cosv = cos(arg);
  float sinv = sin(arg);
 
  // Read Angle, average to reduce noise.
  int angleI = 0;
  for(int i = 0; i < angleAverages; i++) angleI += analogRead(angleSensorPin);
  float angleV = scaleVadc*float(angleI- angleAverages*adcCenter)/float(angleAverages*adcMax);
  //angleV = 0;
  float angleVdesired = 0.15*sinv;

  //Need to change for your system!!!
  float directV = 1.4;

  
  float angleVderiv = (angleV-pastAngleV[past_size-1])*scaleD;

  /*************** Section of Likely User modifications.**********************/

  float motorCmd = 0.85*angleVdesired - 0.85*angleV - 0.05*angleVderiv;     
  float motorCmdFull = (motorCmd + directV)*dacMax/vDrive;

  int motorCmdInt = min(max(int(motorCmdFull),0), dacMax);
  analogWrite(motorOutPWM,motorCmdInt);
  //analogWrite(A14,int((motorCmdLim/vDrive)*dacMax));
  
  // Update previous errors for next time.
  for (int i = past_size-1; i > 0; i--) pastAngleV[i] = pastAngleV[i-1];
  pastAngleV[0] = angleV;

   /***********************************************/

  // Update the accumulating sine and cosine coeffs.
  float magA=0, magC=0;
  int numDtsMod = numDts % MAX_ANG;
  angles[numDtsMod] = angleV;
  motorCmds[numDtsMod] = motorCmd;
  sc += motorCmd*sinv;
  cc += motorCmd*cosv;
  sa += angleV*sinv;
  ca += angleV*cosv;
  int backAperiod = numDts - periodInDts; 
  if(backAperiod >= 0) { // Remove the first point if we've accumulated more than a period.
    backAperiod = backAperiod % MAX_ANG;
    sc -= motorCmds[backAperiod]*sinv; cc -=  motorCmds[backAperiod]*cosv;
    sa -= angles[backAperiod]*sinv;  ca -=  angles[backAperiod]*cosv;
    if (numDts % periodInDts == 0) { // Check convergence at periods (sin = 0).
        magC = sqrt(sc*sc + cc*cc);
        magA = sqrt(sa*sa + ca*ca);
        // If two periods or more AND longer than 3 seconds, then next freq.
        if (((numDts / periodInDts) > 1)  &&  (numDts > 3000000/int(deltaT))) { 
          newFreqFlg = true;
        }
        else {
          scOld = sc; saOld = sa;
          ccOld = cc; caOld = ca;
        }
     }
  }

  if(newFreqFlg) {  //  Just finished a frequency, dump out results
    float phaseSys = atan2(ca,sa)/deg2rad;
    float phaseCmd = atan2(cc,sc)/deg2rad;
    float periodInDtsF = float(periodInDts);
    magA /= periodInDtsF;
    magC /= periodInDtsF;
    packStatus(buf, newFreq, magA, phaseSys, newFreq, magC, phaseCmd, float(headroom));
    Serial.write(buf,30);
    newFreq *= Fmultiplier;
    freqIndex += 1;
  } 
  /*
  else if (loopCounter == numSkip) {  // Lines below are for debugging.
    packStatus(buf, float(0), float(0), float(0), motorCmd, angleV, angleVdesired, int(headroom));
    if(false) Serial.write(buf,30);
  }*/
  
  if(loopCounter == numSkip) loopCounter = 0;
  else loopCounter += 1;

}

void init_loop() {
  // Initialize loop variables
  loopCounter = 0; 
  headroom = deltaT;

  // Zero past errors
  for (int i = past_size-1; i >= 0; i--) pastAngleV[i] = 0;

  // Sweeper specific initializations.
  freqIndex = 0;
  newFreqFlg = true;
  newFreq = firstFreq;
  Fmultiplier = exp(log(lastFreq/firstFreq)/float(MAX_FREQ_PTS));
  headroom = deltaT; 
  loopTime = 0;
}


// Initializes the loop if this is the first time, or if reconnect sent
// from GUI.  Otherwise, just look for serial event.
void startup(){
    if (first_time) {
      while(Serial.available() > 0) Serial.read(); // Clear out rcvr.
      Serial.println(config_message);  // Send the configuration files.
      while (! Serial.available()) {}  // Wait for serial return.
      while(Serial.available() > 0) {   // Clear out rcvr.
        Serial.read();
      }
      init_loop();
      first_time = false;
    } else {
      serialEvent();
    }
}

// Simple serial event, only looks for disconnect character, resets loop if found.
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '~') {  // Got a disconnect!
      first_time = true;
      break;
    }
  }
}

// Load the serial output buffer.
void packStatus(char *buf, float a, float b, float c, float d, float e, float f, float g) {
  
  // Start Byte.
  buf[0] = byte(0);

  // dump in a->g
  int n = 1; 
  memcpy(&buf[n],&a,sizeof(a));
  n+=sizeof(a);
  memcpy(&buf[n],&b,sizeof(b));
  n+=sizeof(b);
  memcpy(&buf[n],&c,sizeof(c));
  n+=sizeof(c);
  memcpy(&buf[n],&d,sizeof(d));
  n+=sizeof(d);
  memcpy(&buf[n],&e,sizeof(e));
  n+=sizeof(e);
  memcpy(&buf[n],&f,sizeof(f));
  n+=sizeof(f);
  memcpy(&buf[n],&g,sizeof(g));
  n+=sizeof(g);

  // Stop byte (255 otherwise unused).
  buf[n] = byte(255); 
}

