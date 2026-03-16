#include <SD.h>
//#include <avr/wdt.h>
#include <ctype.h>
#include <stdlib.h>
//#include <avr/pgmspace.h>
/* LED-CAMERA COORDINATION SOFTWARE FOR TEENSY++ 2.0 and SD card addon
 * Copyright(C) 2012, 2014 Marc Gershow
 * Requires arduino v 1.05 or higher & teensyduino add-on
 */

/* Serial Commands
 * all commands on one line, terminate with \n or \r
 * R 1/2 filename - (Read) open file 1/2 for reading (use for experiment)
 * W filename - (Write) open file for writing
 * C - (Close) both files; done automatically at End of experiment
 * S 1/2 xxx - (Send) transfer xxx bytes from computer to file (specified by W) : xxx can be dec, hex (0x - zero x), or oct (leading 0 - zero)
 * F 1/2 xxx - (Fetch) transfer first xxx bytes from file to computer; if xxx is greater than number of bytes in file, remaining bytes will be 0 : xxx can be dec, hex (0x - zero x), or oct (leading 0 - zero)
 * S,F - xxx must be between 0 and 2^32-1 (maximum value of unsigned long); the conversion is handled by the c function strtoul, so xxx can be dec, hex (0x - zero x), or oct (leading 0 - zero)
 * O 1/2 x - (Output) 1/2 on (x > 0) or off (x == 0)
 *  note: opening a file for reading/writing automatically attempts to enable it
 * B - (Begin) experiment
 * E - (End) experiment
 * T xxx yyy - (sTimulus) set stimulus LED 1 to xxx and stimulus LED 2 to yyy
 * N xx - (Number) of bytes per frame; xx can be dec, hex, or oct, as ready by strtoi. xx must be > 0; unreasonably large values of xx will result in crashes
 * Y - (readY) - returns ready + details if ready to start experiment; returns error + details if not ready to start experiment
 * Q - (Quiet) - only respond to errors [default loud]
 * L - (Loud) - acknowledge all commands [default]
 * H - help - right, good luck with that guy
 * I - Info - returns 5 long integers: A B C D E\n
 *   A frame since experiment start if running; -999 if not running
 *   B time since experiment started in ms
 *   C position in file1 (note that experiment reads ahead)
 *   D position in file2 (note that experiment reads ahead)
 *   E number of bytes output to this point
 */
 
 prog_char helpmessage[] PROGMEM  = {"Serial Commands \n all commands on one line, terminate with \\n or \\r\n"
 "R 1/2 filename - (Read) open file 1/2 for reading (use for experiment)\n"
 "W filename - (Write) open file for writing\n"
 "C - (Close) both files; done automatically at End of experiment\n"
 "S 1/2 xxx - (Send) transfer xxx bytes from computer to file (specified by W) : xxx can be dec, hex (0x - zero x), or oct (leading 0 - zero)\n"
 "F 1/2 xxx - (Fetch) transfer first xxx bytes from file to computer; if xxx is greater than number of bytes in file, remaining bytes will be 0 : xxx can be dec, hex (0x - zero x), or oct (leading 0 - zero)\n"
 "S,F - xxx must be between 0 and 2^32-1 (maximum value of unsigned long); the conversion is handled by the c function strtoul, so xxx can be dec, hex (0x - zero x), or oct (leading 0 - zero)\n"
 "O 1/2 x - (Output) 1/2 on (x > 0) or off (x == 0)\n"
 "  note: opening a file for reading/writing automatically attempts to enable the corresponding output\n"
 "B - (Begin) experiment\n"
 "E - (End) experiment\n"
 "T xxx yyy - (sTimulus) set stimulus LED 1 to xxx and stimulus LED 2 to yyy\n"
 "N xx - IGNORED: Always 1 byte per frame (Number) of bytes per frame; xx can be dec, hex, or oct, as ready by strtoi. xx must be > 0; unreasonably large values of xx will result in crashes\n"
 "Y - (readY) - returns ready + details if ready to start experiment; returns error + details if not ready to start experiment\n"
 "Q - (Quiet) - only respond to errors [default loud]\n"
 "L - (Loud) - acknowledge all commands [default]\n"
 "H - help - this message\n"
 "I - Info - returns 5 long integers: A B C D E\\n\n"
 " A frame since experiment start if running; -999 if not running\n"
 " B time since experiment started in ms\n"
 " C position in file1 (note that experiment reads ahead)\n"
 " D position in file2 (note that experiment reads ahead)\n"
 " E number of bytes output to this point\n"};
 

//values reflect new interface board
// 10 -- EN_B
// 11 EN_R1
// 12 EN_R2
// 13 BASLER_GPIO3
// 14 BASLER_OPTOIN1
// 15 BASLER_GPIO4
// 16 BASLER_OPTOOUT2
// 17 ODL_NPN
//const int ssPin = BUILTIN_SDCARD; // sd card select
const int cameraFlashWindowPin = 16;
const int infraredLedControlPin = 17;
const int indicatorLedPin = LED_BUILTIN; //led on teensy board
const int stimPin2 = 10; //note that output compare registers can't be used here because of mistaken connections (fix in future hardware revision)
const int stimPin1 = 12; //note that output compare registers can't be used here because of mistaken connections (fix in future hardware revision)
const int nBytesPerFrame = 1;

volatile long numflashes = 0;
//TPS92205x - make frequency > 20k so you can't hear it
//https://e2e.ti.com/support/power-management-group/power-management/f/power-management-forum/1559650/faq-tps922051-how-to-reduce-acoustic-noise-of-mlccs-in-tps92205x-and-tps92365x-applications-with-pwm-dimming
//12 bit <= 36621 Hz
// 13 bit <= 18310 Hz
//https://www.pjrc.com/teensy/td_pulse.html
//TPS92055 is 11bit max at 20kHz (2,000:1 at 20-kHz PWM)
//experimentally, minimum pulse width = 300 ns
//13 kHz = 8 bits

const float pwm_freq = 120;
const int pwm_bit_res = 16;
const int max_pwm_value = 65535;
const int fullon = 255;
const int min_pwm_reliable = 8;



inline int cameraFlashValue() {
 return digitalRead(16);
}

inline void irLightsOn() {digitalWrite(infraredLedControlPin, HIGH);}
inline void irLightsOff() {digitalWrite(infraredLedControlPin, LOW);}

inline void indicatorOn() {digitalWrite(indicatorLedPin, HIGH);}
inline void indicatorOff() {digitalWrite(indicatorLedPin, LOW);}

inline void setStimulus2(int value) {analogWrite(stimPin2, value);}
inline void setStimulus1(int value) {
      analogWrite(stimPin1, value); 
return;
  static int lastvalue = 0;
  // digitalWrite(stimPin1, LOW); 
  // delayMicroseconds(10);
  // digitalWrite(stimPin1, value > 0);
  // delayMicroseconds(10); 
  // digitalWrite(stimPin1,LOW);
  // delayMicroseconds(10);
  if (value == lastvalue) {
    return;
  }
  lastvalue = value;
  if (value == 0) {
    digitalWrite(stimPin1, LOW);
    return;
  }
  if (value < min_pwm_reliable){
    analogWrite(stimPin1, min_pwm_reliable);
    delayMicroseconds(1000000/pwm_freq);
  }
  if (value > max_pwm_value - min_pwm_reliable) {
     analogWrite(stimPin1, max_pwm_value - min_pwm_reliable);
    delayMicroseconds(1000000/pwm_freq);
  }

  if (value >= max_pwm_value){
    digitalWrite(stimPin1, HIGH);
  } else {
    analogWrite(stimPin1, value); 
  }
  Serial.print("Stim1 = "); 
  Serial.println(value);
}


//const int notShdnLedPin = PIN_E0;
//inline void ledsDisable() {PORTE &= ~_BV(0);}
//inline void ledsEnable() {PORTE |= _BV(0);}

const boolean debug = false;

volatile boolean verbose = true;

//volatile int countsSinceLastFrame = 17750; //expected rate of 71 ms
//volatile int countsSinceLastFrame_old = 17750;
volatile boolean newFrame = false;
volatile boolean lastflash = false;

boolean output1enabled = false;
boolean output2enabled = false;



int frameByteCount = 0;
long int totalByteCount = 0;

const int bytesToPad = 16;
volatile int currentByteIndex1 = 0;
volatile int readIndex1 = 0;
volatile int currentByteIndex2 = 0;
volatile int readIndex2 = 0;

uint8_t byteBuffer1[bytesToPad];
uint8_t byteBuffer2[bytesToPad];


unsigned int pwmval1;
unsigned int pwmval2;
byte pwmbyte1;
byte pwmbyte2;

const int numFramesToPadExperimentStart = 10;
volatile int experimentStartCountdown = -1;
volatile boolean experimentRunning = false;

File sdfile1;
File sdfile2;
const int MAXFILECHARS = 255;
char filename1[MAXFILECHARS+1];
char filename2[MAXFILECHARS+1];


const int SERIALBUFFERSIZE = 64;
byte serialBuffer[SERIALBUFFERSIZE];
const int SERIAL_CHARS_TO_BUFFER = 255;

long int experimentStartTime = 0;
long int experimentElapsedFrames = -999;

long int frameCount = 0;

boolean logarithmic = false;

float logTable[256];



void createLogTable() {
  float maxx = log(255);
  logTable[0] = 0;
  for (int j = 1; j < 256; ++j) {
    logTable[j] = exp(j*maxx/255)/255.0;
  }
  logTable[255] = 1.0; //avoid any problems with almost theres
  
}
  

boolean cameraFlashValueDebounced(int numdebounces) {
  int flashval = 0;
  for (int j = 0; j < numdebounces; ++j) {
    flashval = flashval + cameraFlashValue();
  }
  return flashval > numdebounces / 2;
}

void cameraFlash() {
  boolean flashval = cameraFlashValueDebounced(7);
  if (flashval == lastflash) {
    return;
  }
  lastflash = flashval;
  
  
  
  if (!lastflash) { //camera flashes low at start of cycle
    irLightsOn();
    
    
    indicatorOn();
    startNewFrame();
    ++numflashes;
  } else {
    irLightsOff();
    indicatorOff();  
  }
  
}

void startNewFrame() {
  
 
  
  if (experimentRunning) {
    ++experimentElapsedFrames;
  }
  if ((experimentStartCountdown) > 0) {
    experimentStartCountdown--;
    experimentElapsedFrames = -experimentStartCountdown;
    experimentRunning = (experimentStartCountdown == 0);
    if (experimentRunning) {
      experimentStartTime = millis();
      totalByteCount = 0;
    }
    //digitalWrite(experimentAboutToStartIndicatorPin, experimentRunning);
  }
  
  frameByteCount = 0;
  nextByte();
}

void nextByte() {
  setStimulus1(pwmval1);
  setStimulus2(pwmval2);
  if (!experimentRunning || (++frameByteCount > nBytesPerFrame)) {
    return;
  } //keep us from putting too many bits into a frame if the camera is a little slow or the avr is a little fast
  ++totalByteCount;
  updatePwmVal();
}

inline void setPwmVal(byte pwmb1, byte pwmb2) {
  pwmbyte1 = pwmb1;
  pwmbyte2 = pwmb2;
  if (logarithmic) {
    pwmval1 = (unsigned int) (logTable[pwmb1] * max_pwm_value);
    pwmval2 = (unsigned int) (logTable[pwmb2] * max_pwm_value);
  } else {
    pwmval1 = (unsigned int) (pwmb1/255.0 * max_pwm_value);
    pwmval2 = (unsigned int) (pwmb2/255.0 * max_pwm_value);
  }
}
void updatePwmVal() {
  setPwmVal (byteBuffer1[currentByteIndex1],byteBuffer2[currentByteIndex2]);
  currentByteIndex1 = (currentByteIndex1+1)%bytesToPad;
  currentByteIndex2 = (currentByteIndex2+1)%bytesToPad;
}


boolean loadByteBuffers () {
  boolean error = false;
  if (output1enabled && !sdfile1) {
    output1enabled = false;
    return true;
  }
  if (output1enabled) {
    for (int j = 0; j < bytesToPad; ++j) {
      int rr = sdfile1.read();
      error |= (rr < 0);
      byteBuffer1[j] = rr;
    }
  } else {
     for (int j = 0; j < bytesToPad; ++j) {      
      byteBuffer1[j] = 0;
    }
  }
  if (output2enabled && !sdfile2) {
    output2enabled = false;
    return true;
  }
  if (output2enabled) {
    for (int j = 0; j < bytesToPad; ++j) {
      int rr = sdfile2.read();
      error |= (rr < 0);
      byteBuffer2[j] = rr;
    }
  } else {
     for (int j = 0; j < bytesToPad; ++j) {      
      byteBuffer2[j] = 0;
    }
  }
  currentByteIndex1 = readIndex1 = 0;
  currentByteIndex2 = readIndex2 = 0;
  return error;
}

void pollForNewByte () {
  if (!experimentRunning || (output1enabled && !sdfile1) || (output2enabled && !sdfile2)) {
    return;
  }
  if (output1enabled) {
    for ( ; readIndex1 != currentByteIndex1; readIndex1 = (readIndex1+1)%bytesToPad) {
      int rr = sdfile1.read();
      if (rr < 0) {
        endExperiment(); //on error, terminate experiment
      }
      byteBuffer1[readIndex1] = rr;
    }
  }
  if (output2enabled) {
    for ( ; readIndex2 != currentByteIndex2; readIndex2 = (readIndex2+1)%bytesToPad) {
      int rr = sdfile2.read();
      if (rr < 0) {
        endExperiment(); //on error, terminate experiment
      }
      byteBuffer2[readIndex2] = rr;
    }
  }
}

void setNumBytesPerFrame (int nbytes) {
//  nBytesPerFrame = 1;
}


void loop() {
  //wdt_reset(); //pet the dog
  serialPoll();
  pollForNewByte();  
}


void serialPoll() {
  static char buffer[SERIAL_CHARS_TO_BUFFER+1];
  buffer[SERIAL_CHARS_TO_BUFFER] = (char) '\0';
  static int len = 0;
  while (Serial.available() && len < (SERIAL_CHARS_TO_BUFFER)) {
    //wdt_reset();
    char c = Serial.read();
    if (isspace(c) && len <= 0) { //trim any leading whitespace
      continue;
    }
    if ('\n' == c || '\r' == c) {
     buffer[len] = '\0';
     len = 0;
    
     if (executeSerialCommand(buffer)) {
       Serial.print("Error: could not parse/execute your command: ");
       Serial.println(buffer);
     }
     continue;
    }
    buffer[len++] = c;
  }
  if (len >= SERIAL_CHARS_TO_BUFFER) {
    Serial.print("E: Serial Buffer Overrun: ");
    buffer[SERIAL_CHARS_TO_BUFFER] = '\0';
    Serial.println(buffer);
    len = 0;
  }
}

void printHelp() {
  Serial.println (helpmessage);
}



boolean executeSerialCommand(char *command) {
  boolean err = false;
  while (isspace(command[0])) {
    ++command;
  }
  switch(toupper(command[0])) {
    case 'R': 
      err = openFileForReading(command+1);
      
      break;
    case 'W': 
      err = openFileForWriting(command+1);
      
      break;
    case 'C':
      if (!closeFile()) {
        Serial.println("files closed");
      }
      break;
    case 'S':
      err = writeBytesFromSerialToFile(command+1);
      
      break;
    case 'F':
      err = readBytesFromFileToSerial(command+1);

      break;
    case 'B':
      err = startExperiment();
      if (verbose && !err) Serial.println("OK: started");
      break;
    case 'E':
      endExperiment();
      if (verbose) Serial.println("OK: ended");
      break;
    case 'T':
      err = setStimulus(command+1);
      if (verbose && !err) {
        Serial.print("OK: stimuli set to ");
        Serial.print(pwmbyte1); Serial.print (" , "); Serial.println (pwmbyte2);
      }
      break;
    case 'N':
      err = setNumBytesPerFrameFromString(command+1); 
      if (verbose && !err) {
         Serial.println();Serial.print("OK: "); Serial.print (nBytesPerFrame); Serial.println(" bytes per frame.");
      }
      
      break;
    case 'O':
      err = setOutputState(command + 1);
      break;
    case 'Q':
      verbose = false;
      break;
    case 'L':
      verbose = true;
      Serial.println ("OK: I'll be chatty");
      break;
    case '?': case 'H':
      printHelp();
      break;
    case 'Y':
      reportReady();
      break;
    case 'I':
      getInfo();
      break;
    case 'G':
      err = setLogarithmicState(command+1);
      break;
    case 'D':
      printDiagnostics();
      break;
    
      
    default:
      Serial.println("unrecognized command");
      err = true;
      break;
      }
  return err;
}

void printDiagnostics (void) {
  Serial.printf ("num flashes = ");
  Serial.println(numflashes);
  Serial.print ("pwmval1 = ");
  Serial.println (pwmval1);
  Serial.print ("pwmval2 = ");
  Serial.println (pwmval2);
}

boolean setOutputState(const char *command) {
  if (command == NULL) {
    return true;
  }

  while (isspace(command[0]) && command[0] != '\0') {
    ++command;
  }
  int fn = atoi(command);
  while (!isspace(command[0]) && command[0] != '\0') {
    ++command;
  }
  int x = atoi(command);
  if (fn < 1 || fn > 2 || x < 0) {
    return true;
  }
  boolean state = x > 0;
  if (fn == 1) {
    output1enabled = state;
    if (verbose) {
      Serial.print ("OK: Output 1 ");
      if (state) {
        Serial.println("enabled");
      } else {
        Serial.println("disabled");
      }
    }
  } 
  if (fn == 2) {
    output2enabled = state;
    if (verbose) {
      Serial.print ("OK: Output 2 ");
      if (state) {
        Serial.println("enabled");
      } else {
        Serial.println("disabled");
      }
    }
  }
  return false;
  
}


/* I - Info - returns 5 long integers: A B C D E\n
 *   A frame since experiment start if running; -999 if not running
 *   B time since experiment started in ms
 *   C position in file1 (note that experiment reads ahead)
 *   D position in file2 (note that experiment reads ahead)
 *   E number of bytes output to this point
 */
void getInfo() {
  Serial.print(experimentElapsedFrames); Serial.print('\t');
  if (experimentRunning) {
    Serial.print(millis() - experimentStartTime);
  } else {
    Serial.print (-1);
  }
  Serial.print('\t');
  if (sdfile1) {
    Serial.print(sdfile1.position());
  } else {
    Serial.print (-1);
  } 
  Serial.print('\t');
  if (sdfile2) {
    Serial.print(sdfile2.position());
  } else {
    Serial.print (-1);
  } 
  Serial.print('\t');
  Serial.println(totalByteCount);
  
  
}

boolean setNumBytesPerFrameFromString (const char *str) {
  int nb = atoi(str);
  if (nb <= 0 || nb > 150) {
    return true;
  }
  setNumBytesPerFrame (nb);
  return false;
}


boolean setLogarithmicState (const char *str) {
  int nb = atoi(str);
  if (nb < 0 ) {
    Serial.println("error: need a 0 or 1 to set state");
    return true;
  }
  logarithmic = (nb != 0);
  Serial.print("OK: output "); 
  if (logarithmic) {
    Serial.println("LOGARITHMIC");
  } else {
    Serial.println("LINEAR");
  }
  return false;
}

boolean openFileForReading(const char *command) {
  if (command == NULL) {
    return true;
  }

  while (isspace(command[0]) && command[0] != '\0') {
    ++command;
  }
  int fn = atoi(command);
  while (!isspace(command[0]) && command[0] != '\0') {
    ++command;
  }
  switch (fn) {
    case 1:  
    return (openFile1ForReading (command));
    break;
    case 2:
    return (openFile2ForReading (command));
    break;
    default:
    Serial.println("must specify file 1 or 2");
    return true;
    break;
  }
  return true;
}
boolean openFile1ForReading(const char *fname) {
  if (fname != NULL) {
    while (isspace(fname[0])) {
      ++fname;
    }
    strncpy(filename1, fname, MAXFILECHARS);
    filename1[MAXFILECHARS] = '\0'; //note size(filename) = MAXFILECHARS + 1
  } else {
    //if output1 has been disabled, don't enable it by opening file
    if (!output1enabled) {
      return false;
    }
  }
  if (sdfile1) {
    sdfile1.close();
  }
  sdfile1 = SD.open(filename1, FILE_READ);
  if (!sdfile1) {
    Serial.print("could not open "); Serial.println(filename1);
    return true;
  }
  output1enabled = sdfile1.peek() >= 0;
  if (verbose && output1enabled && fname != NULL) {
    Serial.print("OK: File "); Serial.print(filename1); Serial.print(" with "); Serial.print(sdfile1.size()); Serial.println (" open for reading output 1"); 
  }
  return (!output1enabled);
}

boolean openFile2ForReading(const char *fname) {
  if (fname != NULL) {
    while (isspace(fname[0])) {
      ++fname;
    }
    strncpy(filename2, fname, MAXFILECHARS);
    filename2[MAXFILECHARS] = '\0'; //note size(filename) = MAXFILECHARS + 1
  } else {
    //if output2 has been disabled, don't enable it by opening file
    if (!output2enabled) {
      return false;
    }
  }
  if (sdfile2) {
    sdfile2.close();
  }
  sdfile2 = SD.open(filename2, FILE_READ);
  if (!sdfile2) {
    Serial.print("could not open "); Serial.println(filename2);
    return true;
  }
  output2enabled = sdfile2.peek() >= 0;
  if (verbose && output2enabled && fname != NULL) {
    Serial.print("OK: File "); Serial.print(filename2); Serial.print(" with "); Serial.print(sdfile1.size()); Serial.println (" open for reading output 2"); 
  }
  return (!output2enabled);
}

boolean closeFile() {
  if (sdfile1) {
    sdfile1.close();
  }
  if (sdfile2) {
    sdfile2.close();
  }
  return false;
}

boolean openFileForWriting(const char *fname) {
  if (fname != NULL) {
    while (isspace(fname[0]) && fname[0] != '\0') {
      ++fname;
    }
  }
  int fn = atoi(fname);
  //Serial.print("fn = "); Serial.println(fn);
  
  if (fn < 1 || fn > 2) {
    Serial.println("must specify file 1 or 2");
    return true;
  }
  
  while (!isspace(fname[0]) && fname[0] != '\0') {
    ++fname;
  }
  //Serial.print("remaining command point1: ");Serial.println(fname);
  while (isspace(fname[0]) && fname[0] != '\0') {
    ++fname;
  }
  //Serial.print("remaining command point1: ");Serial.println(fname);
  
  char filename[MAXFILECHARS+1];
  
  strncpy(filename, fname, MAXFILECHARS);
  filename[MAXFILECHARS] = '\0'; //note size(filename) = MAXFILECHARS + 1
 // Serial.print("filename: "); Serial.println(filename);
  String d = String(filename);
  int ind = d.lastIndexOf('/');
  if (ind > 0) {
    d = d.substring(0,ind);
    char *dd =  (char *) d.c_str();
   // Serial.println(dd);
    if (!SD.exists(dd)) {
     // Serial.print("creating directory: ");
 //     Serial.println(dd);
      if (!SD.mkdir(dd)) {
        Serial.println("error");
        Serial.print("could not create directory: ");
        Serial.println(dd);
        return false;
      }
    }
  }
  boolean err = true;
 // Serial.println("hi");
  if (fn == 1) {
    err = false;
    if (sdfile1) {
      sdfile1.close();
    }
    sdfile1 = SD.open(filename, FILE_WRITE);
    if (!sdfile1) {
      Serial.print("could not open "); Serial.println(filename);
      err = true;
    } else {
      output1enabled = true;
    }
    strncpy(filename1, filename, MAXFILECHARS);
    filename1[MAXFILECHARS] = '\0'; //note size(filename) = MAXFILECHARS + 1
  } 
  if (fn == 2) {
    err = false;
    if (sdfile2) {
      sdfile2.close();
    }
    sdfile2 = SD.open(filename, FILE_WRITE);
    if (!sdfile2) {
      Serial.print("could not open "); Serial.println(filename);
      err = true;
    } else {
      output2enabled = true;
    }
    strncpy(filename2, filename, MAXFILECHARS);
    filename2[MAXFILECHARS] = '\0'; //note size(filename) = MAXFILECHARS + 1
  } 
  //Serial.println(filename);
  //Serial.println(filename1);
  //Serial.println(filename2);
  
  if (!err && verbose) {
    Serial.print("OK: File "); Serial.print(fn); Serial.print(": ");Serial.print(filename);  Serial.println (" open for writing"); 
  }
 
  return err;
}

int waitForSerialAvailable (unsigned long timeout) {
  unsigned long t0 = millis();
  int n2r;
  while (!(n2r = Serial.available())) { //single = is intentional
    if (millis() - t0 > timeout) {
      return -1;
    }
  }
  return min(SERIALBUFFERSIZE, n2r);
}

boolean writeBytesFromSerialToFile (const char *command) {
  if (command != NULL) {
    while (isspace(command[0]) && command[0] != '\0') {
      ++command;
    }
  }
  int fn = atoi(command);
  while (!isspace(command[0]) && command[0] != '\0') {
    ++command;
  }
  
  unsigned long nbytes = strtoul (command, NULL, 0);
  
  boolean err = true;
  if (fn == 1) {
    err = writeBytesFromSerialToFile (sdfile1, nbytes);
  }
  if (fn == 2) {
    err = writeBytesFromSerialToFile (sdfile2, nbytes);
  }
  if (verbose && !err) {
    Serial.print("OK: ");
    Serial.print(nbytes); Serial.print(" bytes written to file "); 
    Serial.print(fn);
    Serial.print(": ");
    if (fn == 1) {
      Serial.print(filename1); 
    } else {
      Serial.print(filename2);
    }
    Serial.print("  new file size is "); 
    if (fn == 1) {
      Serial.print(sdfile1.size());
    } else {        
      Serial.println(sdfile2.size());
    } 
  }
  if (fn < 1 || fn > 2) {
    Serial.println ("specify file 1 or 2"); 
  }
  return err;
    
}
boolean writeBytesFromSerialToFile (File sdfile, unsigned long nbytes) {
  if (!sdfile) {
    return true;
  }
  
  size_t n2r;
  for ( ;nbytes > 0;) {
    //wdt_reset();
    const int timeout = 1000;
    if ((n2r = waitForSerialAvailable(timeout)) <= 0) { //single = is intentional
      Serial.print ("timeout waiting for remaining "); Serial.print(nbytes); Serial.println(" bytes");
      return true;
    }
    Serial.readBytes((char *) serialBuffer, n2r);
    if (sdfile.write(serialBuffer, n2r) != n2r) {
      Serial.println("write error");
      return true;
    }
    nbytes -= n2r;
  }
  sdfile.flush();
  
  return false;
}

boolean setStimulus(const char *command) { //could update with some kind of check to make sure there is a number
  
  if (command != NULL) {
    while (isspace(command[0]) && command[0] != '\0') {
      ++command;
    }
  }
  int v1 = atoi(command);
  if (v1 < 0) {
    return true;
  }
  while (!isspace(command[0]) && command[0] != '\0') {
    ++command;
  }
  int v2 = atoi(command);
  if (v2 < 0) {
    v2 = 0;
  }
  setPwmVal(v1, v2);
 setStimulus1(pwmval1);
  setStimulus2(pwmval2);
  return false;
}

boolean readBytesFromFileToSerial (const char *command) {
  boolean v = verbose;
  verbose = false;
  
  if (command != NULL) {
    while (isspace(command[0]) && command[0] != '\0') {
      ++command;
    }
  }
  int fn = atoi(command);
  while (!isspace(command[0]) && command[0] != '\0') {
    ++command;
  }
  
  unsigned long nbytes = strtoul (command, NULL, 0);
  boolean err = true;
  if (fn == 1) {
    if (!sdfile1) {
      if (openFile1ForReading(NULL)) {
        Serial.println("could not open file 1 for reading");
        return true;
      }
    }   
    err = readBytesFromFileToSerial (sdfile1, nbytes);
  }
  if (fn == 2) {
    if (!sdfile2) {
      if (openFile2ForReading(NULL)) {
        Serial.println("could not open file 2 for reading");
        return true;
      }
    }
    err = readBytesFromFileToSerial (sdfile2, nbytes);
  }
  
  verbose = v;
  if (verbose && !err) {
    Serial.print("OK: ");
    Serial.print(nbytes); Serial.print(" bytes read from file "); 
    Serial.print(fn);
    Serial.print(": ");
    if (fn == 1) {
      Serial.println(filename1); 
    } else {
      Serial.println(filename2);
    }
  } 
  if (fn < 1 || fn > 2) {
    Serial.println ("specify file 1 or 2"); 
  }
  return err;
  
}
boolean readBytesFromFileToSerial (File sdfile, unsigned long &nbytes) {
  /*unsigned long */
  
  if (!sdfile) {
    for (unsigned long j = 0 ; j < nbytes; ++j) {
      //wdt_reset();
      Serial.write((uint8_t) 0);
      if (!(j%64)) {
        Serial.flush();
      }
    }
      Serial.flush(); Serial.println();
      Serial.println("no file open");
      return true;
  }
  
  boolean err = false;
  unsigned long target = nbytes;
  if (nbytes < 0) {
    target = sdfile.size();
  }
  
  for (unsigned long j = 0;j < target && !err; ++j) {
    //wdt_reset();
    int v =  sdfile.read(); 
    err |= (v < 0);  
    Serial.write((uint8_t) (max(0, v)));
    if (!(j%64)) {
      Serial.flush();
    }
  }
  Serial.flush();
  if (err) {
     Serial.println(); Serial.println ("insufficient bytes to read/read error");
  }
  
  return err;
}

boolean startExperiment() {
 if (!readyToStartExperiment()) {
  return true;
 }
 experimentStartCountdown = numFramesToPadExperimentStart;
 return false;
}

boolean readyToStartExperiment() {
  //closes file and reopens it; takes care of case that we had the file open for writing
  openFile1ForReading(NULL);
  openFile2ForReading(NULL);
  
  if (!output1enabled && !output2enabled) {
    return false;
  }
  if ((output1enabled && !sdfile1) || (output2enabled && !sdfile2)) {
    return false;
  }
  if (sdfile1) {
    sdfile1.seek(0);
  }
  if (sdfile2) {
    sdfile2.seek(0);
  }
  return !loadByteBuffers();
  
}

void reportReady() {
  if (readyToStartExperiment()) {
    Serial.println("READY BYTE PWM");
    Serial.print("bytes per frame = "); Serial.println(nBytesPerFrame);
  } else {
    Serial.println("ERROR");
  }
  Serial.print ("output 1 ");
  if (output1enabled) {
    Serial.println("enabled");
  } else {
    Serial.println("disabled");
  }
  Serial.println(filename1);
  if (!sdfile1) {
    Serial.println("file not open");
  } else {
    Serial.print(sdfile1.size()); Serial.println(" bytes");
  }
  
  Serial.print ("output 2 ");
  if (output2enabled) {
    Serial.println("enabled");
  } else {
    Serial.println("disabled");
  }
  Serial.println(filename2);
  if (!sdfile2) {
    Serial.println("file not open");
  } else {
    Serial.print(sdfile2.size()); Serial.println(" bytes");
  }  
  
  if (logarithmic) {
    Serial.println("output is LOGARITHMIC");
  } else {
    Serial.println("output is LINEAR");
  }
}

void endExperiment() {
  //disableTimer3Interrupt();
  experimentStartCountdown = -1;
  experimentRunning = false;
  experimentElapsedFrames = -999;
  if (output1enabled) {
    sdfile1.close();
  }
  if (output2enabled) {
    sdfile2.close();
  }
  
  setPwmVal(0,0);
}
 

void setup() {
  
  //wdt_disable();
  interrupts();
    
  pinMode(indicatorLedPin, OUTPUT); 
  digitalWrite(indicatorLedPin, HIGH);
    
  
  Serial.begin(9600);
  
  
  // set the camera flash window pin as an input with pullup
  pinMode(cameraFlashWindowPin, INPUT_PULLUP);
  
  //set the ir and stimulus led control pins as outputs
  pinMode(infraredLedControlPin, OUTPUT);
  digitalWrite(infraredLedControlPin, LOW);
  
  pinMode(stimPin1, OUTPUT);
  digitalWrite(stimPin1, LOW);
  pinMode(stimPin2, OUTPUT);
  digitalWrite(stimPin2, LOW);
  
  analogWriteFrequency(stimPin1,pwm_freq);
  analogWriteFrequency(stimPin2,pwm_freq);
  analogWriteResolution(pwm_bit_res);
  for (int j = 0; j < 256; ++j){
    setPwmVal(j,255-j);
    setStimulus1(pwmval1);
    setStimulus2(pwmval2);
    delay(5);
  }
  


  
  //start SD library
  SD.begin(BUILTIN_SDCARD);
  
  createLogTable();

  attachInterrupt(cameraFlashWindowPin, cameraFlash, CHANGE);
  
  digitalWrite(indicatorLedPin, LOW);
  
  //wdt_enable(WDTO_8S); //reset every 8 seconds unless watchdog timer is reset

  for (int j = 0; j < 5; ++j) {
    //wdt_reset();
    digitalWrite(indicatorLedPin, j%2);
    delay(500);
  }
//  expIndOff();
  
 setNumBytesPerFrame(1);
 setPwmVal(0,0);
}





