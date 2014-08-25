#include <SD.h>
#include <avr/wdt.h>
#include <ctype.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
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
 "N xx - (Number) of bytes per frame; xx can be dec, hex, or oct, as ready by strtoi. xx must be > 0; unreasonably large values of xx will result in crashes\n"
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
 

const int ssPin = PIN_B0;

const int cameraFlashWindowPin = PIN_D0;
inline int cameraFlashValue() {
 return (PIND & _BV(0));
 // return !(PIND & _BV(0));
}

const int infraredLedControlPin = PIN_C0;
inline void irLightsOn() {PORTC |= _BV(0);}
inline void irLightsOff() {PORTC &= ~_BV(0);}

const int indicatorLedPin = PIN_D6; //led on teensy board
inline void indicatorOn() {PORTD |= _BV(6);}
inline void indicatorOff() {PORTD &= ~_BV(6);}
inline boolean indicatorIsOn() {return (PORTD & _BV(6));}

const int stimPin2 = PIN_C6; //note that output compare registers can't be used here because of mistaken connections (fix in future hardware revision)
const int stimPin1 = PIN_C5; //note that output compare registers can't be used here because of mistaken connections (fix in future hardware revision)

inline void stimulus2On() {PORTC |= _BV(6);}
inline void stimulus2Off() {PORTC &= ~_BV(6);}  
inline void setStimulus2(boolean on) {if (on) stimulus2On(); else stimulus2Off();}


inline void stimulus1On() {PORTC |= _BV(5);}
inline void stimulus1Off() {PORTC &= ~_BV(5);}  
inline void setStimulus1(boolean on) {if (on) stimulus1On(); else stimulus1Off();}

const int syncPin = PIN_B4;

const int notShdnLedPin = PIN_E0;
inline void ledsDisable() {PORTE &= ~_BV(0);}
inline void ledsEnable() {PORTE |= _BV(0);}

const boolean debug = false;

volatile boolean verbose = true;

volatile int countsSinceLastFrame = 17750; //expected rate of 71 ms
volatile boolean newFrame = false;

boolean output1enabled = false;
boolean output2enabled = false;


const int tnt1prescale = 64;
int tnt3prescale = 8;
int tnt3prescaleByte = 1;
unsigned int timer3countsPerByte;

int nBytesPerFrame = 8; //number of bits output per frame
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

boolean timer3setup = false;

boolean logarithmic = false;

float logTable[256];

void setup() {
  //first, turn off the clock prescaler, to get 16MHz operation
  noInterrupts();
  CLKPR = _BV(CLKPCE);
  CLKPR = 0;
  //clear reset source and disable watchdog
  MCUSR = 0;
  wdt_disable();
  interrupts();
    
  pinMode(indicatorLedPin, OUTPUT); 
  digitalWrite(indicatorLedPin, HIGH);
    
  pinMode(notShdnLedPin, OUTPUT);
  ledsEnable(); //modify for on-board led drivers
  
  //setup serial
  Serial.begin(9600);
  
  // set the slaveSelectPin as an output:
  pinMode (ssPin, OUTPUT);
  digitalWrite(ssPin, LOW);
  
  // set the camera flash window pin as an input with pulup
  pinMode(cameraFlashWindowPin, INPUT_PULLUP);
  
  //set the ir and stimulus led control pins as outputs
  pinMode(infraredLedControlPin, OUTPUT);
  digitalWrite(infraredLedControlPin, LOW);
  
  pinMode(stimPin1, OUTPUT);
  digitalWrite(stimPin1, LOW);
  pinMode(stimPin2, OUTPUT);
  digitalWrite(stimPin2, LOW);
  
  pinMode(syncPin, OUTPUT);
  digitalWrite(syncPin, LOW); //FIX LATER -- for now 1.6 default clock
  
  //start SD library
  SD.begin(ssPin);
  
  createLogTable();

  setupTimer1();
  disableTimer3Interrupts();
  setupTimer3(tnt3prescale);
  attachInterrupt(cameraFlashWindowPin, cameraFlash, CHANGE);
  
  digitalWrite(indicatorLedPin, LOW);
  
  wdt_enable(WDTO_8S); //reset every 8 seconds unless watchdog timer is reset

  for (int j = 0; j < 5; ++j) {
    wdt_reset();
    digitalWrite(indicatorLedPin, j%2);
   // digitalWrite(experimentAboutToStartIndicatorPin, j%2);
    delay(500);
  }
//  expIndOff();
  
 setNumBytesPerFrame(8);
 setPwmVal(0,0);

 enableTimer3Interrupts();
 /*
 Serial.print("pwmval1 = "); Serial.println(pwmval1);
 Serial.print("pwmval2 = "); Serial.println(pwmval2);
 
 Serial.print("ocr3a = "); Serial.println(OCR3A);
 Serial.print("ocr3b = "); Serial.println(OCR3B);
 Serial.print("ocr3c = "); Serial.println(OCR3C);
*/

}

ISR ( TIMER3_COMPA_vect ) {
  nextByte();
}

//B = 1; C = 2  
ISR ( TIMER3_COMPB_vect ) {
  stimulus1Off();
}
ISR ( TIMER3_COMPC_vect ) {
  stimulus2Off();
}

void createLogTable() {
  float maxx = log(255);
  logTable[0] = 0;
  for (int j = 1; j < 256; ++j) {
    logTable[j] = exp(j*maxx/255)/255.0;
  }
  logTable[255] = 1.0; //avoid any problems with almost theres
  
}
  
  
void cameraFlash() {
  
  if (!cameraFlashValue()) { //camera flashes low at start of cycle
    irLightsOn();
    
    unsigned char sreg = SREG;
    noInterrupts();
    countsSinceLastFrame = TCNT1; //Timer 1 counts clocks per frame
    TCNT1 = 0; //reset counter
    SREG = sreg;
    
    indicatorOn();
    startNewFrame();
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
    if (experimentRunning = (experimentStartCountdown == 0)) {
      experimentStartTime = millis();
      totalByteCount = 0;
    }
    //digitalWrite(experimentAboutToStartIndicatorPin, experimentRunning);
  }
  
  frameByteCount = 0;
  
  interrupts();
  updateTimer3();
  nextByte();
}

void nextByte() {
  setStimulus1(pwmval1 > 0);
  setStimulus2(pwmval2 > 0);
  
  OCR3B = pwmval1;
  OCR3C = pwmval2;
  if (pwmbyte1 == 255) {
    disableTimer3BInterrupt();
  } else {
    enableTimer3BInterrupt();
  }
  if (pwmbyte2 == 255) {
    disableTimer3CInterrupt();
  } else {
    enableTimer3CInterrupt();
  }
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
    pwmval1 = (unsigned int) (logTable[pwmb1] * timer3countsPerByte);
    pwmval2 = (unsigned int) (logTable[pwmb2] * timer3countsPerByte);
  } else {
    pwmval1 = (unsigned int) (pwmb1/255.0 * timer3countsPerByte);
    pwmval2 = (unsigned int) (pwmb2/255.0 * timer3countsPerByte);
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
  nBytesPerFrame = nbytes;
  float targetPreScale = nbytes > 2 ? 8 : 64;
  
  setupTimer3(targetPreScale);
}


void loop() {
  wdt_reset(); //pet the dog
  serialPoll();
  pollForNewByte();
  
}


void serialPoll() {
  static char buffer[SERIAL_CHARS_TO_BUFFER+1];
  buffer[SERIAL_CHARS_TO_BUFFER] = (char) '\0';
  static int len = 0;
  while (Serial.available() && len < (SERIAL_CHARS_TO_BUFFER)) {
    wdt_reset();
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
  Serial.print ("pwmval1 = ");
  Serial.println (pwmval1);
  Serial.print ("pwmval2 = ");
  Serial.println (pwmval2);
  Serial.print ("timer3 counts per byte = ");
  Serial.println(timer3countsPerByte);
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
  
  char filename[MAXFILECHARS];
  
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
  
  int n2r;
  for ( ;nbytes > 0;) {
    wdt_reset();
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
    for (long j = 0 ; j < nbytes; ++j) {
      wdt_reset();
      Serial.write((uint8_t) 0);
      if (!(j%64)) {
        Serial.flush();
      }
    }
      Serial.flush(); Serial.println();
      Serial.println("no file open");
      return true;
  }
  
  int n2w;
  boolean err = false;
  unsigned long target = nbytes;
  if (nbytes < 0) {
    target = sdfile.size();
  }
  
  for (unsigned long j = 0;j < target && !err; ++j) {
    wdt_reset();
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
 
 
void setupTimer1() {
  int prescalerValues[] = {1,8,64,256,1024};
  byte prescalerBytes[] = {1,2,3,4,5};
  int ps;
  for (ps = 0; ps < 5; ps++) {
    if (prescalerValues[ps] >= tnt1prescale) {
       break;
    }
  } 
  unsigned char sreg = SREG;
  noInterrupts();
  TCCR1B = 0; //stop clock
  TCNT1 = 0;
  TCCR1A = 0;
  TCCR1C = 0;
  TCCR1B |= prescalerBytes[ps]; //start clock with appropriate prescaler

  SREG = sreg; //re-enable interrupts if previously enabled
}
 
void setupTimer3(float targetPreScale) {
  int prescalerValues[] = {1,8,64,256,1024};
  byte prescalerBytes[] = {1,2,3,4,5};
  int ps;
  for (ps = 0; ps < 5; ps++) {
    if (prescalerValues[ps] >= targetPreScale - 0.001) {
       break;
    }
  } 
  tnt3prescale = prescalerValues[ps];
  tnt3prescaleByte = prescalerBytes[ps];
  unsigned char sreg = SREG;
  noInterrupts();
  TCCR3A = 0; //for CTC mode
  TCCR3B = _BV(WGM32);  //_BV(WGM33) | PWM
  TCCR3C = 0;
  SREG = sreg; //re-enable interrupts if previously enabled
}

void resetTimer3() {
  TCNT3 = 0;
}

void updateTimer3() {
  timer3countsPerByte = (unsigned int) (1.0*countsSinceLastFrame*(tnt1prescale/tnt3prescale)/nBytesPerFrame + 0.5); 
  unsigned char sreg = SREG;
  noInterrupts();
  TCCR3B = _BV(WGM32); //for CTC mode
  OCR3A = timer3countsPerByte;  
  TCNT3 = 0;
  TCCR3B |= tnt3prescaleByte; //start clock  
  SREG = sreg; //re-enable interrupts if previously enabled
 
  
}

inline void enableTimer3BInterrupt() {
  TIMSK3 |= _BV(OCIE3B);
}
inline void disableTimer3BInterrupt() {
  TIMSK3 &= ~_BV(OCIE3B); 
}
inline void enableTimer3CInterrupt() {
  TIMSK3 |= _BV(OCIE3C);
}
inline void disableTimer3CInterrupt() {
  TIMSK3 &= ~_BV(OCIE3C); 
}

inline void enableTimer3Interrupts() {
  TIMSK3 |= (_BV(OCIE3A ) | _BV(OCIE3B) | _BV(OCIE3C) ); 
  interrupts(); // make sure global interrupts are on
}

inline void disableTimer3Interrupts() {
  TIMSK3 &= (~_BV(OCIE3A ) & ~_BV(OCIE3B) & ~_BV(OCIE3C)); 
}








