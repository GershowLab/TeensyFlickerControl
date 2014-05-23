#include <SD.h>
#include <avr/wdt.h>
#include <ctype.h>
#include <stdlib.h>

/* LED-CAMERA COORDINATION SOFTWARE FOR TEENSY++ 2.0 and SD card addon
 * Copyright(C) 2012, 2014 Marc Gershow
 * Requires arduino v 1.05 or higher & teensyduino add-on
 */

/* Serial Commands
 * all commands on one line, terminate with \n or \r
 * R filename - (Read) open file for reading (use for experiment)
 * W filename - (Write) open file for writing
 * C - (Close) file; done automatically at End of experiment
 * S xxx - (Send) transfer xxx bytes from computer to file (specified by W) : xxx can be dec, hex (0x - zero x), or oct (leading 0 - zero)
 * F xxx - (Fetch) transfer first xxx bytes from file to computer; if xxx is greater than number of bytes in file, remaining bytes will be 0 : xxx can be dec, hex (0x - zero x), or oct (leading 0 - zero)
 * S,F - xxx must be between 0 and 2^32-1 (maximum value of unsigned long); the conversion is handled by the c function strtoul, so xxx can be dec, hex (0x - zero x), or oct (leading 0 - zero)
 * B - (Begin) experiment
 * E - (End) experiment
 * T x - (sTimulus) set stimulus LED off (x = 0) or on (x ~= 0);
 * N xx - (Number) of bits per frame; xx can be dec, hex, or oct, as ready by strtoi. xx must be > 0; unreasonably large values of xx will result in crashes
 * Y - (readY) - returns ready + details if ready to start experiment; returns error + details if not ready to start experiment
 * Q - (Quiet) - only respond to errors [default loud]
 * L - (Loud) - acknowledge all commands [default]
 * H - help - right, good luck with that guy
 */
 
 //comment this line out for red stimulation
 #define BLUE 1 

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

const int stimulusLedControlPin = PIN_C5; //OC3B
inline boolean stimulusLedControlPinIsOn() {return (PORTC & _BV(5));}
inline void stimulusOn() {PORTC |= _BV(5);}
inline void stimulusOff() {PORTC &= ~_BV(5);}
inline void setStimulus(boolean on) {if (on) stimulusOn(); else stimulusOff();}

const int syncPin = PIN_B4;

const int notShdnLedPin = PIN_E0;
inline void ledsDisable() {PORTE &= ~_BV(0);}
inline void ledsEnable() {PORTE |= _BV(0);}

const boolean debug = false;

volatile boolean verbose = true;

volatile int countsSinceLastFrame = 17750; //expected rate of 71 ms
volatile boolean newFrame = false;


const int tnt1prescale = 64;
int tnt3prescale = 8;
int tnt3prescaleByte = 1;
unsigned int timer3countsPerByte;

int nBytesPerFrame = 5; //number of bits output per frame
int frameByteCount = 0;
long int totalByteCount = 0;
const int bytesToPad = 16;
volatile int currentByteIndex = 0;
volatile int readIndex = 0;
uint8_t byteBuffer[bytesToPad];
unsigned int pwmval;
byte pwmbyte;

const int numFramesToPadExperimentStart = 10;
volatile int experimentStartCountdown = -1;
volatile boolean experimentRunning = false;

File sdfile;
const int MAXFILECHARS = 255;
char filename[MAXFILECHARS+1];

const int SERIALBUFFERSIZE = 64;
byte serialBuffer[SERIALBUFFERSIZE];

const int SERIAL_CHARS_TO_BUFFER = 255;

long int experimentStartTime = 0;
long int experimentElapsedFrames = -999;

long int frameCount = 0;

boolean timer3setup = false;

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
  //digitalWrite(notShdnLedPin, HIGH);
  //ledsDisable();
  ledsEnable();
  
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
  
  pinMode(stimulusLedControlPin, OUTPUT);
  digitalWrite(stimulusLedControlPin, LOW);
  
  pinMode(syncPin, OUTPUT);
  digitalWrite(syncPin, LOW); //FIX LATER -- for now 1.6 default clock
  
  //start SD library
  SD.begin(ssPin);
  
  

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
  
 setNumBytesPerFrame(5);
 // updateTimer3();
 setPwmVal(0);
 enableTimer3Interrupts();
 Serial.print("pwmval = "); Serial.println(pwmval);
 Serial.print("ocr3a = "); Serial.println(OCR3A);
 Serial.print("ocr3b = "); Serial.println(OCR3B);
 

}

ISR ( TIMER3_COMPA_vect ) {
  nextByte();
}

ISR ( TIMER3_COMPB_vect ) {
  stimulusOff();
//  setStimulus(OCR3A <= TCNT3);
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
  setStimulus(pwmval > 0);
  OCR3B = pwmval;
  if (pwmbyte == 255) {
    disableTimer3BInterrupt();
  } else {
    enableTimer3BInterrupt();
  }
//  if (pwmval > 0) {
  //  stimulusOn();
  //}
  if (!experimentRunning || (++frameByteCount > nBytesPerFrame)) {
    return;
  } //keep us from putting too many bits into a frame if the camera is a little slow or the avr is a little fast
  
  //OCR3B = pwmval;  

  ++totalByteCount;
  updatePwmVal();
//  expIndOff();
}

inline void setPwmVal(byte pwmb) {
  pwmbyte = pwmb;
  pwmval = (unsigned int) (pwmb/255.0 * timer3countsPerByte);
}
void updatePwmVal() {
  setPwmVal (byteBuffer[currentByteIndex]);
//  pwmbyte = 
 // pwmval = (unsigned int) (((float) byteBuffer[currentByteIndex])/255.0*timer3countsPerByte);
  currentByteIndex = (currentByteIndex+1)%bytesToPad;
}


boolean loadByteBuffer () {
  boolean error = false;
  if (!sdfile) {
    return true;
  }
  for (int j = 0; j < bytesToPad; ++j) {
    int rr = sdfile.read();
    error |= (rr < 0);
    byteBuffer[j] = rr;
  }
  currentByteIndex = readIndex = 0;
  updatePwmVal();
  return error;
}

void pollForNewByte () {
  if (!experimentRunning || !sdfile) {
    return;
  }
  for ( ; readIndex != currentByteIndex; readIndex = (readIndex+1)%bytesToPad) {
    int rr = sdfile.read();
    if (rr < 0) {
      endExperiment(); //on error, terminate experiment
    }
    byteBuffer[readIndex] = rr;
  }
}

void setNumBytesPerFrame (int nbytes) {
  nBytesPerFrame = nbytes;
  
//  float targetPreScale = countsSinceLastFrame/65000.0*tnt1prescale / nbytes;  
  //float targetPreScale = 1.0*tnt1prescale / nbytes;  
  
  float targetPreScale = nbytes > 2 ? 8 : 64;
  
  setupTimer3(targetPreScale);
}


void loop() {
  wdt_reset(); //pet the dog
  serialPoll();
  pollForNewByte();
  /*
  if (timer3setup == false && millis() > 5000 ) {
    updateTimer3();
    timer3setup = true;
  }
  */
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
  Serial.println ("look in the sketch for help!");
}


/* Serial Commands
 * all commands on one line, terminate with \n or \r
 * R filename - (Read) open file for reading (use for experiment)
 * W filename - (Write) open file for writing
 * S xxx - (Send) transfer xxx bytes from computer to file (specified by W) : xxx can be dec, hex (0x - zero x), or oct (leading 0 - zero)
 * F xxx - (Fetch) transfer first xxx bytes from file to computer; if xxx is greater than number of bytes in file, remaining bytes will be 0 : xxx can be dec, hex (0x - zero x), or oct (leading 0 - zero)
 * S,F - xxx must be between 0 and 2^32-1 (maximum value of unsigned long); the conversion is handled by the c function strtoul, so xxx can be dec, hex (0x - zero x), or oct (leading 0 - zero)
 * B - (Begin) experiment
 * E - (End) experiment
 * N xx - (Number) of bits per frame; xx can be dec, hex, or oct, as ready by strtoi. xx must be > 0; unreasonably large values of xx will result in crashes
 * I - Info - returns 4 long integers: A B C D \n
 *   A frame since experiment start if running; -999 if not running
 *   B time since experiment started in ms
 *   C position in file (note that experiment reads ahead 1 byte)
 *   D number of bits output to this point
 * Q - (Quiet) - only respond to errors [default loud]
 * L - (Loud) - acknowledge all commands [default]
 * T x - (sTimulus) set stimulus LED off (x = 0) or on (x ~= 0);
 * Y - (readY) - returns ready + details if ready to start experiment; returns error + details if not ready to start experiment
 * H - help - right, good luck with that guy
 */


boolean executeSerialCommand(char *command) {
  boolean err = false;
  while (isspace(command[0])) {
    ++command;
  }
  long nbytes;
  switch(toupper(command[0])) {
    case 'R': 
      err = openFileForReading(command+1);
      if (!err && verbose) {
        Serial.print("OK: File "); Serial.print(filename); Serial.print(" with "); Serial.print(sdfile.size()); Serial.println (" open for reading"); 
      }
      break;
    case 'W': 
      err = openFileForWriting(command+1);
      if (!err && verbose) {
        Serial.print("OK: File "); Serial.print(filename);  Serial.println (" open for writing"); 
      }
      break;
    case 'C':
      if (!closeFile()) {
        Serial.println("file closed");
      }
      break;
    case 'S':
      err = writeBytesFromSerialToFile(command+1);
      if (verbose && !err) {
        Serial.print("OK: Bytes written to "); Serial.print(filename); Serial.print("  new file size is "); Serial.println(sdfile.size()); 
      }
      break;
    case 'F':
      err = readBytesFromFileToSerial(command+1, nbytes);
      if (verbose && !err && nbytes > 0) {
        Serial.print("OK: Tranfer of "); Serial.print(nbytes); Serial.print(" bytes from "); Serial.print(filename); Serial.println(" complete"); 
      }
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
        Serial.print("OK: stimulus");
        if (stimulusLedControlPinIsOn()) {
          Serial.println(" on");
        } else {
          Serial.println(" off");
        }
      }
      break;
    case 'N':
      err = setNumBytesPerFrameFromString(command+1); 
      if (verbose && !err) {
         Serial.println();Serial.print("OK: "); Serial.print (nBytesPerFrame); Serial.println(" bytes per frame.");
      }
      
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
      PINE = 1;
      break;
    default:
      Serial.println("unrecognized command");
      err = true;
      break;
      }
  return err;
}
/* I - Info - returns 4 long integers: A B C D \n
 *   A frame since experiment start if running; -999 if not running
 *   B time since experiment started in ms
 *   C position in file (note that experiment reads ahead 1 byte)
 *   D number of bits output to this point
 */
void getInfo() {
  Serial.print(experimentElapsedFrames); Serial.print('\t');
  if (experimentRunning) {
    Serial.print(millis() - experimentStartTime);
  } else {
    Serial.print (-1);
  }
  Serial.print('\t');
  if (sdfile) {
    Serial.print(sdfile.position());
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

boolean openFileForReading(const char *fname) {
  if (fname != NULL) {
    while (isspace(fname[0])) {
      ++fname;
    }
    strncpy(filename, fname, MAXFILECHARS);
    filename[MAXFILECHARS] = '\0'; //note size(filename) = MAXFILECHARS + 1
  }
  if (sdfile) {
    sdfile.close();
  }
  sdfile = SD.open(filename, FILE_READ);
  if (!sdfile) {
    Serial.print("could not open "); Serial.println(filename);
    return true;
  }
  return (sdfile.peek() < 0);
  
}
boolean closeFile() {
  if (sdfile) {
    sdfile.close();
  }
  return false;
}

boolean openFileForWriting(const char *fname) {
  while (isspace(fname[0])) {
    ++fname;
  }
  strncpy(filename, fname, MAXFILECHARS);
  filename[MAXFILECHARS] = '\0'; //note size(filename) = MAXFILECHARS + 1
  
  String d = String(filename);
  int ind = d.lastIndexOf('/');
  if (ind > 0) {
    d = d.substring(0,ind);
    char *dd =  (char *) d.c_str();
    Serial.println(dd);
    if (!SD.exists(dd)) {
      if (!SD.mkdir(dd)) {
        return false;
      }
    }
  }

  
  if (sdfile) {
    sdfile.close();
  }
  sdfile = SD.open(filename, FILE_WRITE);
  if (!sdfile) {
    Serial.print("could not open "); Serial.println(filename);
    return true;
  }
  return false;
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
  unsigned long nbytes = strtoul (command, NULL, 0);
  
  if (!sdfile) {
    if (openFileForWriting("tmpfile.bin")) {
      return true;
    }
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
  //digitalWrite(stimulusLedControlPin, atoi(command));
  setPwmVal(atoi(command));
  
  return false;
}
//void setStimulusValue(byte val) {
//  pwmval = min((unsigned int) (1.0*val/255*timer3countsPerByte), OCR3A);
//}

boolean readBytesFromFileToSerial (const char *command, long &nbytes) {
  /*unsigned long */
  nbytes = strtoul (command, NULL, 0);
  if (!sdfile) {
    openFileForReading(NULL);
  }
  
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
  
  for (unsigned long j = 0;j < target; ++j) {
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
 //enableTimer3Interrupt();
 return false;
}

boolean readyToStartExperiment() {
  //closes file and reopens it; takes care of case that we had the file open for writing
  if (openFileForReading(NULL)) {
    return false;
  }
  if (!sdfile) {
    return false;
  }
  sdfile.seek(0);
  
  return !loadByteBuffer();
  
}

void reportReady() {
  if (readyToStartExperiment()) {
    Serial.println("READY BYTE PWM");
    Serial.println(filename);
    Serial.print(sdfile.size()); Serial.println(" bytes");
    Serial.print("bits per frame = "); Serial.println(nBytesPerFrame);
  } else {
    Serial.println("ERROR");
    Serial.println(filename);
    if (!sdfile) {
      Serial.println("file not open");
    } else {
      Serial.print(sdfile.size()); Serial.println(" bytes");
    }
  }
  
}

void endExperiment() {
  //disableTimer3Interrupt();
  experimentStartCountdown = -1;
  experimentRunning = false;
  experimentElapsedFrames = -999;
  sdfile.close();
  //stimulusOff();
  setPwmVal(0);
 // expIndOff();
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

  SREG = sreg;
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
 //setup for fast PWM mode
  //TCCR3A = (_BV(COM3B1) | _BV(WGM31) | _BV(WGM30)); old - clear on compare, set on top
  //TCCR3A = (_BV(COM3B1) | _BV(COM3B0) | _BV(WGM31) | _BV(WGM30)); //new set on compare, clear on top
  TCCR3B = _BV(WGM32);  //_BV(WGM33) | PWM
  TCCR3C = 0;
  SREG = sreg;
}
void resetTimer3() {
  TCNT3 = 0;
}

void updateTimer3() {
  timer3countsPerByte = (unsigned int) (1.0*countsSinceLastFrame*(tnt1prescale/tnt3prescale)/nBytesPerFrame + 0.5); 
  unsigned char sreg = SREG;
  noInterrupts();
  //TCCR3B = _BV(WGM33) | _BV(WGM32);  //set to FAST PWM mode; stop counter
  TCCR3B = _BV(WGM32); //for CTC mode
  OCR3A = timer3countsPerByte;  
  //TCNT3 = OCR3A - 2; //start it early so it triggers a reset correctly
  TCNT3 = 0;
  TCCR3B |= tnt3prescaleByte; //start clock  
  SREG = sreg; 
 // TCCR3C |= _BV(FOC3A); //force output compare
 // stimulusOn();
  
}

inline void enableTimer3BInterrupt() {
  TIMSK3 |= _BV(OCIE3B);
}
inline void disableTimer3BInterrupt() {
  TIMSK3 &= ~_BV(OCIE3B); // disable CTC interrupt
}

inline void enableTimer3Interrupts() {
  TIMSK3 |= (_BV(OCIE3A ) | _BV(OCIE3B) ); // Enable CTC interrupt
  interrupts(); // make sure global interrupts are on
}
inline void disableTimer3Interrupts() {
  TIMSK3 &= (~_BV(OCIE3A ) & ~_BV(OCIE3B)); // disable CTC interrupt
}








