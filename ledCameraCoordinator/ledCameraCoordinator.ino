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
 

const int ssPin = PIN_B0;

const int cameraFlashWindowPin = PIN_D0;
inline int cameraFlashValue() {
 return (PIND & _BV(0));
}

const int infraredLedControlPin = PIN_C0;
inline void irLightsOn() {PORTC |= _BV(0);}
inline void irLightsOff() {PORTC &= ~_BV(0);}

const int indicatorLedPin = PIN_D6; //led on teensy board
inline void indicatorOn() {PORTD |= _BV(6);}
inline void indicatorOff() {PORTD &= ~_BV(6);}
inline boolean indicatorIsOn() {return (PORTD & _BV(6));}

const int stimulusLedControlPin = PIN_F0;
inline boolean stimulusLedControlPinIsOn() {return (PORTF & _BV(0));}


const int experimentAboutToStartIndicatorPin = PIN_E0;


const boolean debug = false;

volatile boolean verbose = true;

volatile int countsSinceLastFrame = 18750;
volatile boolean newFrame = false;
volatile boolean newByte = false;

int currentByte = 0;
int nextByte = 0;


const int tnt1prescale = 64;
int tnt3prescale = 1;
int tnt3prescaleByte = 1;

int nBitsPerFrame = 75; //number of bits output per frame
int frameBitCount = 0;
int bitCount = 0;
long totalBitCount = 0;

const int numFramesToPadExperimentStart = 10;
volatile int experimentStartCountdown = -1;
volatile boolean experimentRunning = false;

File sdfile;
const int MAXFILECHARS = 12;
char filename[MAXFILECHARS+1];

const int SERIALBUFFERSIZE = 64;
byte serialBuffer[SERIALBUFFERSIZE];

const int SERIAL_CHARS_TO_BUFFER = 255;

long int experimentStartTime = 0;
long int experimentElapsedFrames = -999;

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
    
  pinMode(experimentAboutToStartIndicatorPin, OUTPUT);
  digitalWrite(experimentAboutToStartIndicatorPin, LOW);
  
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
  
  
  //start SD library
  SD.begin(ssPin);
  
  

  setupTimer1();
  
  attachInterrupt(cameraFlashWindowPin, cameraFlash, CHANGE);
  
  digitalWrite(indicatorLedPin, LOW);
  
  wdt_enable(WDTO_8S); //reset every 8 seconds unless watchdog timer is reset

  for (int j = 0; j < 4; ++j) {
    wdt_reset();
    digitalWrite(indicatorLedPin, j%2);
    delay(500);
  }
  
  
}

ISR ( TIMER3_COMPA_vect ) {
  nextBit();
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
  digitalWrite(experimentAboutToStartIndicatorPin, LOW);
  if (experimentRunning) {
    ++experimentElapsedFrames;
  }
  if ((experimentStartCountdown--) > 0) {
    experimentElapsedFrames = -experimentStartCountdown;
    if (experimentRunning = (experimentStartCountdown == 0)) {
      experimentStartTime = millis();
      totalBitCount = 0;
    }
    digitalWrite(experimentAboutToStartIndicatorPin, !experimentRunning);
  }
  //experimentRunning = experimentReady; //this way the experiment always starts on a frame
  frameBitCount = 0;
  updateTimer3();
  nextBit();
}

void nextBit() {
  if (!experimentRunning || (++frameBitCount > nBitsPerFrame)) {
    return;
  } //keep us from putting too many bits into a frame if the camera is a little slow or the avr is a little fast
  bitCount = bitCount%8;
  if (bitCount == 0) {
    newByte = true;
    currentByte = nextByte;
  }
  if (currentByte < 0) { //error reading from file
    digitalWrite (stimulusLedControlPin, 0);
    return;
  }
  ++totalBitCount;
  digitalWrite (stimulusLedControlPin, currentByte & _BV(bitCount)); //digital write is "slow" ~55 microseconds 
}

void pollForNewByte () {
  if (!experimentRunning || !newByte || !sdfile) {
    return;
  }
  newByte = false;
  nextByte = sdfile.read();
}

void setNumBitsPerFrame (int nbits) {
  nBitsPerFrame = nbits;
  float targetPreScale = 1.0*tnt1prescale / nBitsPerFrame - 0.001;
  int prescalerValues[] = {1,8,64,256,1024};
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
      err = readBytesFromFileToSerial(command+1);
      if (verbose && !err) {
        Serial.print("OK: Tranfer from "); Serial.print(filename); Serial.println(" complete"); 
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
      err = setNumBitsPerFrameFromString(command+1); 
      if (verbose && !err) {
        Serial.print("OK: "); Serial.print (nBitsPerFrame); Serial.println(" bits per frame.");
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
  Serial.println(totalBitCount);
  
  
}

boolean setNumBitsPerFrameFromString (const char *str) {
  int nb = atoi(str);
  if (nb <= 0 || nb > 150) {
    return true;
  }
  setNumBitsPerFrame (nb);
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
  digitalWrite(stimulusLedControlPin, atoi(command));
  return false;
}

boolean readBytesFromFileToSerial (const char *command) {
  unsigned long nbytes = strtoul (command, NULL, 0);
  
  if (!sdfile) {
    for ( ; nbytes > 0; --nbytes) {
      wdt_reset();
      Serial.write((uint8_t) 0);
    }
      Serial.println("no file open");
      return true;
  }
  
  int n2w;
  boolean err = false;
  for ( ;nbytes > 0; --nbytes) {
    wdt_reset();
    int v =  sdfile.read(); 
    err |= (v < 0);  
    Serial.write(min(0, v));
  }
  if (err) {
    Serial.println ("insufficient bytes to read/read error");
  }
  return err;
}

boolean startExperiment() {
 if (!readyToStartExperiment()) {
  return true;
 }
 experimentStartCountdown = numFramesToPadExperimentStart;
 enableBitInterrupt();
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
  currentByte = 0;
  nextByte = sdfile.read();
  return (nextByte >= 0);
}

void reportReady() {
  if (readyToStartExperiment()) {
    Serial.println("READY");
    Serial.println(filename);
    Serial.print(sdfile.size()); Serial.println(" bytes");
    Serial.print("bits per frame = "); Serial.println(nBitsPerFrame);
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
  disableBitInterrupt();
  experimentStartCountdown = -1;
  experimentRunning = false;
  experimentElapsedFrames = -999;
  sdfile.close();
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
    if (prescalerValues[ps] >= targetPreScale) {
       break;
    }
  } 
  tnt3prescale = prescalerValues[ps];
  tnt3prescaleByte = prescalerBytes[ps];
  unsigned char sreg = SREG;
  noInterrupts();
  TCCR3A = 0;
  TCCR3C = 0;
  SREG = sreg;
}

void updateTimer3() {
  int timer3countsPerBit = (int) (1.0*countsSinceLastFrame*(tnt1prescale/tnt3prescale)/nBitsPerFrame + 0.5); 
  unsigned char sreg = SREG;
  noInterrupts();
  TCCR3B = _BV(WGM32); //set to CTC mode; stop counter
  TCNT3 = 0;
  OCR3A = timer3countsPerBit;  
  TCCR3B |= tnt3prescaleByte; //start clock  
  SREG = sreg; //reenables interrupts if they were previously enabled (I think)
}

void enableBitInterrupt() {
  TIMSK3 |= _BV(OCIE3A ); // Enable CTC interrupt
  interrupts(); // make sure global interrupts are on
}
void disableBitInterrupt() {
  TIMSK3 &= ~_BV(OCIE3A ); // disable CTC interrupt
}








