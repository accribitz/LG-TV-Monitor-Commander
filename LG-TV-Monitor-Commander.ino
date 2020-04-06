/*
 * 
 * LG TV/Monitor Commander
 * 
 * Control LG TV/Monitor using Arduino's serial port or SoftSerial.
 * 
 * Copyright 2020 accribitz
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * This software is released under the GPLv3 License, see LICENSE.
 * 
 */

#include <EEPROM.h>

/*
 * If target board does NOT have SERIAL_PORT_MONITOR, 
 * this program use SoftwareSerial to control EEPROM.
 * 
 * You need to set following CONTROL_RX_PIN and CONTROL_TX_PIN pins.
 * CONTROL_RX_PIN and CONTROL_TX_PIN pins can NOT use input pins.
 * See limitations of SoftwareSerial library.
 * https://www.arduino.cc/en/Reference/SoftwareSerial
*/
#ifndef SERIAL_PORT_MONITOR

  #define CONTROL_RX_PIN 14
  #define CONTROL_TX_PIN 15
  #define CONTROL_PORT SoftSerial_Control

  #include <SoftwareSerial.h>
  SoftwareSerial CONTROL_PORT( CONTROL_RX_PIN, CONTROL_TX_PIN );
#else
  #define CONTROL_PORT SERIAL_PORT_MONITOR
#endif

/*
 * If target board does NOT have SERIAL_PORT_HARDWARE_OPEN, 
 * this program use SoftwareSerial to control LG device.
 * 
 * You need to set following LG_RX_PIN and LG_TX_PIN pins.
 * LG_RX_PIN and LG_TX_PIN pins can NOT use input pins.
 * See limitations of SoftwareSerial library.
 * https://www.arduino.cc/en/Reference/SoftwareSerial
*/
#ifndef SERIAL_PORT_HARDWARE_OPEN
  #define LG_RX_PIN 10
  #define LG_TX_PIN 11
  #define LG_PORT SoftSerial_LG

  #include <SoftwareSerial.h>
  SoftwareSerial LG_PORT( LG_RX_PIN, LG_TX_PIN );
#else
  #define LG_PORT SERIAL_PORT_HARDWARE_OPEN
#endif

// constants won't change. They're used here to set:
const unsigned short int  DATA_LENGTH     = 8;    // data length (8 bits)
const unsigned long       SERIAL_BOUDRATE = 9600; // baud rate 9600 bps (UART)
const unsigned short int  DEBOUNCE_DELAY  = 50;   // debounce time; increase if the output flickers
const unsigned short int  SERIAL_DELAY    = 50;   // delay for listing LG command ack 

// Variables will change:
unsigned short int maxButtons;                          // the maximam buttons smaller of NUM_DIGITAL_PINS or EEPROM size
String commandInputBuffer;                              // input buffer of control command input
bool usedPins[ NUM_DIGITAL_PINS ];                      // pins are used for input(true) or not(false)
unsigned short int buttonState[ NUM_DIGITAL_PINS ];     // the current reading from the input pin
unsigned short int lastButtonState[ NUM_DIGITAL_PINS ]; // the previous reading from the input pin
unsigned long lastDebounceTime[ NUM_DIGITAL_PINS ];     // the previous reading from the input pin


void setup() {
  LG_PORT.begin( SERIAL_BOUDRATE );
  CONTROL_PORT.begin( SERIAL_BOUDRATE );

  showSpec();
  showEeprom();

  for ( unsigned short int thisPin = 0; thisPin < maxButtons; thisPin++ ) {
    buttonState[ thisPin ] = HIGH;
    lastButtonState[ thisPin ] = HIGH;
  }

  setInputPin();
  commandInputBuffer = "";

  showHelp();
}

void loop() {
  signed char incomingChar;
  boolean isCommand = false;

  /*
   * Wait Control Serial port input.
   */
  if( -1 != ( incomingChar = CONTROL_PORT.read() )) {
    // buffering input less than ( DATA_LENGHT * 2 ) chars
    commandInputBuffer += char( incomingChar );
    if ( '\r' == incomingChar || '\n' == incomingChar || commandInputBuffer.length() > DATA_LENGTH * 2 ) {
      commandInputBuffer.trim();

      // Show board spec
      if ( ! commandInputBuffer.compareTo( String(F("spec")) )) {
        showSpec();
        isCommand = true;
      }

      // Show EEPROM
      if ( ! commandInputBuffer.compareTo( String(F("show")) )) {
        showEeprom();
        isCommand = true;
      }

      // Clear EEPROM
      if ( ! commandInputBuffer.compareTo( String(F("clear")) )) {
        clearEeprom();
        isCommand = true;
      }

      // Export EEPROM
      if ( ! commandInputBuffer.compareTo( String(F("export")) )) {
        exportEeprom();
        isCommand = true;
      }

      // Set Control Code Data to EEPROM
      if ( ! commandInputBuffer.substring( 0, commandInputBuffer.indexOf( ' ' )).compareTo( String(F("save")) )) {
        // Get Command
        commandInputBuffer = commandInputBuffer.substring( commandInputBuffer.indexOf( ' ' ), commandInputBuffer.length() );
        commandInputBuffer.trim();

        saveCommand( commandInputBuffer );
        isCommand = true;
      }

      // Send Control Code from EEPROM Data
      if ( ! commandInputBuffer.substring( 0, commandInputBuffer.indexOf( ' ' )).compareTo( String(F("send")) )) {
        // Get Command
        commandInputBuffer = commandInputBuffer.substring( commandInputBuffer.indexOf( ' ' ), commandInputBuffer.length() );
        commandInputBuffer.trim();

        sendPinsCommand( commandInputBuffer );
        isCommand = true;
      }

      // Pass Though Input
      if ( ! commandInputBuffer.substring( 0, commandInputBuffer.indexOf( ' ' )).compareTo( String(F("pass")) )) {
        // Get Target Raw Data
        commandInputBuffer = commandInputBuffer.substring( commandInputBuffer.indexOf( ' ' ), commandInputBuffer.length() );
        commandInputBuffer.trim();

        sendDataString( commandInputBuffer );
        isCommand = true;
      }

      // Show Help
      if ( ! isCommand && 0 != commandInputBuffer.length() ) {
        showHelp();
      }

      commandInputBuffer = "";
    }
  }

  /*
   *  This loop check button stats.
   */
  for ( unsigned short int thisPin = 0; thisPin < maxButtons; thisPin++ ){
    if ( usedPins[ thisPin ] && ( LOW == digitalRead( thisPin ) )) {
      if ( LOW != lastButtonState[ thisPin ] ) {
        lastDebounceTime[ thisPin ] = millis();
        lastButtonState[ thisPin ] = LOW;
      }
      if (( LOW != buttonState[ thisPin ] ) && ( millis() - lastDebounceTime[ thisPin ] ) > DEBOUNCE_DELAY ) {
        sendSavedData( thisPin );
        buttonState[ thisPin ] = LOW;
      }
    } else {
      lastButtonState[ thisPin ] = HIGH;
      buttonState[ thisPin ] = HIGH;
    }
  }
}

void showHelp () {
  CONTROL_PORT.print( F("Commands : Description\n") );
  CONTROL_PORT.print( F("    spec : Show board specifications.\n") );
  CONTROL_PORT.print( F("    show : Show saved commands.\n") );
  CONTROL_PORT.print( F("    save : Save command.\n") );
  CONTROL_PORT.print( F("           save[ ][PIN_NUMBER][ ][Command1][Command2][ ][Set ID][ ][Data]\n") );
  CONTROL_PORT.print( F("           Example 1 : Save power off command to PIN 1 EEPROM. \"save 2 ka 00 00\"\n") );
  CONTROL_PORT.print( F("           Example 2 : Clear PIN 1 EEPROM data. \"save 2\"\n") );
  CONTROL_PORT.print( F("    send : Send saved command.\n") );
  CONTROL_PORT.print( F("           send[ ][PIN_NUMBER]\n") );
  CONTROL_PORT.print( F("           Example   : Send PIN 2 command. \"send 2\"\n") );
  CONTROL_PORT.print( F("    pass : Pass through input.\n") );
  CONTROL_PORT.print( F("           pass[ ][Command1][Command2][ ][Set ID][ ][Data]\n") );
  CONTROL_PORT.print( F("           Example   : Send power off command. \"pass ka 00 00\"\n") );
  CONTROL_PORT.print( F("   clear : Clear all saved commands.\n") );
  CONTROL_PORT.print( F("  export : Show saved control commands for copy & paste.\n\n") );
}

/*
 * Calculate maxButtons and show board specification.
 */
void showSpec () {
  unsigned short int eepromLength = EEPROM.length();

  CONTROL_PORT.print( F("EEPROM SIZE         : ") );
  CONTROL_PORT.print( eepromLength, DEC );
  CONTROL_PORT.print( F(" Bytes\n") );
  CONTROL_PORT.print( F("EEPROM SIZE / ") );
  CONTROL_PORT.print( DATA_LENGTH, DEC );
  CONTROL_PORT.print( F("  (A): ") );
  CONTROL_PORT.print( eepromLength / DATA_LENGTH, DEC );
  CONTROL_PORT.print( F(" (Limitation of EEPROM size.)\n") );
  CONTROL_PORT.print( F("NUM_DIGITAL_PINS (B): ") );
  CONTROL_PORT.print( NUM_DIGITAL_PINS, DEC );
  CONTROL_PORT.print( F(" (Limitation of digital input PINs. Defined \"pins_arduino.h\".)\n") );
  
  CONTROL_PORT.print( F("MAX INPUT BUTTONS   : ") );
  if ( eepromLength / DATA_LENGTH <= NUM_DIGITAL_PINS ) {
    CONTROL_PORT.print( eepromLength / DATA_LENGTH, DEC );
    maxButtons = eepromLength / DATA_LENGTH;
  } else {
    CONTROL_PORT.print( NUM_DIGITAL_PINS, DEC ); 
    maxButtons = NUM_DIGITAL_PINS;
  }
  CONTROL_PORT.print( F(" (Smaller of (A) or (B).)\n\n") );
}

/*
 * Check char code is alphabet, numbe or space.
 */
bool isCommandCode ( char code ) {
  if ( isAlphaNumeric( code ) || ' ' == code ) {
    return true;
  }
  return false;
}

/*
 * Send data string to LG_PORT. 
 * Receive ACK from LG_PORT.
 */
void sendDataString ( String data ) {
  String ack = "";
  data += F("\r");

  // If using SoftSerial, change listen port to read ACK.
  #ifndef SERIAL_PORT_HARDWARE_OPEN
    LG_PORT.listen();
  #endif

  while ( LG_PORT.available() > 0 ) {
    ack = LG_PORT.read();
  }

  LG_PORT.println( data );

  ack = "";
  delay( SERIAL_DELAY );
  while ( LG_PORT.available() > 0 ) {
    ack += char( LG_PORT.read() );
  }

  CONTROL_PORT.print( F("Sent : ") );
  CONTROL_PORT.println( data );
  CONTROL_PORT.print( F("ACK  : ") );
  CONTROL_PORT.println( ack );
  
  // If using SoftSerial, change listen port to read user input.
  #ifndef SERIAL_PORT_MONITOR
    CONTROL_PORT.listen();
  #endif
}

void sendSavedData ( unsigned short int thisPin ) {
  String dataBuffer = "";
  char commandCode;

  for ( unsigned short int commandIndex = 0; commandIndex < DATA_LENGTH; commandIndex++ ) {
    commandCode = EEPROM[ thisPin * DATA_LENGTH + commandIndex ];
    if ( isCommandCode( commandCode ) ) {
       dataBuffer += commandCode;
    }
  }

  sendDataString( dataBuffer );
}

/*
 * Show formatted style EEPROM data.
 */
void exportEeprom () {
  char commandCode;

  CONTROL_PORT.print( F("Export EEPROM...\n") );
  CONTROL_PORT.print( F("--------\n") );
  for ( unsigned short int thisPin = 0; thisPin < maxButtons; thisPin++ ){
    if ( usedPins[ thisPin ] ) {
      CONTROL_PORT.print( F("set ") );
      CONTROL_PORT.print( thisPin, DEC );
      CONTROL_PORT.print( ' ' );
      for ( unsigned short int commandIndex = 0; commandIndex < DATA_LENGTH; commandIndex++ ){
        commandCode = EEPROM[ thisPin * DATA_LENGTH + commandIndex ];
        if ( isCommandCode( commandCode ) ) {
          CONTROL_PORT.print( commandCode );
        }
      }
      CONTROL_PORT.print( F("\n") );
    }
  }
  CONTROL_PORT.print( F("--------\n") );
  CONTROL_PORT.print( F("Done.\n\n") );
}

/*
 * Write EEPROM 
 * Start address = PIN_NUMBER * DATA_LENGTH
 */
void writeEeprom ( unsigned short int thisPin, char commandCodes[ DATA_LENGTH ] ) {
  for ( unsigned short int commandIndex = 0; commandIndex < DATA_LENGTH; commandIndex++ ) {
      EEPROM.update( thisPin * DATA_LENGTH + commandIndex, commandCodes[ commandIndex ] );
  }
}

/*
 * Show saved EEPROM data.
 */
void showEeprom () {
  unsigned short int i, j;
  char code;
  
  CONTROL_PORT.print( F("PIN:[Command1][Command2][ ][Set ID][ ][Data](\"_\" is null.)\n") );

  for ( i = 0; i < maxButtons ; i++ ) {
    usedPins[ i ] = false;

    if ( i < 10 ) {
      CONTROL_PORT.print( ' ' );
    }
    CONTROL_PORT.print( ' ' );
    CONTROL_PORT.print( i, DEC );
    CONTROL_PORT.print( ':' );
    
    for ( j = 0; j < DATA_LENGTH; j++ ) {
      code = EEPROM[ i * DATA_LENGTH + j ];
      if ( isCommandCode( code ) ) {
        CONTROL_PORT.print( code );
        usedPins[ i ] = true;
      } else {
        CONTROL_PORT.print( '_' );
      }
    }
    CONTROL_PORT.print( '\n' );
  }
  CONTROL_PORT.print( '\n' );
}

/*
 * Fill all usable EEPROM area char(255).
 */
void clearEeprom () {
  char code[ DATA_LENGTH ];
  unsigned short int i;

  for ( i = 0; i < DATA_LENGTH; i++ ) {
    code[ i ] = 255;
  }

  exportEeprom();

  CONTROL_PORT.print( F("Clear EEPROM ( Fill char(255) ) ") );
  for ( i = 0; i < maxButtons; i++ ) {
    writeEeprom( i, code );
    CONTROL_PORT.print( '.' );
  }
  CONTROL_PORT.print( F(" Done.\n\n") );

  showEeprom();
  setInputPin();
}

/*
 * Fetch usedPins[] and set pinMode "INPUT_PULLUP".
 */
void setInputPin () {
  unsigned short int i;

  CONTROL_PORT.print( F("Set enabled PINs' pinMode INPUT_PULLUP ") );
  for( i = 0; i < maxButtons; i++ ) {
    if ( usedPins[i] ) {
      pinMode( i, INPUT_PULLUP );
      CONTROL_PORT.print( '.' );
    }
  }
  CONTROL_PORT.print( F(" Done.\n\n") );
}

/*
 * Save control command to target PIN's EEPROM.
 */
void saveCommand( String command ) {
  short int i, pin;
  char data[ DATA_LENGTH + 1 ];
  String temp;

  // Get pin number. 
  temp = command.substring( 0, command.indexOf( ' ' ) );
  temp.trim();
  pin = convertPinNumber( temp );

  if ( pin < 0 || maxButtons <= pin ) {
    CONTROL_PORT.print( F("PIN is not defined.\n\n") );

    return;
  }

  // Trim command
  temp = command.substring( command.indexOf( ' ' ), command.length() );
  temp.trim();

  // Too long command
  if ( DATA_LENGTH < temp.length() ) {
    CONTROL_PORT.print( F("Too long command \"") );
    CONTROL_PORT.print( temp );
    CONTROL_PORT.print( F("\"\n\n") );

    return;
  }

  // Initialize data[]
  for ( i = 0; i < DATA_LENGTH + 1; i++ ) {
    data[ i ] = 255;
  }

  if ( 0 == temp.length() ) {
    CONTROL_PORT.print( F("Clear PIN ") );
    CONTROL_PORT.print( pin, DEC );
    CONTROL_PORT.print( F(".\n\n") );
  } else {
    temp.toCharArray( data, i + 1 );

    CONTROL_PORT.print( F("Save PIN ") );
    CONTROL_PORT.print( pin );
    CONTROL_PORT.print( F(" : \"") );

    for ( i = 0; i < DATA_LENGTH + 1; i++ ) {
      if ( isCommandCode( data[ i ] ) ) {
        CONTROL_PORT.print( data[ i ] );
      } else {
        data[ i ] = 255;
      }
    }
    
    CONTROL_PORT.print( F("\"\n\n") );
  }

  writeEeprom( pin, data );

  showEeprom();
  setInputPin();
}

/*
 * Send the command saved on target PIN's.
 */
void sendPinsCommand( String pinString ) {
  short int pinNumber;

  pinNumber = convertPinNumber( pinString );

  if ( 0 <= pinNumber && pinNumber < maxButtons && usedPins[ pinNumber ] ) {
    sendSavedData( pinNumber );
  }

  return;
}

/*
 * Check pinString and convert to integer.
 */
short int convertPinNumber( String pinString ) {
  unsigned short int i;

  if ( pinString.length() == 0 ) {
    return -1;
  }

  for ( i = 0; i < pinString.length(); i++ ) {
    if ( ! isDigit( pinString[ i ] )) {
      return -1;
    }
  }

  return pinString.toInt();
}
