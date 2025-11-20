#include <Arduino.h> //All library imports
#include <SPI.h>
#include <mcp_can.h>

const uint8_t HALL_PIN = 2;                // Hall pin on pin 2
const uint8_t MCP2515_CS = 10;             // THE MCP2515 chip select wire on pin 10
const uint8_t MCP2515_INT = 3;             // The MCP2515 Interrupt wire to pin 3
const unsigned long RPM_SAMPLE_MS = 500UL; // long is used for timers because a width of 32 bits is sufficient
const uint8_t PULSES_PER_REV = 1;          // number of pulses per revolution from the hall sensor, hardware dependent
MCP_CAN CAN(MCP2515_CS);                   // mcp_can is the library class, where CAN is the object we created and named it CAN, the argument is simply what we defined early and is an address so the library knows where to find the MCP2515.(on pin 10)

volatile unsigned long pulseCount = 0;    // this counts the number pulses from the hall sensor during the sample period
volatile bool canMessageReceived = false; // flag to indicate  CAN message received, initially fasle and changes to true when interrupt occurs ie. a can message is received.

// Hall Sensor  ISR    //used for fast non blocking counting of pulses from the hall sensor

void hallISR()
{ // included IRAM_ATTR to ensure the ISR is placed in IRAM for faster execution on certain platforms. This ensures it runs on fast internal ram vs flash memory. Arduino UNO its not necessary and the ISR would work fine without it.
    pulseCount++;
}

void MCP_ISR() // CAN ISR used to indicate that a CAN message has been received.
{
    canMessageReceived = true;
}

void setup()
{                         // setup function running once at the beginning of the code to initialize all functions within once.
    Serial.begin(115200); // starts the serial monitor at a specified baude rate which must match the cpu's baud rate.
    while (!Serial)
    {
    }                                                      // while there is no data nothing happens
    Serial.println(F("\n--- RPM over CAN: Starting ---")); // the first output message on the serial monitor to show once data comes in
}

// Hall Sensor setup
pinMode(HALL_PIN, INPUT_PULLUP);                                      // reacts to the change from the pull up resistor, when the sensor detects a magnet it connects the pin to ground causing a falling edge case, HIGH TO LOW
attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);   // attachInterrupt attaches a pin to an interrupt ,digitalPinToInterrupt converts a pin number to an isr number internally, in this case it is HALL_PIN, when this happens the hallISR is triggered,and FALLING specifies the case in which it does so, from HIGH TO LOW only because we configured the pin using input_pullup.
Serial.println(F("Hall sensor interrupt attached on D2 (FALLING).")); // prints a notification that the hall_isr has executed successfully.

// CAN init
SPI.begin();                                  // Begins the spi communication with CAN
Serial.println(F("Initializing MCP2515...")); // print line after successfully Initializing.
if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
{                                         // if MCP2515 begins successfully we give it three configurations, MCP_ANY which tells the MCP2515 to listen all incoming can IDS, CAN_500KBPS sets the CAN bit rate which should be the same on all attached CAN devices and the MCP_8MHZ, is the oscilator frequency used to calculate correcting timing between CAN messages.
    Serial.println(F("MCP2515 Init OK")); // print this line if MCP2515 begins successfully.
}
else
{
    Serial.println(F("MCP2515 init FAIL")); // print this line if MCP2515 fails
}
CAN.setMode(MCP_NORMAL); // sets MCP2515 into normal operation mode so it can send and receive can FRAMES on the bus.
delay(10);               // Once MCP2515 begins it gives it a tiny pause to adjust before sending and receiving messages.Also this delay is in the code but it not considered blocking as it only runs once during the setup and not in the main loop.
