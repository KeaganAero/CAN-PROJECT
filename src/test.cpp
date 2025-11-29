// #include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

const uint8_t HALL_PIN = 2;
const uint8_t MCP2515_CS = 10;
const uint8_t MCP2515_INT = 3;
const unsigned long RPM_SAMPLE_MS = 500UL;
const uint8_t PULSES_PER_REV = 2;
const uint8_t LED_PIN = 4;
bool LED_PIN_STATE = HIGH;
unsigned long lastToggle = 0;
const unsigned long interval = 500;
const uint16_t CAN_ID_RPM = 0x100;
const uint8_t BUTTON_PIN = 5;

// ---------------- MOTOR DRIVER PINS ----------------
const uint8_t IN1 = 8; // Connect IN1 on motor driver to Arduino pin 8
const uint8_t IN2 = 9; // Connect IN2 on motor driver to Arduino pin 9
bool MOTOR_ENABLED = LOW;

// MOTOR STATE VARIABLES
bool motorRunning = true;
bool lastButtonState = HIGH;
MCP_CAN CAN(MCP2515_CS);

volatile unsigned long pulseCount = 0;    // Hall ISR counter
volatile bool canMessageReceived = false; // new flag for CAN ISR
unsigned long lastSampleTime;
unsigned long lastPulseCountSnapshot;

// Hall sensor ISR
void hallISR()
{
    pulseCount++;
}

// MCP2515 ISR: just set a flag
void MCP_ISR()
{
    canMessageReceived = true; // minimal ISR
}

void setup()
{

    Serial.begin(115200);
    while (!Serial)
    {
    }
    Serial.println(F("\n--- RPM over CAN: Starting ---"));
    // check button pin functionality
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    // Pin 4 to check for power status
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_PIN_STATE ? HIGH : LOW); // Turn on LED to indicate power is on

    // Hall sensor setup
    pinMode(HALL_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);
    Serial.println(F("Hall sensor interrupt attached on D2 (FALLING)."));

    // CAN init
    SPI.begin();
    Serial.print(F("Initializing MCP2515... "));
    if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    {
        Serial.println(F("MCP2515 Init OK"));
    }
    else
    {
        Serial.println(F("MCP2515 Init FAIL"));
    }
    CAN.setMode(MCP_NORMAL);
    delay(10);

    // Attach interrupt for MCP2515 INT pin
    pinMode(MCP2515_INT, INPUT);
    attachInterrupt(digitalPinToInterrupt(MCP2515_INT), MCP_ISR, FALLING);
    Serial.println(F("MCP2515 interrupt attached on D3"));

    // ------------- MOTOR DRIVER SETUP ----------------
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    // Spin motor forward at full speed
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    // ENA should be HIGH or jumper on board

    Serial.println(F("Motor driver initialized, motor should spin forward."));
    // ------------------------------------------------
    Serial.print("IN1 state (should be HIGH): ");
    Serial.println(digitalRead(IN1));
    Serial.print("IN2 state (should be LOW): ");
    Serial.println(digitalRead(IN2));
    Serial.println(F("Motor driver initialized, motor should spin forward."));
    // initial snapshots
    lastSampleTime = millis();
    lastPulseCountSnapshot = 0;
}

void loop()
{

    unsigned long now = millis();

    if (now - lastToggle >= interval)
    {
        lastToggle = now;
        LED_PIN_STATE = !LED_PIN_STATE;
        digitalWrite(LED_PIN, LED_PIN_STATE ? HIGH : LOW);
    }
    ////////////-------------------------------------------- BUTTON TEST --------------------------------------------////////////////

    // I now have a state variable called motor running, which is true when the motor is running and false when stopped, I will make the button a toggle switch to start and stop the motor. I will read the button state and if it is pressed and motor running is true, I will stop the motor and set motor running to false. If the button is pressed and motor running is false, I will start the motor and set motor running to true.

    bool buttonState = digitalRead(BUTTON_PIN);

    // detect button press (HIGH -> LOW)
    if (buttonState == LOW && lastButtonState == HIGH)
    {
        // toggle motor
        motorRunning = !motorRunning;

        if (motorRunning)
        {
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            Serial.println("Motor ON");
        }
        else
        {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, LOW);
            Serial.println("Motor OFF");
        }
    }

    lastButtonState = buttonState;

    // Non-blocking RPM computation
    if (now - lastSampleTime >= RPM_SAMPLE_MS)
    {
        noInterrupts();
        unsigned long pulses = pulseCount;
        pulseCount = 0;
        interrupts();

        float seconds = RPM_SAMPLE_MS / 1000.0f;
        float revsPerSecond = (float)pulses / (seconds * PULSES_PER_REV);
        float rpm = revsPerSecond * 60.0f;

        int32_t rpm_fixed = (int32_t)round(rpm * 100.0f);
        uint8_t payload[4];
        payload[0] = (uint8_t)((rpm_fixed >> 24) & 0xFF);
        payload[1] = (uint8_t)((rpm_fixed >> 16) & 0xFF);
        payload[2] = (uint8_t)((rpm_fixed >> 8) & 0xFF);
        payload[3] = (uint8_t)(rpm_fixed & 0xFF);

        byte sendResult = CAN.sendMsgBuf(CAN_ID_RPM, 0, 4, payload);
        if (sendResult == CAN_OK)
        {
            Serial.print(F("Sent CAN RPM: "));
            Serial.print(rpm, 2);
            Serial.print(F(" RPM (fixed="));
            Serial.print(rpm_fixed);
            Serial.println(F(")"));
        }
        else
        {
            Serial.print(F("CAN send fail, code "));
            Serial.println(sendResult, DEC);
        }

        Serial.print(F("Measured pulses: "));
        Serial.print(pulses);
        Serial.print(F(" in "));
        Serial.print(RPM_SAMPLE_MS);
        Serial.print(F(" ms -> RPM = "));
        Serial.print(rpm, 2);
        Serial.println();

        lastSampleTime = now;
    }

    // --- NEW: handle incoming CAN using INT pin ---
    if (canMessageReceived)
    {
        canMessageReceived = false; // reset flag

        long unsigned int rxId;
        unsigned char len = 0;
        unsigned char rxBuf[8];
        if (CAN.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK)
        {
            Serial.print(F("CAN RX ID: 0x"));
            Serial.print(rxId, HEX);
            Serial.print(F(" len: "));
            Serial.println(len);
            for (uint8_t i = 0; i < len; i++)
            {
                Serial.print(rxBuf[i], HEX);
                Serial.print(' ');
            }
            Serial.println();
        }
    }

    // other non-blocking tasks can run here

    // --- DEBUG LOG ---
    // Serial.print("Loop check: IN1=");
    // Serial.print(digitalRead(IN1));
    // Serial.print(", IN2=");
    // Serial.println(digitalRead(IN2));

    //  delay(1000); // 1 second delay to avoid spamming Serial
}
