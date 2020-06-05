#include <loraWan.h>

// ------------------- Define some constants for convenience -----------------
#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3

#define netId 0x0000 // Network ID for transmitter device. 0x0000 by default

loraWan lora(2,3);

// Previous state of each channel (HIGH or LOW)
volatile byte previous_state[4];

int channelSelector = 0; // Select destination lora
int prevChannel = 0;

int channels[] = {0x0001, 0x0002, 0x0003}; // List of addresses of destination nodes.

// Duration of the pulse on each channel of the receiver in µs (must be within 1000µs & 2000µs)
volatile unsigned int pulse_length[4] = {1500, 1500, 1000, 1500};
int pulse_len_default[] = {1500, 1500, 1000, 1500};

// Used to calculate pulse duration on each channel
volatile unsigned long current_time;
volatile unsigned long timer[4]; // Timer of each channel


// Routine for setting default values to maintain hovering state.
void sendData_Default()
{
    Serial.print('<');
    Serial.print(pulse_len_default[0]);
    Serial.print(pulse_len_default[1]);
    Serial.print(pulse_len_default[2]);
    Serial.print(pulse_len_default[3]);
}

void sendData()
{
    Serial.print('<');
    Serial.print(pulse_length[0]);
    Serial.print(pulse_length[1]);
    Serial.print(pulse_length[2]);
    Serial.print(pulse_length[3]);
}

void selectDest()
{
    int DestVal = analogRead(A0);
    prevChannel = channelSelector;
    if (DestVal >= 800)
    {
        channelSelector = 2;
    }
    else if (DestVal >= 255)
    {
        channelSelector = 1;
    }
    else
    {
        channelSelector = 0;
    }
    if (prevChannel != channelSelector)
    { // Before changing control from current destination to another destination node, set the control values to some defualt values so as to maintain position.
        sendData_Default();
    }
    lora.setDestination(channels[channelSelector]);
}

void setup()
{

    // Configure interrupts for receiver
    PCICR |= (1 << PCIE0);   // Set PCIE0 to enable PCMSK0 scan
    PCMSK0 |= (1 << PCINT0); // Set PCINT0 (digital input 8) to trigger an interrupt on state change
    PCMSK0 |= (1 << PCINT1); // Set PCINT1 (digital input 9) to trigger an interrupt on state change
    PCMSK0 |= (1 << PCINT2); // Set PCINT2 (digital input 10)to trigger an interrupt on state change
    PCMSK0 |= (1 << PCINT3); // Set PCINT3 (digital input 11)to trigger an interrupt on state change

    lora.setGateway(); // Sets current lora as the transmitter.

    Serial.begin(115200);
}

void loop()
{

    selectDest(); // Set the destination address by measuring the pwm value on the additional channels available. The range of values is used to assign the chanel/ Destination address/node.
    sendData();   // Sent the pulse lengths measured by the ISR to the specific copter via the lora module.
}

/**
 * This Interrupt Sub Routine is called each time input 8, 9, 10 or 11 changed state.
 * Read the receiver signals in order to get flight instructions.
 *
 * This routine must be as fast as possible to prevent main program to be messed up.
 * The trick here is to use port registers to read pin state.
 * Doing (PINB & B00000001) is the same as digitalRead(8) with the advantage of using less CPU loops.
 * It is less convenient but more efficient, which is the most important here.
 *
 * @see https://www.arduino.cc/en/Reference/PortManipulation
 * @see https://www.firediy.fr/article/utiliser-sa-radiocommande-avec-un-arduino-drone-ch-6
*/
ISR(PCINT0_vect)
{
    current_time = micros();

    // Channel 1 -------------------------------------------------
    if (PINB & B00000001)
    { // Is input 8 high ?
        if (previous_state[CHANNEL1] == LOW)
        {                                    // Input 8 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL1] = HIGH; // Save current state
            timer[CHANNEL1] = current_time;  // Save current time
        }
    }
    else if (previous_state[CHANNEL1] == HIGH)
    {                                                            // Input 8 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL1] = LOW;                          // Save current state
        pulse_length[CHANNEL1] = current_time - timer[CHANNEL1]; // Calculate pulse duration & save it
    }

    // Channel 2 -------------------------------------------------
    if (PINB & B00000010)
    { // Is input 9 high ?
        if (previous_state[CHANNEL2] == LOW)
        {                                    // Input 9 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL2] = HIGH; // Save current state
            timer[CHANNEL2] = current_time;  // Save current time
        }
    }
    else if (previous_state[CHANNEL2] == HIGH)
    {                                                            // Input 9 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL2] = LOW;                          // Save current state
        pulse_length[CHANNEL2] = current_time - timer[CHANNEL2]; // Calculate pulse duration & save it
    }

    // Channel 3 -------------------------------------------------
    if (PINB & B00000100)
    { // Is input 10 high ?
        if (previous_state[CHANNEL3] == LOW)
        {                                    // Input 10 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL3] = HIGH; // Save current state
            timer[CHANNEL3] = current_time;  // Save current time
        }
    }
    else if (previous_state[CHANNEL3] == HIGH)
    {                                                            // Input 10 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL3] = LOW;                          // Save current state
        pulse_length[CHANNEL3] = current_time - timer[CHANNEL3]; // Calculate pulse duration & save it
    }

    // Channel 4 -------------------------------------------------
    if (PINB & B00001000)
    { // Is input 11 high ?
        if (previous_state[CHANNEL4] == LOW)
        {                                    // Input 11 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL4] = HIGH; // Save current state
            timer[CHANNEL4] = current_time;  // Save current time
        }
    }
    else if (previous_state[CHANNEL4] == HIGH)
    {                                                            // Input 11 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL4] = LOW;                          // Save current state
        pulse_length[CHANNEL4] = current_time - timer[CHANNEL4]; // Calculate pulse duration & save it
    }
}
