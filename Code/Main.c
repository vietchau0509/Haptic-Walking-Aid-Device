//Emb1 - Project -Chau Nguyen 1001764978
// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "clock.h"
#include "wait.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"
#include "rgb_led.h"

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))

#define NUMBER_OF_EVENTS 20

//CH0
#define TRIGGER_PIN_MASK_CH0 32 // Trigger pin is connected to PC5
#define ECHO_PIN_MASK_CH0 16 // Echo pin is connected to PC4 (FREQ_IN_MASK)


//CH1
#define TRIGGER_PIN_MASK_CH1 128 // Trigger pin is connected to PC7
#define ECHO_PIN_MASK_CH1 64 // Echo pin is connected to PC6 (FREQ_IN_MASK)

//CH2
#define TRIGGER_PIN_MASK_CH2 2 // Trigger pin is connected to PD0
#define ECHO_PIN_MASK_CH2 1 // Echo pin is connected to PD1 (FREQ_IN_MASK)


// PortC masks
#define FREQ_IN_MASK 64

// PortF masks
#define BLUE_LED_MASK 4
#define GREEN_LED_MASK 8
#define PUSH_BUTTON_MASK 16


bool timeMode = false;
uint32_t frequency = 0;
uint32_t time = 0;
uint32_t time1 = 0;
uint32_t time2 = 0;
uint32_t time3 = 0;

#define MAX_CHARS 80
#define MAX_FIELDS 6

typedef struct USER_DATA
{
    char buffer[MAX_CHARS + 1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
}
USER_DATA;



typedef struct EVENT_DATA
{
    uint8_t sensor;
    uint32_t minDistMm;
    uint32_t maxDistMm;
    uint8_t  haptic;
    uint8_t pwm;
    uint8_t beat;
    uint32_t ontime, offtime;

} EVENT_DATA;

EVENT_DATA events[NUMBER_OF_EVENTS];
//********//////

void enableCounterMode()
{
    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 40000000;                       // set load value to 40e6 for 1 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)

    // Configure Wide Timer 1 as counter of external events on CCP0 pin
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER1_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER1_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge count mode, count up
    WTIMER1_CTL_R = TIMER_CTL_TAEVENT_POS;           // count positive edges
    WTIMER1_IMR_R = 0;                               // turn-off interrupts
    WTIMER1_TAV_R = 0;                               // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
}

void disableCounterMode()
{
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off time base timer
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off event counter
    NVIC_DIS0_R = 1 << (INT_TIMER1A-16);            // turn-off interrupt 37 (TIMER1A)
}



void enableTimerMode(uint8_t channel)
{
    switch(channel)
    {
        case 0:
            //configuration for channel 0 (WTIMER1)
            WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;
            WTIMER1_CFG_R = 4;
            WTIMER1_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
            WTIMER1_CTL_R = TIMER_CTL_TAEVENT_POS;
            WTIMER1_IMR_R = TIMER_IMR_CAEIM;
            WTIMER1_TAV_R = 0;
            WTIMER1_CTL_R |= TIMER_CTL_TAEN;
            NVIC_EN3_R = 1 << (INT_WTIMER1A-16-96);
            break;

        case 1:
            //configuration for channel 1 (WTIMER5)
            WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;
            WTIMER5_CFG_R = 4;
            WTIMER5_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
            WTIMER5_CTL_R = TIMER_CTL_TAEVENT_POS;
            WTIMER5_IMR_R = TIMER_IMR_CAEIM;
            WTIMER5_TAV_R = 0;
            WTIMER5_CTL_R |= TIMER_CTL_TAEN;
            NVIC_EN3_R = 1 << (INT_WTIMER5A-16-96);
            break;

        case 2:
            //configuration for channel 2 (WTIMER0)
            WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;
            WTIMER0_CFG_R = 4;
            WTIMER0_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
            WTIMER0_CTL_R = TIMER_CTL_TAEVENT_POS;
            WTIMER0_IMR_R = TIMER_IMR_CAEIM;
            WTIMER0_TAV_R = 0;
            WTIMER0_CTL_R |= TIMER_CTL_TAEN;
            NVIC_EN2_R = 1 << (INT_WTIMER0A-16-64);
            break;
        case 3:
            //configuration for channel 2 (WTIMER0)
            WTIMER3_CTL_R &= ~TIMER_CTL_TAEN;
            WTIMER3_CFG_R = 4;
            WTIMER3_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
            WTIMER3_CTL_R = TIMER_CTL_TAEVENT_POS;
            WTIMER3_IMR_R = TIMER_IMR_CAEIM;
            WTIMER3_TAV_R = 0;
            WTIMER3_CTL_R |= TIMER_CTL_TAEN;
            NVIC_EN3_R = 1 << (INT_WTIMER3A-16-96);
            break;
    }
}


void disableTimerMode(uint8_t channel)
{
    switch(channel)
    {
        case 0:
            //configuration for channel 0 (WTIMER1)
            WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;
            NVIC_DIS3_R = 1 << (INT_WTIMER1A-16-96);
            break;

        case 1:
            //configuration for channel 1 (WTIMER5)
            WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;
            NVIC_DIS3_R = 1 << (INT_WTIMER5A-16-96);
            break;

        case 2:
            //configuration for channel 2 (WTIMER0)
            WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;
            NVIC_DIS2_R = 1 << (INT_WTIMER0A-16-64);
            break;
        case 3:
            //configuration for channel 3 (WTIMER3)
            WTIMER3_CTL_R &= ~TIMER_CTL_TAEN;
            NVIC_DIS3_R = 1 << (INT_WTIMER3A-16-96);
            break;
    }
}


// Frequency counter service publishing latest frequency measurements every second
void timer1Isr()
{
    frequency = WTIMER1_TAV_R;                   // read counter input
    WTIMER1_TAV_R = 0;                           // reset counter for next period
    GREEN_LED ^= 1;                              // status
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;           // clear interrupt flag
}





//*******************************************************************************
// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1 |SYSCTL_RCGCWTIMER_R5 | SYSCTL_RCGCWTIMER_R0 | SYSCTL_RCGCWTIMER_R2 | SYSCTL_RCGCWTIMER_R3;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2 |SYSCTL_RCGCGPIO_R3| SYSCTL_RCGCGPIO_R5| SYSCTL_RCGCGPIO_R4;
    _delay_cycles(3);

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK | BLUE_LED_MASK;  // bits 1 and 2 are outputs, other pins are inputs
    GPIO_PORTF_DIR_R &= ~PUSH_BUTTON_MASK;               // bit 4 is an input
    GPIO_PORTF_DR2R_R |= GREEN_LED_MASK | BLUE_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= PUSH_BUTTON_MASK | GREEN_LED_MASK | BLUE_LED_MASK;
                                                         // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R |= PUSH_BUTTON_MASK;                // enable internal pull-up for push button



}
void initHCSR04(void)
{
    GPIO_PORTC_DIR_R |= TRIGGER_PIN_MASK_CH0| TRIGGER_PIN_MASK_CH1; // Set Trigger pin as output
    GPIO_PORTC_DEN_R |= TRIGGER_PIN_MASK_CH0 | ECHO_PIN_MASK_CH0 | TRIGGER_PIN_MASK_CH1 | ECHO_PIN_MASK_CH1; // Enable digital function for Trigger and Echo pins

    GPIO_PORTD_DIR_R |= TRIGGER_PIN_MASK_CH2; // Set Trigger pin as output
    GPIO_PORTD_DEN_R |= TRIGGER_PIN_MASK_CH2 | ECHO_PIN_MASK_CH2; // Enable digital function for Trigger and Echo pins

}



void getsUart0(USER_DATA *data)
{
    uint8_t count = 0;
    char c = NULL;

    while (true)
    {
        c = getcUart0();

        if ((c == 8 || c == 127) && count > 0)
        {
            count--;
        }
        else if (c == 13)
        {
            data->buffer[count] = '\0';
            return;
        }
        else if (c >= 32)
        {
            data->buffer[count] = c;
            count++;
        }

        if (count == MAX_CHARS)
        {
            data->buffer[count] = '\0';
            return;
        }
    }
}

void parseFields(USER_DATA *data)
{
    int i;
    int len = strlen(data->buffer);
    char *buffer = data->buffer;
    data->fieldCount = 0;
    char prevType = 'd';    // Assume the previous character type is a delimiter

    for (i = 0; i < len && data->fieldCount < MAX_FIELDS; i++)
    {
        char c = buffer[i];
        char currType;

        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'))
        {
            currType = 'a';
        }
        else if ((c >= '0' && c <= '9') || c == '-' || c == '.')
        {
            currType = 'n';
        }
        else
        {
            currType = 'd';
            buffer[i] = '\0';   // Convert delimiter to NULL character
        }

        if (prevType == 'd' && (currType == 'a' || currType == 'n'))
        {
            data->fieldType[data->fieldCount] = currType;
            data->fieldPosition[data->fieldCount] = i;
            data->fieldCount++;
        }

        prevType = currType;
    }
}

char *getFieldString(USER_DATA *data, uint8_t fieldNumber)
{
    if (fieldNumber < data->fieldCount)
    {
        return &(data->buffer[data->fieldPosition[fieldNumber]]);
    }
    else
    {
        return NULL;
    }
}


int32_t getFieldInteger(USER_DATA *data, uint8_t fieldNumber)
{
    int32_t fieldValue = 0;

    // Check if fieldNumber is in range and fieldType is numeric
    if (fieldNumber < data->fieldCount && data->fieldType[fieldNumber] == 'n')
    {
        // Convert the field string to an integer and return the value
        fieldValue = atoi(&data->buffer[data->fieldPosition[fieldNumber]]);
    }

    return fieldValue;
}


bool isCommand(USER_DATA *data, const char strCommand[], uint8_t minArguments)
{
    if (strcmp(strCommand, &data->buffer[data->fieldPosition[0]]) != 0)
    {
        return false;
    }
    uint8_t argumentCount = data->fieldCount - 1;
    if (argumentCount < minArguments)
    {
        return false;
    }

    return true;
}

//*********************************************************************
void initEeprom(void)
{
    SYSCTL_RCGCEEPROM_R = 1;
    _delay_cycles(3);
    while (EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
}

void writeEeprom(uint16_t add, uint32_t data)
{
    EEPROM_EEBLOCK_R = add >> 4;
    EEPROM_EEOFFSET_R = add & 0xF;
    EEPROM_EERDWR_R = data;
    while (EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
}

uint32_t readEeprom(uint16_t add)
{
    EEPROM_EEBLOCK_R = add >> 4;
    EEPROM_EEOFFSET_R = add & 0xF;
    return EEPROM_EERDWR_R;
}

// Period timer service publishing latest time measurements every positive edge
void wideTimer1Isr(uint8_t n)
{
   switch (n)
   {
   case 0:
        time = WTIMER1_TAV_R/40;                        // read counter input
        WTIMER1_TAV_R = 0;                           // zero counter for next edge
        WTIMER1_ICR_R = TIMER_ICR_CAECINT;           // clear interrupt flag
        break;
   case 1:
       time1 = WTIMER5_TAV_R/40;                        // read counter input
       WTIMER5_TAV_R = 0;                           // zero counter for next edge
       WTIMER5_ICR_R = TIMER_ICR_CAECINT;
       break;
   case 2:
       time2 = WTIMER0_TAV_R/40;                        // read counter input
       WTIMER0_TAV_R = 0;                           // zero counter for next edge
       WTIMER0_ICR_R = TIMER_ICR_CAECINT;
       break;
   case 3:
       time3 = WTIMER3_TAV_R/40;                        // read counter input
       WTIMER3_ICR_R = TIMER_ICR_CAECINT;
       break;

   }
}



//Function measure distance
uint32_t measure_mm(uint8_t n)
{
    uint32_t distance = 0;
    uint32_t time_us;
    uint32_t timeout;

    if (n == 0)
    {
        GPIO_PORTC_DATA_R |= TRIGGER_PIN_MASK_CH0;
        waitMicrosecond(10);
        GPIO_PORTC_DATA_R &= ~TRIGGER_PIN_MASK_CH0;


        timeout = 100000;
        while(!(GPIO_PORTC_DATA_R & ECHO_PIN_MASK_CH0) && timeout)
        {
            timeout--;

        }
        if (!timeout)
        {
            disableTimerMode(0);
            return 0;
        }

        enableTimerMode(0);
        while(GPIO_PORTC_DATA_R & ECHO_PIN_MASK_CH0);
        wideTimer1Isr(0);
        disableTimerMode(0);
        distance = (time * 1.1 * 345) / 2000; //k=1.1 constant

        return distance;

    }
    else if (n == 1)
    {
        GPIO_PORTC_DATA_R |= TRIGGER_PIN_MASK_CH1;
        waitMicrosecond(10);
        GPIO_PORTC_DATA_R &= ~TRIGGER_PIN_MASK_CH1;


        timeout = 100000;
        while(!(GPIO_PORTC_DATA_R & ECHO_PIN_MASK_CH1) && timeout)
        {
            timeout--;

        }
        if (!timeout)
        {
            disableTimerMode(1);
            return 0;
        }

        waitMicrosecond(100);
        enableTimerMode(1);
        while(GPIO_PORTC_DATA_R & ECHO_PIN_MASK_CH1);
        wideTimer1Isr(1);
        disableTimerMode(1);
        distance = (time1 * 1.1 * 345) / 2000; //k=1.1 constant

        return distance;
    }
    else if (n == 2)
    {
        GPIO_PORTD_DATA_R |= TRIGGER_PIN_MASK_CH2;
        waitMicrosecond(10);
        GPIO_PORTD_DATA_R &= ~TRIGGER_PIN_MASK_CH2;


        timeout = 100000;
        while(!(GPIO_PORTD_DATA_R & ECHO_PIN_MASK_CH2) && timeout)
        {
            timeout--;

        }
        if (!timeout)
        {
            disableTimerMode(2);
            return 0;
        }

        waitMicrosecond(100);
        enableTimerMode(2);
        while(GPIO_PORTD_DATA_R & ECHO_PIN_MASK_CH2);
        wideTimer1Isr(2);
        disableTimerMode(2);
        distance = (time2 * 1.1 * 345) / 2000; //k=1.1 constant

        return distance;
    }
    else
    {
        return 0; // Invalid channel
    }


}



//*************************************************************


void saveDeviceParameters(EVENT_DATA *events, uint8_t numberOfEvents)
{

    uint8_t i;
    for (i = 0; i < numberOfEvents; i++)
    {
        uint16_t eepromAddress = i*sizeof(EVENT_DATA);
        writeEeprom(eepromAddress, events[i].sensor);
        writeEeprom(eepromAddress+1, events[i].minDistMm);
        writeEeprom(eepromAddress+5, events[i].maxDistMm);
        writeEeprom(eepromAddress+9, events[i].haptic);
        writeEeprom(eepromAddress+10, events[i].pwm);
        writeEeprom(eepromAddress+11, events[i].beat);
        writeEeprom(eepromAddress+12, events[i].ontime);
        writeEeprom(eepromAddress+16, events[i].offtime);
    }
}

void loadDeviceParameters(EVENT_DATA *events, uint8_t numberOfEvents)
{
    uint8_t i ;
    for (i = 0; i < numberOfEvents; i++)
    {
        uint16_t eepromAddress = i*sizeof(EVENT_DATA);
        events[i].sensor = readEeprom(eepromAddress );


        events[i].minDistMm = readEeprom(eepromAddress +1);
        events[i].maxDistMm = readEeprom(eepromAddress +5);

        events[i].haptic = readEeprom(eepromAddress +9);
        events[i].pwm = readEeprom(eepromAddress +10);
        events[i].beat = readEeprom(eepromAddress +11);
        events[i].ontime = readEeprom(eepromAddress +12);
        events[i].offtime = readEeprom(eepromAddress +16);

    }
}

void eraseEvent(EVENT_DATA *events, uint8_t eventIndex, uint8_t numberOfEvents)
{
    if (eventIndex < numberOfEvents)
    {
        events[eventIndex].sensor = 0;
        events[eventIndex].minDistMm = 0;
        events[eventIndex].maxDistMm = 0;

    }
}
void erasePattern(EVENT_DATA *events, uint8_t eventIndex, uint8_t numberOfEvents)
{
    if (eventIndex < numberOfEvents)
    {
        events[eventIndex].pwm = 0;
        events[eventIndex].beat = 0;
        events[eventIndex].ontime = 0;
        events[eventIndex].offtime = 0;
        events[eventIndex].haptic = 0;
    }
}


void displayEvents(EVENT_DATA *events, uint8_t numberOfEvents)
{
    putsUart0("Event | Sensor | Min Distance (mm) | Max Distance (mm)\r\n");
    putsUart0("---------------------------------------------------\r\n");
    uint8_t i;
    for (i = 0; i < numberOfEvents; i++)
    {
        char eventString[100];
        sprintf(eventString, "%5u | %6u | %17u | %17u\r\n", i, events[i].sensor, events[i].minDistMm, events[i].maxDistMm);
        putsUart0(eventString);
    }
}

void displayPatterns(EVENT_DATA *events, uint8_t numberOfEvents)
{
    putsUart0("Event | PWM | Beats | On-Time (ms) | Off-Time (ms) | Haptic\r\n");
    putsUart0("-----------------------------------------------------------\r\n");
    uint8_t i;
    for (i = 0; i < numberOfEvents; i++)
    {
        char patternString[100];
        sprintf(patternString, "%5u | %3u | %5u | %12u | %12u | %5s\r\n", i, events[i].pwm, events[i].beat, events[i].ontime, events[i].offtime, events[i].haptic ? "On" : "Off");
        putsUart0(patternString);
    }
}



void setPWM(uint32_t distance[3], EVENT_DATA* events, uint8_t numberOfEvents)
{
    int i, j;
    uint32_t k=0;
    for (i = numberOfEvents - 1; i >= 0; i--)
    {
        EVENT_DATA event = events[i];

        // Check if haptic is on for this event
        if (event.haptic==1)
        {
            // Check if the sensor is in range
            if (event.sensor < 3 && distance[event.sensor] >= event.minDistMm && distance[event.sensor] <= event.maxDistMm)
            {
                // Run the motor with the pattern
                for (j = 0; j < event.beat; j++)
                {
                    k= event.pwm * 10;
                    setRgbColor(k, 0, 0);
                    waitMicrosecond(event.ontime * 1000);

                    setRgbColor(0, 0, 0);
                    waitMicrosecond(event.offtime * 1000);
                }
            }
        }
    }


    setRgbColor(0, 0, 0);
}
//****************************************************************
int main(void)
{
    uint32_t distance[3];

    waitMicrosecond(1000000);

    // Initialize hardware
    initHw();
    initRgb();
    initUart0();
    initHCSR04();
    initEeprom();

    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);
    uint8_t i;
    bool printDistances = false;

    for (i = 0; i < NUMBER_OF_EVENTS; i++)
    {
            events[i].sensor = 0;
         events[i].minDistMm = 0;
         events[i].maxDistMm = 0;
    }
    // Load the event data from EEPROM
    loadDeviceParameters(events, NUMBER_OF_EVENTS);



    USER_DATA data;


    while (1)
    {
          // Measure the distance
          distance[0] = measure_mm(0);
          distance[1] = measure_mm(1);
          distance[2] = measure_mm(2);

          bool inputReceived = false;

          enableTimerMode(3);
          while (time3 < 1000000)
          {
              wideTimer1Isr(3);
              if (kbhitUart0())
              {
                  inputReceived = true;
                  break;
              }

          }

          WTIMER3_TAV_R = 0; // Reset time3 counter

          wideTimer1Isr(3); // Update time3 after resetting the counter



          if (!inputReceived)
          {
              disableTimerMode(3);

              setPWM(distance, events, NUMBER_OF_EVENTS);

              if (printDistances)
              {
                  char str[64];
                  snprintf(str, sizeof(str), "Distance SEN0: %lu mm, SEN1: %lu mm, SEN2: %lu mm\r\n", distance[0], distance[1], distance[2]);
                  putsUart0(str);
              }

              continue;
          }



              // Get input from the user
              getsUart0(&data);

              // Parse the input
              parseFields(&data);

              // Handle the "event" command
              if (isCommand(&data, "event", 4))
              {
                  uint8_t eventIndex = getFieldInteger(&data, 1);
                  uint8_t sensor = getFieldInteger(&data, 2);
                  uint32_t minDistMm = getFieldInteger(&data, 3);
                  uint32_t maxDistMm = getFieldInteger(&data, 4);

                  if (eventIndex < NUMBER_OF_EVENTS)
                  {
                      events[eventIndex].sensor = sensor;
                      events[eventIndex].minDistMm = minDistMm;
                      events[eventIndex].maxDistMm = maxDistMm;

                      // Save the updated event to the EEPROM
                      saveDeviceParameters(events, NUMBER_OF_EVENTS);

                      putsUart0("Event updated successfully.\r\n");
                  } else
                  {
                      putsUart0(data.buffer);
                      putsUart0("-Invalid event index.\r\n");
                  }
              }
              else if (isCommand(&data, "erase", 2) && strcmp(getFieldString(&data, 1), "event") == 0)
              {
                  uint8_t eventIndex = getFieldInteger(&data, 2);

                  if (eventIndex < NUMBER_OF_EVENTS && data.fieldCount == 3)
                  {
                      eraseEvent(events, eventIndex, NUMBER_OF_EVENTS);
                      saveDeviceParameters(events, NUMBER_OF_EVENTS);
                      putsUart0("Event erased successfully.\r\n");
                  }
                  else
                  {
                      putsUart0(data.buffer);
                      putsUart0("-Invalid event index or incorrect command. Syntax: erase event #\r\n");
                  }
              }

              else if (isCommand(&data, "erase", 2) && strcmp(getFieldString(&data, 1), "pattern") == 0)
              {
                  uint8_t eventIndex = getFieldInteger(&data, 2);

                  if (eventIndex < NUMBER_OF_EVENTS && data.fieldCount == 3)
                  {
                      erasePattern(events, eventIndex, NUMBER_OF_EVENTS);
                      saveDeviceParameters(events, NUMBER_OF_EVENTS);
                      putsUart0("Pattern erased successfully.\r\n");
                  }
                  else
                  {
                      putsUart0(data.buffer);
                      putsUart0("-Invalid event index or incorrect command. Syntax: erase pattern #\r\n");
                  }
              }



              else if (isCommand(&data, "and", 3))
              {
                  uint8_t eventIndex0 = getFieldInteger(&data, 1);
                  uint8_t eventIndex1 = getFieldInteger(&data, 2);
                  uint8_t eventIndex2 = getFieldInteger(&data, 3);

                  if(eventIndex0 >= 16 && eventIndex0<= 19 && (events[eventIndex1].sensor == events[eventIndex2].sensor) )
                  {
                      events[eventIndex0].sensor = events[eventIndex1].sensor;

                      events[eventIndex0].minDistMm =(events[eventIndex1].minDistMm > events[eventIndex2].minDistMm) ? events[eventIndex2].minDistMm : events[eventIndex1].minDistMm ;
                      events[eventIndex0].maxDistMm =(events[eventIndex1].maxDistMm > events[eventIndex2].maxDistMm) ? events[eventIndex1].maxDistMm : events[eventIndex2].maxDistMm ;
                      // Save the updated event to the EEPROM
                      saveDeviceParameters(events, NUMBER_OF_EVENTS);

                      putsUart0("Compound Event updated successfully.\r\n");
                  }
                  else
                  {

                      putsUart0("-The command or index is invalid - the AND operation can only be saved on events [16-19] that have the same sensor.\r\n");
                  }
              }

              // Handle the "show" command
              else if (isCommand(&data, "show", 1) && strcmp(getFieldString(&data, 1), "events") == 0)
              {
                  displayEvents(events, NUMBER_OF_EVENTS);
              }

              // Handle the "reboot" command
              else if (isCommand(&data, "reboot", 0))
              {
                  // Clear all event data and save to EEPROM


                  for (i =0 ; i< NUMBER_OF_EVENTS; i++)
                  {
                      eraseEvent(events,i,NUMBER_OF_EVENTS);
                      erasePattern(events, i, NUMBER_OF_EVENTS);

                  }

                  saveDeviceParameters(events, NUMBER_OF_EVENTS);

                  putsUart0("System is rebooting...\r\n");
                  // Reset the microcontroller
                  NVIC_APINT_R = NVIC_APINT_SYSRESETREQ;
                  waitMicrosecond(1000000);
                  putsUart0("=>System is successfully rebooted.\r\n");
              }

              else if (isCommand(&data, "haptic", 2))
              {
                  uint8_t eventIndex = getFieldInteger(&data, 1);
                  char hapticState[4];
                  strcpy(hapticState, getFieldString(&data, 2));
                  if (eventIndex < NUMBER_OF_EVENTS)
                  {
                      if (strcmp(hapticState, "on") == 0)
                      {
                          events[eventIndex].haptic = 1;
                      }
                      else if (strcmp(hapticState, "off") == 0)
                      {
                          events[eventIndex].haptic = 0;
                      }
                      // Save the updated event to the EEPROM
                      saveDeviceParameters(events, NUMBER_OF_EVENTS);
                      putsUart0("Haptic state updated successfully.\r\n");
                  }
                  else
                  {
                      putsUart0(data.buffer);
                      putsUart0("-Invalid event index.\r\n");
                  }
              }
              else if (isCommand(&data, "pattern", 5))
              {
                  uint8_t eventIndex = getFieldInteger(&data, 1);
                  uint8_t pwm = getFieldInteger(&data, 2);
                  uint8_t beats = getFieldInteger(&data, 3);
                  uint32_t onTime = getFieldInteger(&data, 4);
                  uint32_t offTime = getFieldInteger(&data, 5);

                  if (eventIndex < NUMBER_OF_EVENTS)
                  {
                      events[eventIndex].pwm = pwm;
                      events[eventIndex].beat = beats;
                      events[eventIndex].ontime = onTime;
                      events[eventIndex].offtime = offTime;

                      saveDeviceParameters(events, NUMBER_OF_EVENTS);
                      putsUart0("Pattern settings updated successfully.\r\n");
                  }
                  else
                  {
                      putsUart0(data.buffer);
                      putsUart0("-Invalid event index.\r\n");
                  }
              }

              else if (isCommand(&data, "show", 1) && strcmp(getFieldString(&data, 1), "patterns") == 0)
              {
                  displayPatterns(events, NUMBER_OF_EVENTS);
              }

              else if (isCommand(&data, "sensors", 1))
              {
                  char* state = getFieldString(&data, 1);
                  if (strcmp(state, "on") == 0)
                  {
                      printDistances = true;
                      putsUart0("Sensors output enabled.\r\n");
                  }
                  else if (strcmp(state, "off") == 0)
                  {
                      printDistances = false;
                      putsUart0("Sensors output disabled.\r\n");
                  }
                  else
                  {
                      putsUart0(data.buffer);
                      putsUart0("-Invalid command. Syntax: sensors on|off\r\n");
                  }
              }



              // Handle invalid commands
              else
              {
                  putsUart0(data.buffer);
                  putsUart0("-Invalid command.\r\n");
              }
              waitMicrosecond(100000);



      }
  }
