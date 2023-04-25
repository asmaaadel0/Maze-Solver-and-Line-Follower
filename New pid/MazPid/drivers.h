// volatile pointer, which tells the compiler that the value pointed to by the pointer may change at any time (e.g., if the register is written to by another part of the program or by an interrupt routine). This ensures that the compiler will not optimize away any read or write operations on the register.
#include <avr/io.h>
#define INPUT 0x0
#define OUTPUT 0x1
#define LOW 0
#define HIGH 1
#define _BV(bit) (1 << (bit))
#define A0 14 //port c
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19 //port c
#define PRTA 0
#define PRTB 1
#define PRTC 2
#define PRTD 3
#define PORTB_ADDRESS 0x05
#define PORTC_ADDRESS 0x08
#define PORTD_ADDRESS 0x0B
#define PINB_ADDRESS 0x03
#define PINC_ADDRESS 0x06
#define PIND_ADDRESS 0x09
#define DDRB_ADDRESS 0x04
#define DDRC_ADDRESS 0x07
#define DDRD_ADDRESS 0x0A
void types(String a) { Serial.println("it's a String"); }
void types(int a) { Serial.println("it's an int"); }
void types(char *a) { Serial.println("it's a char*"); }
void types(float a) { Serial.println("it's a float"); }
void types(bool a) { Serial.println("it's a bool"); }

/*
 * avr_digitalPinToBitMask - returns a bitmask of a specified pin
 * 
 * Arguments:
 * pin - unsigned 8-bit integer representing the pin number
 * 
 * Returns:
 * bitmask of specified pin
 */
uint8_t avr_digitalPinToBitMask(uint8_t pin) 
{
    if (pin >= 0 && pin <= 7)
    {
      // portD -> (1 << (pin))
        return _BV(pin);
    }
    else if (pin >= 8 && pin <= 13)
    {
      // portB -> (1 << (pin-8))
        return _BV(pin - 8);
    }
    else if (pin >= A0 && pin <= A5)
    {
      // portC -> (1 << (pin - 14))
        return _BV(pin - 14);
    }
}
/**
 * @brief Returns the port of a specified pin based on its number.
 * 
 * @param pin The pin number.
 * @return The port of the specified pin.
 */
uint8_t avr_digitalPinToPort(uint8_t pin)
{
    if (pin >= 0 && pin <= 7)
    {
        return PRTD;
    }
    else if (pin >= 8 && pin <= 13)
    {
        return PRTB;
    }
    else if (pin >= A0 && pin <= A5)
    {
        return PRTC;
    }
}
//why add 0x20 --> This is because the mode register is located 0x20 bytes after the DDR register in memory.
/**
 * @brief Returns a pointer to the data direction register (DDR) of a specified port based on its number.
 * 
 * @param port The port number.
 * @return uint8_t* A pointer to the DDR of the specified port.
 */
uint8_t *avr_portModeRegister(uint8_t port)
{
    if (port == PRTB)
    {
        return (volatile uint8_t *)(DDRB_ADDRESS + 0x20);
    }
    else if (port == PRTC)
    {
        return (volatile uint8_t *)(DDRC_ADDRESS + 0x20);
    }
    else if (port == PRTD)
    {
        return (volatile uint8_t *)(DDRD_ADDRESS + 0x20);
    }
}
/**
 * @brief Returns a pointer to the input register (PIN) of a specified port based on its number.
 * 
 * @param port The port number.
 * @return uint8_t* A pointer to the PIN of the specified port.
 */
uint8_t *avr_portInputRegister(uint8_t port)
{
    if (port == PRTB)
    {
        return (volatile uint8_t *)(PINB_ADDRESS + 0x20);
    }
    else if (port == PRTC)
    {
        return (volatile uint8_t *)(PINC_ADDRESS + 0x20);
    }
    else if (port == PRTD)
    {
        return (volatile uint8_t *)(PIND_ADDRESS + 0x20);
    }
}
/**
 * @brief Returns a pointer to the output register (PORT) of a specified port based on its number.
 * 
 * @param port The port number.
 * @return uint8_t* A pointer to the PORT of the specified port.
 */
uint8_t *avr_portOutputRegister(uint8_t port)
{
    if (port == PRTB)
    {
        return (volatile uint8_t *)(PORTB_ADDRESS + 0x20);
    }
    else if (port == PRTC)
    {
        return (volatile uint8_t *)(PORTC_ADDRESS + 0x20);
    }
    else if (port == PRTD)
    {
        return (volatile uint8_t *)(PORTD_ADDRESS + 0x20);
    }
}
/**
 * @brief Configures the specified pin as either an input or an output.
 * 
 * @param pin The pin number.
 * @param mode The mode to configure the pin as (INPUT or OUTPUT).
 */
void avr_pinMode(uint8_t pin, uint8_t mode)
{
    // Determine the bit mask and port associated with the specified pin
    uint8_t bit = avr_digitalPinToBitMask(pin);
    uint8_t port = avr_digitalPinToPort(pin);
    // Declare pointers to the DDR register for the port
    volatile uint8_t *reg;
    // Obtain a pointer to the DDR register for the port using avr_portModeRegister()
    reg = avr_portModeRegister(port);
    // Configure the pin as either an input or an output, depending on the mode argument
    if (mode == INPUT)
    {
        // Clear the bit associated with the pin in the DDR register to configure the pin as an input
        *reg &= ~bit;
    }
    else
    {
        // Set the bit associated with the pin in the DDR register to configure the pin as an output
        *reg |= bit;
    }
}
/**
 * @brief Writes a digital value (HIGH or LOW) to the specified pin.
 * 
 * @param pin The pin number.
 * @param val The value to write (HIGH or LOW).
 */
void avr_digitalWrite(uint8_t pin, uint8_t val)
{
    uint8_t bit = avr_digitalPinToBitMask(pin);
    uint8_t port = avr_digitalPinToPort(pin);
    volatile uint8_t *out;
    out = avr_portOutputRegister(port);
    if (val == LOW)
    {
        *out &= ~bit;
    }
    else
    {
        *out |= bit;
    }
}
/**
 * @brief Reads the digital value (HIGH or LOW) from the specified pin.
 * 
 * @param pin The pin number.
 * @return int The value read from the pin (HIGH or LOW).
 */
int avr_digitalRead(uint8_t pin)
{
    uint8_t bit = avr_digitalPinToBitMask(pin);
    uint8_t port = avr_digitalPinToPort(pin);
    // Serial.println(PIND);
    // Serial.println(*avr_portInputRegister(port));
    if (*avr_portInputRegister(port) & bit)
        return HIGH;
    else
        return LOW;
}
// uint16_t adc_read(uint8_t ch)
// {
// // select the corresponding channel 0~5
// // ANDing with ’7′ will always keep the value
// // of ‘ch’ between 0 and 5
// ch &= 0b00000111; // AND operation with 7
// ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing
// // start single conversion
// // write ’1′ to ADSC
// ADCSRA |= (1<<ADSC);
// // wait for conversion to complete
// // ADSC becomes ’0′ again
// // till then, run loop continuously
// while(ADCSRA & (1<<ADSC));
// return (ADC);
// }
int avr_analogRead(uint8_t ch)
{
    uint8_t pin = ch - 14; // first maps the channel number to a corresponding pin number
    pin &= 0b00000111;
    ADMUX = 1 << REFS0; //  sets the ADC reference voltage to use the supply voltage as the reference  //remark
    ADCSRA |= (1 << ADEN); //  enables the ADC
    ADMUX = (ADMUX & 0xF8) | pin;
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return (ADC);
}
// pin, which specifies the pin number to enable PWM output on, and
// cycle, which sets the duty cycle of the PWM output. 
/**
 * @brief Writes an analog value (PWM wave) to the specified pin.
 * 
 * @param pin The analog pin number.
 * @param cycle The duty cycle of the PWM wave (0-255).
 * @return uint8_t 0 if successful, 1 if invalid pin number.
 */
uint8_t avr_analogWrite(uint8_t pin, uint8_t cycle)
{

    if (pin == 3)
    {
        DDRD |= _BV(DDD3);
        TCCR2A |= (_BV(COM2B1) | _BV(WGM21) | _BV(WGM20));
        TCCR2B |= (_BV(CS22) | _BV(CS20));
        OCR2B = cycle;
        return (0);
    }
    else if (pin == 5)
    {
        DDRD |= _BV(DDD5);
        TCCR0A |= (_BV(COM0B1) | _BV(WGM21) | _BV(WGM20));
        TCCR0B |= (_BV(CS21) | _BV(CS20));
        OCR0B = cycle;
        return (0);
    }
    else if (pin == 6)
    {
        DDRD |= _BV(DDD6);
        TCCR0A |= (_BV(COM0A1) | _BV(COM0A0) | _BV(WGM21) | _BV(WGM20));
        TCCR0B |= (_BV(CS21) | _BV(CS20));
        OCR0A = cycle;
        return (0);
    }
    else if (pin == 9)
    {
        DDRB |= _BV(DDB1);
        TCCR1A |= (_BV(COM1A1) | _BV(WGM10));
        TCCR1B |= (_BV(WGM12) | _BV(CS11) | _BV(CS10));
        OCR1A = cycle;
        return (0);
    }
    else if (pin == 10)
    {
        DDRB |= _BV(DDB2);
        TCCR1A |= (_BV(COM1B1) | _BV(WGM10));
        TCCR1B |= (_BV(WGM12) | _BV(CS11) | _BV(CS10));
        OCR1B = cycle;
        return (0);
    }
    else if (pin == 11)
    {
        DDRB |= _BV(DDB3);
        TCCR2A |= (_BV(COM2A1) | _BV(WGM21) | _BV(WGM20));
        TCCR2B |= (_BV(CS22) | _BV(CS20));
        OCR2A = cycle;
        return (0);
    }
    else
    {
        return (1);
    }
}