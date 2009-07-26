
#include "pins_arduino.h"

#define GC_PIN 2
#define GC_PIN_DIR DDRD
// these two macros set arduino pin 2 to input or output, which with an
// external 1K pull-up resistor to the 3.3V rail, is like pulling it high or
// low.  These operations translate to 1 op code, which takes 2 cycles
#define GC_HIGH DDRD &= ~0x04
#define GC_LOW DDRD |= 0x04
#define GC_QUERY (PIND & 0x04)

// 8 bytes of data that we get from the controller
char gc_status[8];

void get_gc_status(bool rumble);

void setup()
{
  Serial.begin(9600);

  Serial.print("bit: 0x");
  Serial.println(digitalPinToBitMask(GC_PIN), HEX);
  
  Serial.print("port: 0x");
  Serial.println(digitalPinToPort(GC_PIN), HEX);

  // Status LED
  digitalWrite(13, LOW);
  pinMode(13, OUTPUT);

  // Communication with gamecube controller on this pin
  // Don't remove these lines, we don't want to push +5V to the controller
  digitalWrite(GC_PIN, LOW);  
  pinMode(GC_PIN, INPUT);

  memset(gc_status, 0, 8);

  /*
  Serial.println("Getting status...\n");
  get_gc_status(false);
  Serial.println("\nDone\n");
  */
  
}

/**
 * This sends the magic poll byte 0x400302 to the controller,
 * and dumps the response into gc_status
 */
void get_gc_status(bool rumble)
{
    // Send these bytes
    unsigned char buffer[] = {0x40, 0x03, 0x02};
    char bits;
    char byte_index;
    
    bool bit;

    // Turn off interrupts
    noInterrupts();

    // This routine is very carefully timed by examining the assembly output.
    // Do not change any statements, it could through the timings off
    //
    // We get 16 cycles per microsecond, which should be plenty, but we need to
    // be conservative. Most assembly ops take 1 cycle, but a few take 2
    
    asm volatile (";Starting outer for loop");
    for (byte_index=0; byte_index < 3; ++byte_index)
    {
        asm volatile (";Starting inner for loop");
        // manual for-loop using gotos, so I can insert nops in places that
        // weren't possible before
        bits=7;
inner_loop:
        {
            // Starting a bit, set the line low
            asm volatile (";Setting line to low");
            GC_LOW; // 1 op, 2 cycles

            asm volatile (";branching");
            if (buffer[byte_index] >> 7) {
                asm volatile (";Bit is a 1");
                // 1 bit
                // remain low for 1us, then go high for 3us
                // 9 cycles have gone by so far to get here
                // nop block 1
                asm volatile ("nop\nnop\nnop\nnop\nnop\n");
                
                asm volatile (";Setting line to high");
                GC_HIGH;

                // wait for 3us
                asm volatile ("; need to wait 3us here");
                // nop block 2
                // we'll wait only 2us to sync up with both conditions
                // at the bottom of the if statement
                // subtract 2 cycles for the jump
                // 32 - 2 = 30
                asm volatile ("nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              );

            } else {
                asm volatile (";Bit is a 0");
                // 0 bit
                // remain low for 3us, then go high for 1us
                // 10 cycles have gone by so far to get here
                // nop block 3
                asm volatile ("nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\n");

                asm volatile (";Setting line to high");
                GC_HIGH;

                // wait for 1us
                asm volatile ("; end of conditional branch, need to wait 1us more before next bit");
                
            }
            // end of the if, the line is high and needs to remain
            // high for exactly 16 more cycles, regardless of the previous
            // if branch taken

            // rotate bits
            asm volatile (";rotating out bits");
            buffer[byte_index] <<= 1;
            asm volatile (";continuing inner loop");
            --bits;
            if (bits != 0) {
                // nop block 4
                asm volatile ("nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\n");
                goto inner_loop;
            }
        }
        asm volatile (";continuing outer loop");
        // there are /exactly/ 16 cycles since the end of the conditional above
        // until the line goes low again in the case that the inner loop
        // terminates and the outer loop continues.  so no nops are needed here
    }

    // Listening for an expected 8 bytes of data from the controller and put
    // them into gc_status, an 8 byte array.
    // we put the received bytes into the array backwards, so the first byte
    // goes into slot 0.
    asm volatile (";Starting to listen");
    for (byte_index=0; byte_index<8; ++byte_index)
    {
        for (bits=0; bits<8; ++bits)
        {
            // Loop while the line is high, exit the loop when it goes low
            while (GC_QUERY) {}

            // wait 2us and poll again
            asm volatile ("; waiting 2us and then polling for line state\n"
                          "nop\nnop\nnop\nnop\nnop\n"  
                          "nop\nnop\nnop\nnop\nnop\n"  
                          "nop\nnop\nnop\nnop\nnop\n"  
                          "nop\nnop\nnop\nnop\nnop\n"  
                          "nop\nnop\nnop\nnop\nnop\n"  
                          "nop\nnop\nnop\nnop\nnop\n"  
                          "nop\nnop\n"); 
            bit = 0; // TODO

            // push the bit into the buffer
            gc_status[byte_index] <<= 1;
            gc_status[byte_index] &= bit;

            // wait for the line to go high again. we know how long this is
            // going to be (2us), but it makes it easier to drop back into the top of
            // the loop when the line is high
            while (!GC_QUERY){}
        }
    }

    // re-enable interrupts
    interrupts();

}

void print_gc_status()
{
    int i;
    Serial.print("Gamecube Status: 0x");
    for (i=0; i<8; ++i)
        Serial.print(gc_status[i], HEX);
    Serial.println();
}

void loop()
{
//  PORTB |= 0x20; // DIO 13 HIGH
//  PORTB &= ~0x20; // DIO 13 LOW
  digitalWrite(13, HIGH); // Set led to on

  if (GC_QUERY)
      Serial.println("HIGH");
  else
      Serial.println("LOW");
  delay(1000);

  digitalWrite(13, LOW); // set led to off
  
  if (GC_QUERY)
      Serial.println("HIGH");
  else
      Serial.println("LOW");
  
  delay(1000);
}

