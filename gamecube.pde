
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
struct {
    unsigned char data1;
    unsigned char data2;
    char stick_x;
    char stick_y;
    char cstick_x;
    char cstick_y;
    char left;
    char right;
} gc_status;
char gc_status_extended[66]; // 1 received bit per byte
// I mysteriously seemed to get an extra bit, maybe my timings were off
// somewhere, but this seemed to produce accurate results. That's why
// this array is 66 and not the expected 65 (8 bytes + 1 stop bit)

void get_gc_status(unsigned char *buffer, char length);
void print_gc_status();
void translate_raw_data();

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

  
}

void translate_raw_data()
{
    // The get_gc_status function sloppily dumps its data 1 bit per byte
    // into the get_status_extended char array. It's our job to go through
    // that and put each piece neatly into the struct gc_status
    int i;
    memset(&gc_status, 0, sizeof(gc_status));
    // line 1
    // bits: 0, 0, 0, start, y, x, b a
    for (i=0; i<8; i++) {
        gc_status.data1 |= gc_status_extended[65-i] ? (0x80 >> i) : 0;
    }
    // line 2
    // bits: 1, l, r, z, dup, ddown, dright, dleft
    for (i=0; i<8; i++) {
        gc_status.data2 |= gc_status_extended[65-8-i] ? (0x80 >> i) : 0;
    }
    // line 3
    // bits: joystick x value
    for (i=0; i<8; i++) {
        gc_status.stick_x |= gc_status_extended[65-16-i] ? (0x80 >> i) : 0;
    }
    gc_status.stick_x ^= 0xff;
    for (i=0; i<8; i++) {
        gc_status.stick_y |= gc_status_extended[65-24-i] ? (0x80 >> i) : 0;
    }
    gc_status.stick_y ^= 0xff;
    for (i=0; i<8; i++) {
        gc_status.cstick_x |= gc_status_extended[65-32-i] ? (0x80 >> i) : 0;
    }
    for (i=0; i<8; i++) {
        gc_status.cstick_y |= gc_status_extended[65-40-i] ? (0x80 >> i) : 0;
    }
    for (i=0; i<8; i++) {
        gc_status.left |= gc_status_extended[65-48-i] ? (0x80 >> i) : 0;
    }
    for (i=0; i<8; i++) {
        gc_status.right |= gc_status_extended[65-56-i] ? (0x80 >> i) : 0;
    }
}

/**
 * This sends the given byte sequence to the controller,
 * and dumps the response into gc_status_extended
 * buffer is a pointer to a byte array, length is the
 * length of that byte array
 * length must be at least 1
 */
void get_gc_status(unsigned char *buffer, char length)
{
    // Send these bytes
    char bits;
    char byte_index;
    
    bool bit;

    // Turn off interrupts
    noInterrupts();

    // This routine is very carefully timed by examining the assembly output.
    // Do not change any statements, it could throw the timings off
    //
    // We get 16 cycles per microsecond, which should be plenty, but we need to
    // be conservative. Most assembly ops take 1 cycle, but a few take 2
    //
    // I use manually constructed for-loops out of gotos so I have more control
    // over the outputted assembly. I can insert nops where it was impossible
    // with a for loop
    
    asm volatile (";Starting outer for loop");
outer_loop:
    {
        asm volatile (";Starting inner for loop");
        bits=8;
inner_loop:
        {
            // Starting a bit, set the line low
            asm volatile (";Setting line to low");
            GC_LOW; // 1 op, 2 cycles

            asm volatile (";branching");
            if (*buffer >> 7) {
                asm volatile (";Bit is a 1");
                // 1 bit
                // remain low for 1us, then go high for 3us
                // nop block 1
                asm volatile ("nop\nnop\nnop\nnop\nnop\n");
                
                asm volatile (";Setting line to high");
                GC_HIGH;

                // nop block 2
                // we'll wait only 2us to sync up with both conditions
                // at the bottom of the if statement
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
            // branch path

            asm volatile (";finishing inner loop body");
            --bits;
            if (bits != 0) {
                // nop block 4
                asm volatile ("nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\n");
                // rotate bits
                asm volatile (";rotating out bits");
                *buffer <<= 1;

                goto inner_loop;
            } // fall out of inner loop
        }
        asm volatile (";continuing outer loop");
        // In this case: the inner loop exits and the outer loop iterates,
        // there are /exactly/ 16 cycles taken up by the necessary operations.
        // So no nops are needed here (that was lucky!)
        --length;
        if (length != 0) {
            ++buffer;
            goto outer_loop;
        } // fall out of outer loop
    }

    // send a single stop (1) bit
    // nop block 5
    asm volatile ("nop\nnop\nnop\nnop\n");
    GC_LOW;
    // wait 1 us, 16 cycles, then raise the line 
    // 16-2=14
    // nop block 6
    asm volatile ("nop\nnop\nnop\nnop\nnop\n"
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\nnop\nnop\nnop\n");
    GC_HIGH;


    // Listening for an expected 8 bytes of data from the controller and put
    // them into gc_status, an 8 byte array.
    // we put the received bytes into the array backwards, so the first byte
    // goes into slot 0.
    asm volatile (";Starting to listen");
    unsigned int timeout = 60000;
    bits = 65;
    while (timeout > 0 && bits >= 0)
    {
        if (!GC_QUERY) {
            // line went low
            asm volatile ("; waiting 2us and then polling for line state\n"
                          "nop\nnop\nnop\nnop\nnop\n"  
                          "nop\nnop\nnop\nnop\nnop\n"  
                          "nop\nnop\nnop\nnop\nnop\n"  
                          "nop\nnop\nnop\nnop\nnop\n"  
                          "nop\nnop\nnop\nnop\nnop\n"  
                          "nop\nnop\nnop\nnop\nnop\n"  
                          ""); 
            gc_status_extended[bits] = GC_QUERY;
            --bits;

            while (!GC_QUERY) {} // wait to go high
        }
        --timeout;
    }

    // re-enable interrupts
    interrupts();

}

void print_gc_status()
{
    int i;
    Serial.print("Start: ");
    Serial.println(gc_status.data1 & 0x10 ? 1:0);

    Serial.print("Y:     ");
    Serial.println(gc_status.data1 & 0x08 ? 1:0);

    Serial.print("X:     ");
    Serial.println(gc_status.data1 & 0x04 ? 1:0);

    Serial.print("B:     ");
    Serial.println(gc_status.data1 & 0x02 ? 1:0);

    Serial.print("A:     ");
    Serial.println(gc_status.data1 & 0x01 ? 1:0);

    Serial.print("L:     ");
    Serial.println(gc_status.data2 & 0x40 ? 1:0);
    Serial.print("R:     ");
    Serial.println(gc_status.data2 & 0x20 ? 1:0);
    Serial.print("Z:     ");
    Serial.println(gc_status.data2 & 0x10 ? 1:0);

    Serial.print("Dup:   ");
    Serial.println(gc_status.data2 & 0x08 ? 1:0);
    Serial.print("Ddown: ");
    Serial.println(gc_status.data2 & 0x04 ? 1:0);
    Serial.print("Dright:");
    Serial.println(gc_status.data2 & 0x02 ? 1:0);
    Serial.print("Dleft: ");
    Serial.println(gc_status.data2 & 0x01 ? 1:0);

    Serial.print("Stick X:");
    Serial.println(gc_status.stick_x, DEC);
    Serial.print("Stick Y:");
    Serial.println(gc_status.stick_y, DEC);

    Serial.print("cStick X:");
    Serial.println(gc_status.cstick_x, DEC);
    Serial.print("cStick Y:");
    Serial.println(gc_status.cstick_y, DEC);

    Serial.print("L:     ");
    Serial.println(gc_status.left, DEC);
    Serial.print("R:     ");
    Serial.println(gc_status.right, DEC);
}

void loop()
{

  memset(gc_status_extended, 0, sizeof(gc_status_extended));

  unsigned char command[] = {0x40, 0x03, 0x00};
  digitalWrite(13, HIGH); // Set led to on
  get_gc_status(command, 3);
  digitalWrite(13, LOW); // set led to off

  translate_raw_data();

  print_gc_status();

  
  
  delay(1000);
}

