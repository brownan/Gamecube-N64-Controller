
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
    unsigned char left;
    unsigned char right;
} gc_status;
char gc_raw_dump[65]; // 1 received bit per byte

void gc_send(unsigned char *buffer, char length);
void gc_get();
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
        gc_status.data1 |= gc_raw_dump[i] ? (0x80 >> i) : 0;
    }
    // line 2
    // bits: 1, l, r, z, dup, ddown, dright, dleft
    for (i=0; i<8; i++) {
        gc_status.data2 |= gc_raw_dump[8+i] ? (0x80 >> i) : 0;
    }
    // line 3
    // bits: joystick x value
    // these are signed values, but what format? I'm guessing
    // one's complement, because the values I get look strange
    // when viewed directly (most things are two's complement)
    for (i=0; i<8; i++) {
        gc_status.stick_x |= gc_raw_dump[16+i] ? (0x80 >> i) : 0;
    }
    for (i=0; i<8; i++) {
        gc_status.stick_y |= gc_raw_dump[24+i] ? (0x80 >> i) : 0;
    }
    for (i=0; i<8; i++) {
        gc_status.cstick_x |= gc_raw_dump[32+i] ? (0x80 >> i) : 0;
    }
    for (i=0; i<8; i++) {
        gc_status.cstick_y |= gc_raw_dump[40+i] ? (0x80 >> i) : 0;
    }
    for (i=0; i<8; i++) {
        gc_status.left |= gc_raw_dump[48+i] ? (0x80 >> i) : 0;
    }
    for (i=0; i<8; i++) {
        gc_status.right |= gc_raw_dump[56+i] ? (0x80 >> i) : 0;
    }
}

/**
 * This sends the given byte sequence to the controller
 * length must be at least 1
 */
void gc_send(unsigned char *buffer, char length)
{
    // Send these bytes
    char bits;
    char byte_index;
    
    bool bit;

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

}

void gc_get()
{
    // Listening for an expected 8 bytes of data from the controller and put
    // them into gc_status, an 8 byte array.
    // we put the received bytes into the array backwards, so the first byte
    // goes into slot 0.
    asm volatile (";Starting to listen");
    unsigned char timeout;
    char bitcount = 64;
    char *bitbin = gc_raw_dump;

    // Again, using gotos here to make the assembly more predictable and
    // optimization easier (please don't kill me)
read_loop:
    timeout = 0x3f;
    // wait for line to go low
    while (GC_QUERY) {
        if (!--timeout)
            return;
    }
    // wait approx 2us and poll the line
    asm volatile (
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\nnop\nnop\nnop\nnop\n"  
            );
    *bitbin = GC_QUERY;
    ++bitbin;
    --bitcount;
    if (bitcount == 0)
        return;

    // wait for line to go high again
    // it may already be high, so this should just drop through
    timeout = 0x3f;
    while (!GC_QUERY) {
        if (!--timeout)
            return;
    }
    goto read_loop;

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

    // clear out raw data buffer
    memset(gc_raw_dump, 0, sizeof(gc_raw_dump));

    // we write the status command to the controller:
    unsigned char command[] = {0x40, 0x03, 0x00};

    // turn on the led, so we can visually see things are happening
    digitalWrite(13, HIGH); // Set led to on
    // don't want interrupts getting in the way
    noInterrupts();
    // send those 3 bytes
    gc_send(command, 3);
    // read in data and dump it to gc_raw_dump
    gc_get();
    // end of time sensitive code
    interrupts();
    digitalWrite(13, LOW); // set led to off

    // translate the data in gc_raw_dump to something useful
    translate_raw_data();

    // and print it
    print_gc_status();

  
  
  delay(1000);
}

