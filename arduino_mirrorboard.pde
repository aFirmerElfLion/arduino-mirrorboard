/*-
 * Copyright (c) 2011 Darran Hunt (darran [at] hunt dot net dot nz)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * This is an implementation of xkcd's mirrorboard
 * based on http://hunt.net.nz/users/darran/weblog/c6f35/Arduino_USB_Keyboard_Passthrough.html
 */


/* Arduino Keyboard HID driver and USB host shield pass through */

#include <Max3421e.h>
#include <Usb.h>

/* keyboard data taken from configuration descriptor */
#define KBD_ADDR        1
#define KBD_EP          1
#define KBD_IF          0
#define EP_MAXPKTSIZE   8
#define EP_POLL         0x0a

/* Sticky keys */
#define CAPSLOCK    57
#define NUMLOCK     71
#define SCROLLLOCK  83
#define SPACEBAR    44  

/* Sticky keys output report bitmasks */
#define REP_NUMLOCK       0x01
#define REP_CAPSLOCK      0x02
#define REP_SCROLLLOCK    0x04

EP_RECORD ep_record[2];  //endpoint record structure for the keyboard

uint8_t buf[8] = { 0 };      // keyboard buffer
uint8_t old_buf[8] = { 0 };// last poll
uint8_t very_old_buf[8] = { 0 };
uint8_t sub_buf[8] = { 0 }; 
uint8_t old_sub_buf[8] = { 0 };
/* Sticky key state */
bool numLock = false;
bool capsLock = false;
bool scrollLock = false;
bool switched = false;

void setup();
void loop();
bool key_is_new(byte key, byte *report);
void keySub(uint8_t *buf, uint8_t *subbuf);

/* key substitution table; mirrors keys */
const uint8_t sub[54][1] = {
    { 51 }, /* a */
    { 5 },  /* b */
    { 16 }, /* c */
    { 14 }, /* d */
    { 12 }, /* e */
    { 13 }, /* f */
    { 11 }, /* g */
    { 10 }, /* h */
    { 8 },  /* i */
    { 9 },  /* j */
    { 7 },  /* k */
    { 22 }, /* l */
    { 6 },  /* m */
    { 25 }, /* n */
    { 26 }, /* o */
    { 20 }, /* p */
    { 19 }, /* q */
    { 24 }, /* r */
    { 15 }, /* s */
    { 28 }, /* t */
    { 21 }, /* u */
    { 17 }, /* v */
    { 18 }, /* w */
    { 54 }, /* x */
    { 23 }, /* y */
    { 55 }, /* z */
    { 45 }, /* 1 */
    { 39 },  /* 2 */
    { 38 }, /* 3 */
    { 37 }, /* 4 */
    { 36 }, /* 5 */
    { 35 }, /* 6 */
    { 34 }, /* 7 */
    { 33 }, /* 8 */
    { 32 }, /* 9 */
    { 31 }, /* 0 */
    { 40 }, /* RETURN */
    { 41 }, /* ESCAPE */
    { 42 }, /* DELETE */
    { 42 }, /* Tab */
    { 44 }, /* spacebar */
    { 30 }, /* - */
    { 53 }, /* ´ */
    { 47 }, /* ü */
    { 48 }, /* + */
    { 49 }, /* # */
    { 50 }, /* 50 */
    { 4 },  /* ö */
    { 52 }, /* ä */
    { 46 }, /* ^ */
    { 27 }, /* , */
    { 29 }, /* . */
    { 100 },/* - */
    { 40 }  /* capslock */

};

MAX3421E Max;
USB Usb;

void setup() 
{
    Serial.begin(9600);
    Max.powerOn();
    delay(200);
}

void loop() 
{
    Max.Task();
    Usb.Task();

    if (Usb.getUsbTaskState() == USB_STATE_CONFIGURING) {
        kbd_init();
        Usb.setUsbTaskState( USB_STATE_RUNNING);
    }

    if (Usb.getUsbTaskState() == USB_STATE_RUNNING) {
        kbd_poll();
    }
}

/* Initialize keyboard */
void kbd_init( void )
{
    byte rcode = 0;  //return code

    /* Initialize data structures */
    ep_record[0] = *(Usb.getDevTableEntry(0,0));  //copy endpoint 0 parameters
    ep_record[1].MaxPktSize = EP_MAXPKTSIZE;
    ep_record[1].Interval  = EP_POLL;
    ep_record[1].sndToggle = bmSNDTOG0;
    ep_record[1].rcvToggle = bmRCVTOG0;
    Usb.setDevTableEntry(1, ep_record);              //plug kbd.endpoint parameters to devtable

    /* Configure device */
    rcode = Usb.setConf(KBD_ADDR, 0, 1);                    
    if (rcode) {
        while(1);  //stop
    }
    delay(2000);
}

/* Poll USB keyboard for new key presses, send through to host via 
 * the Keyboard HID driver in the atmega8u2
 */
void kbd_poll(void)
{
    char i;
    byte rcode = 0;     //return code
    uint8_t ledReport;
    static uint8_t lastLedReport = 0;
    static uint8_t leds = 0;
    static uint8_t lastLeds = 0;

    /* poll keyboard */
    rcode = Usb.inTransfer(KBD_ADDR, KBD_EP, 8, (char *)buf);
    if (rcode != 0) {
	return;
    }
    /* Check for change */
    for (i=0; i<8; i++) {     
            if (buf[i] != old_buf[i]) {
                // substitute keys if spacebar is pressed 
                keySub(buf, sub_buf);
                // Send the new key presses to the host 
                Serial.write(sub_buf, 8); 

             /** // print replaced buffer and buffer
             for( int j = 0; j < 8; j++ ) {
              
                Serial.print( sub_buf[ j ], HEX );
                Serial.print(" ");
            }              
            Serial.print("       ");
            for( int j = 0; j < 8; j++ ) {
              
                Serial.print( buf[ j ], HEX );
                Serial.print(" ");
            }    
             Serial.println(" ");
             **/

	    ledReport = Serial.read();
#if 0
            /* Not working yet, ledReport is always 0  */
	    if (ledReport != lastLedReport) {
		rcode = Usb.setReport( KBD_ADDR, 0, 1, KBD_IF, 0x02, 0, (char *)&ledReport );
		lastLedReport = ledReport;
	    }
#endif
            break;
	}
    }

    // Check for status keys and adjust led status 
    for (i=2; i<8; i++) {
	if (buf[i] == 0) {
	    //0 marks end of keys in the report 
	    break;
	}
	// Check new keys for status change keys 
	if (key_is_new(buf[i], old_buf)) {
	    switch (buf[i]) {
	    case CAPSLOCK:
		capsLock =! capsLock;
		leds = capsLock ? leds |= REP_CAPSLOCK : leds &= ~REP_CAPSLOCK;
		break;
	    case NUMLOCK:
		numLock =! numLock;
		leds = numLock ? leds |= REP_NUMLOCK : leds &= ~REP_NUMLOCK;
		break;
	    case SCROLLLOCK:
		scrollLock =! scrollLock;
		leds = scrollLock ? leds |= REP_SCROLLLOCK : leds &= ~REP_SCROLLLOCK;
		break;
	    default:
	        break;
	    }
	}
    }

    // Got a change in LED status? 
    if (lastLeds != leds) {
	Usb.setReport( KBD_ADDR, 0, 1, KBD_IF, 0x02, 0, (char *)&leds );
	lastLeds = leds;
    }

    // update the old buffer //
    for (i=0; i<8; i++) {
        very_old_buf[i] = old_buf[i];
	old_buf[i] = buf[i];
    }

}

/*
* Function:    key_is_new(key, report)
* Description: see if key is new or is already in report
* Returns:     false if found, true if not
*/

void keySub(uint8_t *buf, uint8_t *subbuf)
{           
    subbuf[0] = buf[0];
    subbuf[1] = buf[1];

    uint8_t tmp_buf[8] = { 0 }; 
    for (int ind=2; ind<8; ind++) {
      tmp_buf[ind] = 0;
    }

    // we do not want to send a signal if the space bar is held, 
    // but if it is pressed and released.
    // therefore, we look for this pattern:
    //
    // 0  0  0  0  0  0  0  0 <- some key is released 
    // 0  0  2c 0  0  0  0  0 <- spacebar (and only the spacebar) is pressed
    // 0  0  0  0  0  0  0  0 <- spacebar is released
    // 
    // and only then do we send the signal that the spacebar is pressed 
    // and released.

    for (int ind=0; ind<8; ind++) {
        if(buf[ind] != 0 || very_old_buf[ind] != 0 ) break;
        
        if( ind==7 && old_buf[2] == SPACEBAR && old_buf[3]==0) {
             Serial.write(old_buf, 8); 
             Serial.write(tmp_buf, 8); 
             return;
        }
    }

    for (int ind=2; ind<8; ind++) {
	subbuf[ind] = buf[ind];
                     // take care of german "<" and "-"
                     if (buf[2] == SPACEBAR && subbuf[3] == 100) {
                     subbuf[3] = 0;
                     subbuf[2] = 56;
                     }
        if ((buf[ind] >= 4) && (buf[ind] <= 57)) {
	    if (buf[2] == SPACEBAR) {
	        subbuf[2] = sub[buf[ind]-4][0];// - 'a' + 4;
                subbuf[ind] = 0;
                     
	    }
          
        }
     }
     if (subbuf[2] == SPACEBAR) subbuf[2] = 0;

}

  

bool key_is_new(uint8_t key, uint8_t *report)
{
    uint8_t ind;
    for (ind=2; ind<8; ind++) {
	if (report[ind] == key) {
	    return false;
	}
    }
    return true;
}

