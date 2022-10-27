#include <Arduino.h>

#include <USBComposite.h>
#include <PsxControllerHwSpi.h>

USBXBox360 usbcon;
PsxControllerHwSpi<PB0> psx;

boolean haveController = false;

#define	toDegrees(rad) (rad * 180.0 / PI)
#define deadify(var, thres) (abs (var) > (thres) ? (var) : 0)

const unsigned long POLLING_INTERVAL = 4U;  // 4ms for x360
const byte ANALOG_DEAD_ZONE = 50U;


uint8_t state = 0;
void setup() {
    pinMode(PC13, OUTPUT);

    digitalWrite(PC13, LOW);
    usbcon.begin();
    usbcon.manualReport = true;

    while (!USBComposite);
    digitalWrite(PC13, HIGH);
}

void die(uint16_t blinkMillis) {
    while (1){
        state ^=1;
        delay(blinkMillis);
        digitalWrite(PC13, state);
    }
}


void loop() {
    static unsigned long last = 0;

    if (millis () - last >= POLLING_INTERVAL) {
        last = millis ();

        if (!haveController) {
            if (psx.begin ()) {
                //Serial1.println("Controller found!");
                //digitalWrite(PC13, LOW);
                if (!psx.enterConfigMode ()) {
                    //Serial1.println("Cannot enter config mode");
                    //digitalWrite(PC13, HIGH);
                    die(500);
                } else {
                    // Try to enable analog sticks
                    if (!psx.enableAnalogSticks ()) {
                        //Serial1.println("Cannot enable analog sticks");
                        //digitalWrite(PC13, HIGH);
                        die(1000);
                    }

                    if (!psx.enableAnalogButtons ()) {
                        //Serial1.println("Cannot enable analog sticks");
                        //digitalWrite(PC13, HIGH);
                        die(1000);
                    }

                    if (!psx.exitConfigMode ()) {
                        //Serial1.println("Cannot exit config mode");
                        //digitalWrite(PC13, HIGH);
                        die(2000);
                    }
                }

                haveController = true;
            }
        } else {
            if (!psx.read ()) {
                //Serial1.println("Controller lost :(");
                haveController = false;
            } else {

                /* Flash led with buttons, I like this but it introduces a bit of
                 * lag, so let's keep it disabled by default
                 */
                //~ digitalWrite (LED_BUILTIN, !!psx.getButtonWord ());

                // Read was successful, so let's make up data for Joystick

                // Buttons first!

                uint8_t hasPress = false;
                uint8_t tmp;
                usbcon.button(XBOX_A, tmp=psx.buttonPressed(PSB_CROSS)); hasPress |= tmp;
                usbcon.button(XBOX_B, tmp=psx.buttonPressed(PSB_CIRCLE)); hasPress |= tmp;
                usbcon.button(XBOX_X, tmp=psx.buttonPressed(PSB_SQUARE)); hasPress |= tmp;
                usbcon.button(XBOX_Y, tmp=psx.buttonPressed(PSB_TRIANGLE)); hasPress |= tmp;
                usbcon.button(XBOX_DUP, tmp=psx.buttonPressed(PSB_PAD_UP)); hasPress |= tmp;
                usbcon.button(XBOX_DDOWN, tmp=psx.buttonPressed(PSB_PAD_DOWN)); hasPress |= tmp;
                usbcon.button(XBOX_DLEFT, tmp=psx.buttonPressed(PSB_PAD_LEFT)); hasPress |= tmp;
                usbcon.button(XBOX_DRIGHT, tmp=psx.buttonPressed(PSB_PAD_RIGHT)); hasPress |= tmp;
                usbcon.button(XBOX_START, tmp=psx.buttonPressed(PSB_START)); hasPress |= tmp;
                usbcon.button(XBOX_BACK, tmp=psx.buttonPressed(PSB_SELECT)); hasPress |= tmp;
                usbcon.button(XBOX_L3, tmp=psx.buttonPressed(PSB_L3)); hasPress |= tmp;
                usbcon.button(XBOX_R3, tmp=psx.buttonPressed(PSB_R3)); hasPress |= tmp;
                usbcon.button(XBOX_LSHOULDER, tmp=psx.buttonPressed(PSB_L1)); hasPress |= tmp;
                usbcon.button(XBOX_RSHOULDER, tmp=psx.buttonPressed(PSB_R1)); hasPress |= tmp;


                usbcon.sliderLeft(psx.getAnalogButton(PSAB_L2) > 2 ? 255 : 0);
                usbcon.sliderRight(psx.getAnalogButton(PSAB_R2) > 2 ? 255 : 0);

                if (hasPress)
                    GPIOC->regs->ODR &= ~(1 << 13);
                else
                    GPIOC->regs->ODR |= (1 << 13);


                byte lx, ly;
                if (psx.getLeftAnalog(lx, ly)) {
                    ly = ANALOG_MAX_VALUE - ly;

                    double_t x = ((1.0*lx)-128.0) / 127.0;
                    double_t y = ((1.0*ly)-128.0) / 127.0;

                    double_t a = atan2(y,x);
                    double_t d = sqrt(x*x + y*y);

                    if (d > 1.0) d = 1.0;

                    double_t cx = 32767.0 * d * cos(a);
                    double_t cy = 32767.0 * d * sin(a);

                    if (abs(cx) < 2048.0 && abs(cy) < 2048.0) {
                        cx = 0.0;
                        cy = 0.0;
                    }

                    usbcon.X((int16_t)cx);
                    usbcon.Y((int16_t)cy);
                }

                byte rx, ry;
                if (psx.getRightAnalog(rx, ry)) {
                    ry = ANALOG_MAX_VALUE - ry;

                    double_t x = ((1.0*rx)-128.0) / 127.0;
                    double_t y = ((1.0*ry)-128.0) / 127.0;

                    double_t a = atan2(y,x);
                    double_t d = sqrt(x*x + y*y);

                    if (d > 1.0) d = 1.0;

                    double_t cx = 32767.0 * d * cos(a);
                    double_t cy = 32767.0 * d * sin(a);

                    if (abs(cx) < 2048.0 && abs(cy) < 2048.0) {
                        cx = 0.0;
                        cy = 0.0;
                    }

                    usbcon.XRight((int16_t)cx);
                    usbcon.YRight((int16_t)cy);
                }

                usbcon.send();
            }
        }
    }
}