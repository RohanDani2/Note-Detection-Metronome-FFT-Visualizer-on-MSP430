#include "sound.h"
#include "Timer32_HAL.h"

#define SYSTEMCLOCK 48000000
extern HWTimer_t timer0;

void InitSound()
{
    GPIO_setAsPeripheralModuleFunctionOutputPin(
    GPIO_PORT_P2,
                                                GPIO_PIN7,
                                                GPIO_PRIMARY_MODULE_FUNCTION);
}

//TODO: Write a comment for this function
void ConfigurePWM(Timer_A_PWMConfig *pwmConfig_p, basic_note_t note)
{

    double toneFreq[49] = { 0.00,   // silent
            16.35,  // c0
            18.35,  // d0
            20.60,  // e0
            21.83,  // f0
            24.50,  // g0
            27.50,  // a0
            30.87,  // b0
            21.22,  // f0 flat
            29.14,  // b0 flat
            32.70,  // c1
            36.71,  // d1
            41.20,  // e1
            43.65,  // f1
            49.00,  // g1
            55.00,  // a1
            61.74,  // b1
            65.41,  // c2
            73.42,  // d2
            82.41,  // e2
            87.31,  // f2
            98.00,  // g2
            110.00, // a2
            123.47, // b2
            130.81, // c3
            146.83, // d3
            164.81, // e3
            174.61, // f3
            196.00, // g3
            220.00, // a3
            246.94, // b3
            261.63, // C4
            293.66, // D4
            329.63, // E4
            349.63, // F4
            392.00, // G4
            440.00, // A4
            493.88, // B4
            523.25, // C5
            587.33, // D5
            659.25, // E5
            698.46, // F5
            783.99, // G5
            880.00, // A5
            987.77, // B5
            739.99, // F5 sharp
            369.99, // F4 sharp
            369.99, // g4 flat
            1046.50 // C6
            };

    // TODO: The Timer_A register connected to the speaker. Currently, this is set to TIMER_A_CAPTURECOMPARE_REGISTER_0, which could or could not be the right answer.
    // You need to find this information from the combination of the BoosterPack and Launchpad user guides.
    // The printed quick guide can be the best place to find this information. If you hold the ctrl key and click on TIMER_A_CAPTURECOMPARE_REGISTER_0, you will see
    // the other options. (Refer to HW6_RGB and how you drove RGB colors)
    pwmConfig_p->compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4;

    // TODO: See if this mode suits you. If not, change it to something else. Ctrl+click can show you other modes. Refer to the user guide for details.
    pwmConfig_p->compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;

    // The clock source used for the Timer_A. This is not the frequency of the clock. It is a flag that tells what clock source is used.
    // We are using system clock and the below value should not be changed.
    pwmConfig_p->clockSource = TIMER_A_CLOCKSOURCE_SMCLK;

    // TODO: The clock divider, you need to choose this flag. Currently this value is set to 1. If you hold ctrl key and click on the TIMER_A_CLOCKSOURCE_DIVIDER_1 macro
    // you will see the other options, there are multiple options between 1 and 64 (not all numbers are possible)
    // Remember this is a flag that later decides the value of prescaler (clock divider) and not the clock divider value itself
    pwmConfig_p->clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_16;

    // TODO: Choose the period of the PWM in terms of number of counter cycles. Currently, it is set to SYSTEMCLOCK which is 48M
    // You need to generate a different PWM for each note, pay attention to the clock divider you chose earlier.
    // Hint: you need to incorporate toneFreq to complete this part.
    int i;

    pwmConfig_p->timerPeriod = SYSTEMCLOCK
            / (toneFreq[note] * TIMER_A_CLOCKSOURCE_DIVIDER_16);

    // TODO: This is the duty cycle in terms of counter cycles. Currently it is set equal to pwmConfig.timerPeriod, which means 100% duty cycle.
    // Pay attention to the duty cycle of a musical note. Multiply the current value with a fraction to achieve that.
    pwmConfig_p->dutyCycle = pwmConfig_p->timerPeriod * 0.5;

}
