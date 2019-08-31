/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//****************************************************************************
//
// main.c - MSP-EXP432P401R + Educational Boosterpack MkII - Microphone FFT
//
//          CMSIS DSP Software Library is used to perform 512-point FFT on
//          the audio samples collected with MSP432's ADC14 from the Education
//          Boosterpack's onboard microhpone. The resulting frequency bin data
//          is displayed on the BoosterPack's 128x128 LCD.
//
//****************************************************************************
#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "HAL_I2C.h"
#include "HAL_OPT3001.h"
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include <arm_math.h>
#include <arm_const_structs.h>
#include "ButtonLED_HAL.h"
#include "Timer32_HAL.h"
#include "graphics_hal.h"
#include "sound.h"

#define TEST_LENGTH_SAMPLES 2048
#define SAMPLE_LENGTH 2048

typedef struct
{
    char key[3];
    int value;
} note_frequency[1];

extern HWTimer_t timer0, timer1;
#define PRESSED 0 // When a button is pressed, it is grounded (logic 0)
#define RELEASED 1
#define BOOSTER_BUTTON_1 BIT1 //5.1 //S1
#define BOOSTER_BUTTON_2 BIT5 //3.5 //S2
/* ------------------------------------------------------------------
 * Global variables for FFT Bin Example
 * ------------------------------------------------------------------- */
uint32_t fftSize = SAMPLE_LENGTH;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;
volatile arm_status status;

/* Graphic library context */
Graphics_Context g_sContext;

#define SMCLK_FREQUENCY     48000000
#define SAMPLE_FREQUENCY    8000

/* DMA Control Table */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(MSP_EXP432P401RLP_DMAControlTable, 1024)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=1024
#elif defined(__GNUC__)
__attribute__ ((aligned (1024)))
#elif defined(__CC_ARM)
__align(1024)
#endif
static DMA_ControlTable MSP_EXP432P401RLP_DMAControlTable[32];

/* FFT data/processing buffers*/
float hann[SAMPLE_LENGTH];
int16_t data_array1[SAMPLE_LENGTH];
int16_t data_array2[SAMPLE_LENGTH];
int16_t data_input[SAMPLE_LENGTH * 2];
int16_t data_output[SAMPLE_LENGTH];

volatile int switch_data = 0;

uint32_t color = 0;

/* Timer_A PWM Configuration Parameter */
Timer_A_PWMConfig pwmConfig = {
TIMER_A_CLOCKSOURCE_SMCLK,
                                TIMER_A_CLOCKSOURCE_DIVIDER_1, (SMCLK_FREQUENCY
                                        / SAMPLE_FREQUENCY),
                                TIMER_A_CAPTURECOMPARE_REGISTER_1,
                                TIMER_A_OUTPUTMODE_SET_RESET,
                                (SMCLK_FREQUENCY / SAMPLE_FREQUENCY) / 2 };

int main(void)
{
    initialize();
    /* Draw Title, x-axis, gradation & labels */
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    Graphics_clearDisplay(&g_sContext);
    Graphics_drawLineH(&g_sContext, 0, 127, 115);
    Graphics_drawLineV(&g_sContext, 0, 115, 117);
    Graphics_drawLineV(&g_sContext, 16, 115, 116);
    Graphics_drawLineV(&g_sContext, 31, 115, 117);
    Graphics_drawLineV(&g_sContext, 32, 115, 117);
    Graphics_drawLineV(&g_sContext, 48, 115, 116);
    Graphics_drawLineV(&g_sContext, 63, 115, 117);
    Graphics_drawLineV(&g_sContext, 64, 115, 117);
    Graphics_drawLineV(&g_sContext, 80, 115, 116);
    Graphics_drawLineV(&g_sContext, 95, 115, 117);
    Graphics_drawLineV(&g_sContext, 96, 115, 117);
    Graphics_drawLineV(&g_sContext, 112, 115, 116);
    Graphics_drawLineV(&g_sContext, 127, 115, 117);

    Graphics_drawStringCentered(&g_sContext, (int8_t *) "2048-Point FFT",
    AUTO_STRING_LENGTH,
                                64, 6,
                                OPAQUE_TEXT);
    Graphics_drawStringCentered(&g_sContext, (int8_t *) "0",
    AUTO_STRING_LENGTH,
                                4, 122,
                                OPAQUE_TEXT);
    Graphics_drawStringCentered(&g_sContext, (int8_t *) "1",
    AUTO_STRING_LENGTH,
                                32, 122,
                                OPAQUE_TEXT);
    Graphics_drawStringCentered(&g_sContext, (int8_t *) "2",
    AUTO_STRING_LENGTH,
                                64, 122,
                                OPAQUE_TEXT);
    Graphics_drawStringCentered(&g_sContext, (int8_t *) "3",
    AUTO_STRING_LENGTH,
                                96, 122,
                                OPAQUE_TEXT);
    Graphics_drawStringCentered(&g_sContext, (int8_t *) "4",
    AUTO_STRING_LENGTH,
                                125, 122,
                                OPAQUE_TEXT);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_drawStringCentered(&g_sContext, (int8_t *) "kHz",
    AUTO_STRING_LENGTH,
                                112, 122,
                                OPAQUE_TEXT);

    note_frequency noteFreq[60] = { { "A1", 55 }, { "A2", 110 }, { "A3", 220 },
                                    { "A4", 440 }, { "A5", 880 },
                                    { "B1", 61.73 }, { "B2", 123.47 },
                                    { "B3", 246.94 }, { "B4", 493.88 },
                                    { "B5", 987.76 }, { "C1", 32.70 },
                                    { "C2", 65.40 }, { "C3", 130.81 },
                                    { "C4", 261.62 }, { "C5", 523.25 },
                                    { "D1", 36.70 }, { "D2", 73.41 },
                                    { "D3", 146.83 }, { "D4", 293.66 },
                                    { "D5", 587.33 }, { "E1", 41.20 },
                                    { "E2", 82.40 }, { "E3", 110 }, { "E4",
                                                                      329.62 },
                                    { "E5", 659.25 }, { "F1", 43.65 },
                                    { "F2", 87.30 }, { "F3", 174.61 },
                                    { "F4", 349.22 }, { "F5", 698.45 },
                                    { "G1", 48.99 }, { "G2", 97.99 },
                                    { "G3", 195.99 }, { "G4", 391.99 },
                                    { "G5", 783.99 }, { "A#1", 58.27 },
                                    { "A#2", 116.54 }, { "A#3", 233.08 }, {
                                            "A#4", 466.16 },
                                    { "A#5", 932.32 }, { "C#1", 34.64 }, {
                                            "C#2", 69.29 },
                                    { "C#3", 138.59 }, { "C#4", 277.18 }, {
                                            "C#5", 554.36 },
                                    { "D#1", 38.89 }, { "D#2", 77.78 },
                                    { "D#3", 155.56 }, { "D#4", 311.12 }, {
                                            "D#5", 622.25 },
                                    { "F#1", 46.24 }, { "F#2", 92.49 },
                                    { "F#3", 184.99 }, { "F#4", 369.99 }, {
                                            "F#5", 739.98 },
                                    { "G#1", 51.91 }, { "G#2", 103.82 }, {
                                            "G#3", 207.65 },
                                    { "G#4", 415.30 }, { "G#5", 783.99 } };

    // Initialize Hann Window
    int n;
    for (n = 0; n < SAMPLE_LENGTH; n++)
    {
        hann[n] = 0.5f - 0.5f * cosf((2 * PI * n) / (SAMPLE_LENGTH - 1));
    }

    /* Configuring Timer_A to have a period of approximately 500ms and
     * an initial duty cycle of 10% of that (3200 ticks)  */
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);

    /* Initializing ADC (MCLK/1/1) */
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, 0);

    ADC14_setSampleHoldTrigger(ADC_TRIGGER_SOURCE1, false);

    /* Configuring GPIOs (4.3 A10) */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN3,
    GPIO_TERTIARY_MODULE_FUNCTION);

    /* Configuring ADC Memory */
    ADC14_configureSingleSampleMode(ADC_MEM0, true);
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS,
    ADC_INPUT_A10,
                                    false);

    /* Set ADC result format to signed binary */
    ADC14_setResultFormat(ADC_SIGNED_BINARY);

    /* Configuring DMA module */
    DMA_enableModule();
    DMA_setControlBase(MSP_EXP432P401RLP_DMAControlTable);

    DMA_disableChannelAttribute(DMA_CH7_ADC14,
    UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
    UDMA_ATTR_HIGH_PRIORITY |
    UDMA_ATTR_REQMASK);

    /* Setting Control Indexes. In this case we will set the source of the
     * DMA transfer to ADC14 Memory 0
     *  and the destination to the
     * destination data array. */
    MAP_DMA_setChannelControl(
    UDMA_PRI_SELECT | DMA_CH7_ADC14,
                              UDMA_SIZE_16 | UDMA_SRC_INC_NONE |
                              UDMA_DST_INC_16 | UDMA_ARB_1);
    MAP_DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CH7_ADC14,
    UDMA_MODE_PINGPONG,
                               (void*) &ADC14->MEM[0], data_array1,
                               SAMPLE_LENGTH);

    MAP_DMA_setChannelControl(
    UDMA_ALT_SELECT | DMA_CH7_ADC14,
                              UDMA_SIZE_16 | UDMA_SRC_INC_NONE |
                              UDMA_DST_INC_16 | UDMA_ARB_1);
    MAP_DMA_setChannelTransfer(UDMA_ALT_SELECT | DMA_CH7_ADC14,
    UDMA_MODE_PINGPONG,
                               (void*) &ADC14->MEM[0], data_array2,
                               SAMPLE_LENGTH);

    /* Assigning/Enabling Interrupts */
    MAP_DMA_assignInterrupt(DMA_INT1, 7);
    MAP_Interrupt_enableInterrupt(INT_DMA_INT1);
    MAP_DMA_assignChannel(DMA_CH7_ADC14);
    MAP_DMA_clearInterruptFlag(7);
    MAP_Interrupt_enableMaster();

    /* Now that the DMA is primed and setup, enabling the channels. The ADC14
     * hardware should take over and transfer/receive all bytes */
    MAP_DMA_enableChannel(7);
    MAP_ADC14_enableConversion();

    button_t LauchpadLeftButton;
    initButton(&LauchpadLeftButton, GPIO_PORT_P1, GPIO_PIN1, &timer0);

    button_t LauchpadRightButton;
    initButton(&LauchpadRightButton, GPIO_PORT_P1, GPIO_PIN4, &timer0);

    button_t S1_Button;
    initButton(&S1_Button, GPIO_PORT_P5, GPIO_PIN1, &timer0);

    button_t S2_Button;
    initButton(&S2_Button, GPIO_PORT_P3, GPIO_PIN5, &timer0);

    bool fftScreen = true;
    bool fftPrint = false;
    bool metronomeScreen = false;
    bool metronomePrint = false;
    bool noteDetectScreen = false;
    bool notePrint = false;
    float lux;

    initHWTimer0();
    initHWTimer1();
    InitSound();

    /* Initialize I2C communication */
    Init_I2C_GPIO();
    I2C_init();

    /* Initialize OPT3001 digital ambient light sensor */
    OPT3001_init();
    __delay_cycles(100000);

    while (1)
    {
        static bool clearMinLuxDone = false;
        static bool clearMaxLuxDone = false;
        static bool clearFFTDone = false;
        static bool clearRevFFTDone = false;
        static bool noteDetectDone = false;
        static bool noteDetectRevDone = false;
        static int prevState = 0;
        /* Obtain lux value from OPT3001 */
        lux = OPT3001_getLux();
        bool leftButtonPushed = ButtonPushed(&LauchpadLeftButton);
        bool rightButtonPushed = ButtonPushed(&LauchpadRightButton);
        bool s1ButtonPushed = ButtonPushed(&S1_Button);
        bool s2ButtonPushed = ButtonPushed(&S2_Button);
        if (leftButtonPushed)
        { //the FSM for the three different screens
            if (fftScreen)
            {
                fftScreen = false;
                fftPrint = false;
                metronomeScreen = true;
            }
            else if (metronomeScreen)
            {
                metronomeScreen = false;
                metronomePrint = false;
                noteDetectScreen = true;
            }
            else if (noteDetectScreen)
            {
                noteDetectScreen = false;
                notePrint = false;
                fftScreen = true;
            }
        }
        else if (rightButtonPushed)
        { //the FSM for the three different screens
            if (fftScreen)
            {
                fftScreen = false;
                fftPrint = false;
                noteDetectScreen = true;
            }
            else if (noteDetectScreen)
            {
                noteDetectScreen = false;
                notePrint = false;
                metronomeScreen = true;
            }
            else if (metronomeScreen)
            {
                metronomeScreen = false;
                metronomePrint = false;
                fftScreen = true;
            }
        }

        if (fftScreen)
        {
            clearMaxLuxDone = false;
            clearMinLuxDone = false;
            noteDetectDone = false;
            noteDetectRevDone = false;
            if (!fftPrint)
            {
                Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
                LCDClearDisplay(&g_sContext);
                WriteFFT(&g_sContext);
                fftPrint = true;
            }

            if (lux < 7)
            {
                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
                Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
                if (!clearRevFFTDone)
                {
                    clearRevFFTDone = true;
                    clearFFTDone = false;
                    LCDClearDisplay(&g_sContext);
                }
                DrawReverseFFT(&g_sContext);
            }
            else
            {
                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
                Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
                if (!clearFFTDone)
                {
                    LCDClearDisplay(&g_sContext);
                    clearFFTDone = true;
                    clearRevFFTDone = false;
                }
                WriteFFT(&g_sContext);
            }

            MAP_PCM_gotoLPM0();

            int i = 0;

            /* Computer real FFT using the completed data buffer */
            if (switch_data & 1)
            {
                for (i = 0; i < 2048; i++)
                {
                    data_array1[i] = (int16_t) (hann[i] * data_array1[i]);
                }
                arm_rfft_instance_q15 instance;
                status = arm_rfft_init_q15(&instance, fftSize, ifftFlag,
                                           doBitReverse);

                arm_rfft_q15(&instance, data_array1, data_input);
            }
            else
            {
                for (i = 0; i < 2048; i++)
                {
                    data_array2[i] = (int16_t) (hann[i] * data_array2[i]);
                }
                arm_rfft_instance_q15 instance;
                status = arm_rfft_init_q15(&instance, fftSize, ifftFlag,
                                           doBitReverse);

                arm_rfft_q15(&instance, data_array2, data_input);
            }

            /* Calculate magnitude of FFT complex output */
            for (i = 0; i < 4096; i += 2)
            {
                data_output[i / 2] = (int32_t) (sqrtf(
                        (data_input[i] * data_input[i])
                                + (data_input[i + 1] * data_input[i + 1])));
            }

            q15_t maxValue;
            uint32_t maxIndex = 0;

            arm_max_q15(data_output, fftSize, &maxValue, &maxIndex);

            if (maxIndex <= 64)
            {
                color = ((uint32_t) (0xFF * (maxIndex / 64.0f)) << 8) + 0xFF;
            }
            else if (maxIndex <= 128)
            {
                color = (0xFF - (uint32_t) (0xFF * ((maxIndex - 64) / 64.0f)))
                        + 0xFF00;
            }
            else if (maxIndex <= 192)
            {
                color = ((uint32_t) (0xFF * ((maxIndex - 128) / 64.0f)) << 16)
                        + 0xFF00;
            }
            else
            {
                color = ((0xFF - (uint32_t) (0xFF * ((maxIndex - 192) / 64.0f)))
                        << 8) + 0xFF0000;
            }

            /* Draw frequency bin graph */
            for (i = 0; i < 256; i += 2)
            {
                int x = min(100,
                            (int )((data_output[i] + data_output[i + 1]) / 8));

                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
                Graphics_drawLineV(&g_sContext, i / 2, 114 - x, 14);
                Graphics_setForegroundColor(&g_sContext, color);
                Graphics_drawLineV(&g_sContext, i / 2, 114, 114 - x);
            }
        }
        else if (noteDetectScreen)
        {
            clearMaxLuxDone = false;
            clearMinLuxDone = false;
            clearFFTDone = false;
            clearRevFFTDone = false;
            static int j = 0;
            static int num = 0;

            if (!notePrint)
            {
                Graphics_clearDisplay(&g_sContext);
                int8_t noteString[15] = "NOTE DETECTION";
                Graphics_drawString(&g_sContext, noteString, -1, 30, 3, true);
                notePrint = true;
            }
            int8_t noteString[15] = "NOTE DETECTION";
            if (lux < 7)
            {
                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLUE);
                Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
                if (!noteDetectRevDone)
                {
                    LCDClearDisplay(&g_sContext);
                    noteDetectRevDone = true;
                    noteDetectDone = false;
                }
                Graphics_drawString(&g_sContext, noteString, -1, 30, 3, true);
                Graphics_drawString(&g_sContext, noteFreq[j]->key, -1, 45, 25,
                                    true);
            }
            else
            {
                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLUE);
                Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
                if (!noteDetectDone)
                {
                    LCDClearDisplay(&g_sContext);
                    noteDetectDone = true;
                    noteDetectRevDone = false;
                }
                Graphics_drawString(&g_sContext, noteString, -1, 30, 3, true);
                Graphics_drawString(&g_sContext, noteFreq[j]->key, -1, 45, 25,
                                    true);
            }

            for (j = 0; j < 59; j++)
            {
                if ((data_output[num] >= noteFreq[j]->value - 2
                        && data_output[num] <= noteFreq[j]->value + 2))
                {

                    Graphics_drawString(&g_sContext, noteFreq[j]->key, -1, 45,
                                        25, true);
                }
                if (j == 59)
                {
                    j = 0;
                    num = 0;
                }
            }
            num++;
        }
        else if (metronomeScreen)
        {
            clearFFTDone = false;
            clearRevFFTDone = false;
            noteDetectDone = false;
            noteDetectRevDone = false;
            static int bpmValue = 100;
            char string[3];
            if (!metronomePrint)
            {
                DrawBPM(&g_sContext, bpmValue);
                metronomePrint = true;
            }

            if ((s1ButtonPushed) && (bpmValue < 200))
            {
                bpmValue = bpmValue + 10;
                ChangeBPM(&g_sContext, bpmValue, lux);

            }
            else if ((s2ButtonPushed) && (bpmValue > 10))
            {
                bpmValue = bpmValue - 10;
                ChangeBPM(&g_sContext, bpmValue, lux);
            }

            if (lux < 7)
            {
                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
                Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_BLUE);
                if (!clearMinLuxDone)
                {
                    LCDClearDisplay(&g_sContext);
                    clearMinLuxDone = true;
                    clearMaxLuxDone = false;
                }
                int8_t bpmString[5] = "BPM:";
                string[0] = (bpmValue / 100) + '0';
                string[1] = ((bpmValue % 100) / 10) + '0';
                string[2] = (bpmValue % 10) + '0';
                Graphics_drawString(&g_sContext, (int8_t*) string, -1, 64, 5,
                true);
                Graphics_drawString(&g_sContext, bpmString, -1, 40, 5, true);
            }
            else
            {
                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLUE);
                Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
                if (!clearMaxLuxDone)
                {
                    LCDClearDisplay(&g_sContext);
                    clearMaxLuxDone = true;
                    clearMinLuxDone = false;
                }
                int8_t bpmString[5] = "BPM:";
                string[0] = (bpmValue / 100) + '0';
                string[1] = ((bpmValue % 100) / 10) + '0';
                string[2] = (bpmValue % 10) + '0';
                Graphics_drawString(&g_sContext, (int8_t*) string, -1, 64, 5,
                true);
                Graphics_drawString(&g_sContext, bpmString, -1, 40, 5, true);
            }

            Graphics_fillCircle(&g_sContext, 64, 64, 16);
            ClearBPMCircle(&g_sContext, lux);
            if (PlayNote_nonblockingBPM(bpmValue))
            {
                MakeBPMCircle(&g_sContext, lux);
                PlayNote_blockingBPM(bpmValue);
            }
        }
    }
}

/* Completion interrupt for ADC14 MEM0 */
void DMA_INT1_IRQHandler(void)
{
    /* Switch between primary and alternate bufferes with DMA's PingPong mode */
    if (DMA_getChannelAttribute(7) & UDMA_ATTR_ALTSELECT)
    {
        DMA_setChannelControl(
        UDMA_PRI_SELECT | DMA_CH7_ADC14,
                              UDMA_SIZE_16 | UDMA_SRC_INC_NONE |
                              UDMA_DST_INC_16 | UDMA_ARB_1);
        DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CH7_ADC14,
        UDMA_MODE_PINGPONG,
                               (void*) &ADC14->MEM[0], data_array1,
                               SAMPLE_LENGTH);
        switch_data = 1;
    }
    else
    {
        DMA_setChannelControl(
        UDMA_ALT_SELECT | DMA_CH7_ADC14,
                              UDMA_SIZE_16 | UDMA_SRC_INC_NONE |
                              UDMA_DST_INC_16 | UDMA_ARB_1);
        DMA_setChannelTransfer(UDMA_ALT_SELECT | DMA_CH7_ADC14,
        UDMA_MODE_PINGPONG,
                               (void*) &ADC14->MEM[0], data_array2,
                               SAMPLE_LENGTH);
        switch_data = 0;
    }
}

bool PlayNote_nonblockingBPM(int bpmValue)
{
    static Timer_A_PWMConfig pwmConfig;
    ConfigurePWM(&pwmConfig, note_silent);

    static OneShotSWTimer_t noteLength;

    static NoteStatus noteStatus = notPlaying;
    switch (noteStatus)
    {
    case playing:
        if (OneShotSWTimerExpired(&noteLength))
        {
            Timer_A_stopTimer(TIMER_A0_BASE);
            noteStatus = notPlaying;
            return true;
        }
        else
        {
            return false;
        }
    case notPlaying:
        InitOneShotSWTimer(&noteLength, &timer0, bpmValue * 1000);
        StartOneShotSWTimer(&noteLength);
        Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
        noteStatus = playing;
        return false;
    }
}

void LCDClearDisplay(Graphics_Context *g_sContext_p)
{
    Graphics_clearDisplay(g_sContext_p);
}

void PlayNote_blockingBPM(int bpmValue)
{
    // The struct that holds all the info for PWM driving the buzzer
    Timer_A_PWMConfig pwmConfig;
    ConfigurePWM(&pwmConfig, note_d5);

    //  the one shot timer for playing the note
    OneShotSWTimer_t noteLength;
    InitOneShotSWTimer(&noteLength, &timer0, bpmValue * 1000);
    StartOneShotSWTimer(&noteLength);

    // Start driving the pwm to generate the sound
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);

    // wait until the duration of the note is over, i.e. the timer is expired
    while (!(OneShotSWTimerExpired(&noteLength)))
        ;

    // We stop the PWM
    Timer_A_stopTimer(TIMER_A0_BASE);
}

void initialize()
{
    /* Halting WDT and disabling master interrupts */
    MAP_WDT_A_holdTimer();
    MAP_Interrupt_disableMaster();

    /* Set the core voltage level to VCORE1 */
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);

    /* Set 2 flash wait states for Flash bank 0 and 1*/
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);

    /* Initializes Clock System */
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /* Initializes display */
    Crystalfontz128x128_Init();

    /* Set default screen orientation */
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);

    /* Initializes graphics context */
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128,
                         &g_sCrystalfontz128x128_funcs);

}
