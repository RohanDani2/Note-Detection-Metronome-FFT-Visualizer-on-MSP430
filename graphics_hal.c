#include "graphics_hal.h"

void WriteFFT(Graphics_Context *g_sContext_p)
{
    Graphics_drawLineH(g_sContext_p, 0, 127, 115);
    Graphics_drawLineV(g_sContext_p, 0, 115, 117);
    Graphics_drawLineV(g_sContext_p, 16, 115, 116);
    Graphics_drawLineV(g_sContext_p, 31, 115, 117);
    Graphics_drawLineV(g_sContext_p, 32, 115, 117);
    Graphics_drawLineV(g_sContext_p, 48, 115, 116);
    Graphics_drawLineV(g_sContext_p, 63, 115, 117);
    Graphics_drawLineV(g_sContext_p, 64, 115, 117);
    Graphics_drawLineV(g_sContext_p, 80, 115, 116);
    Graphics_drawLineV(g_sContext_p, 95, 115, 117);
    Graphics_drawLineV(g_sContext_p, 96, 115, 117);
    Graphics_drawLineV(g_sContext_p, 112, 115, 116);
    Graphics_drawLineV(g_sContext_p, 127, 115, 117);

    Graphics_drawStringCentered(g_sContext_p,
                                 (int8_t *)"2048-Point FFT",
                                 AUTO_STRING_LENGTH,
                                 64,
                                 6,
                                 OPAQUE_TEXT);
     Graphics_drawStringCentered(g_sContext_p,
                                 (int8_t *)"0",
                                 AUTO_STRING_LENGTH,
                                 4,
                                 122,
                                 OPAQUE_TEXT);
     Graphics_drawStringCentered(g_sContext_p,
                                 (int8_t *)"1",
                                 AUTO_STRING_LENGTH,
                                 32,
                                 122,
                                 OPAQUE_TEXT);
     Graphics_drawStringCentered(g_sContext_p,
                                 (int8_t *)"2",
                                 AUTO_STRING_LENGTH,
                                 64,
                                 122,
                                 OPAQUE_TEXT);
     Graphics_drawStringCentered(g_sContext_p,
                                 (int8_t *)"3",
                                 AUTO_STRING_LENGTH,
                                 96,
                                 122,
                                 OPAQUE_TEXT);
     Graphics_drawStringCentered(g_sContext_p,
                                 (int8_t *)"4",
                                 AUTO_STRING_LENGTH,
                                 125,
                                 122,
                                 OPAQUE_TEXT);
    Graphics_drawStringCentered(g_sContext_p,
                                (int8_t *)"kHz",
                                AUTO_STRING_LENGTH,
                                112,
                                122,
                                OPAQUE_TEXT);
}

void DrawReverseFFT(Graphics_Context *g_sContext_p)
{
    Graphics_drawLineH(g_sContext_p, 0, 127, 115);
    Graphics_drawLineV(g_sContext_p, 0, 115, 117);
    Graphics_drawLineV(g_sContext_p, 16, 115, 116);
    Graphics_drawLineV(g_sContext_p, 31, 115, 117);
    Graphics_drawLineV(g_sContext_p, 32, 115, 117);
    Graphics_drawLineV(g_sContext_p, 48, 115, 116);
    Graphics_drawLineV(g_sContext_p, 63, 115, 117);
    Graphics_drawLineV(g_sContext_p, 64, 115, 117);
    Graphics_drawLineV(g_sContext_p, 80, 115, 116);
    Graphics_drawLineV(g_sContext_p, 95, 115, 117);
    Graphics_drawLineV(g_sContext_p, 96, 115, 117);
    Graphics_drawLineV(g_sContext_p, 112, 115, 116);
    Graphics_drawLineV(g_sContext_p, 127, 115, 117);

    Graphics_drawStringCentered(g_sContext_p,
                                (int8_t *)"2048-Point FFT",
                                AUTO_STRING_LENGTH,
                                64,
                                6,
                                OPAQUE_TEXT);
    Graphics_drawStringCentered(g_sContext_p,
                                (int8_t *)"0",
                                AUTO_STRING_LENGTH,
                                4,
                                122,
                                OPAQUE_TEXT);
    Graphics_drawStringCentered(g_sContext_p,
                                (int8_t *)"1",
                                AUTO_STRING_LENGTH,
                                32,
                                122,
                                OPAQUE_TEXT);
    Graphics_drawStringCentered(g_sContext_p,
                                (int8_t *)"2",
                                AUTO_STRING_LENGTH,
                                64,
                                122,
                                OPAQUE_TEXT);
    Graphics_drawStringCentered(g_sContext_p,
                                (int8_t *)"3",
                                AUTO_STRING_LENGTH,
                                96,
                                122,
                                OPAQUE_TEXT);
    Graphics_drawStringCentered(g_sContext_p,
                                (int8_t *)"4",
                                AUTO_STRING_LENGTH,
                                125,
                                122,
                                OPAQUE_TEXT);
    Graphics_drawStringCentered(g_sContext_p,
                                (int8_t *)"kHz",
                                AUTO_STRING_LENGTH,
                                112,
                                122,
                                OPAQUE_TEXT);
}

void MakeBPMCircle(Graphics_Context *g_sContext_p, float lux)
{
    if (lux < 7)
    {
        Graphics_setForegroundColor(g_sContext_p, GRAPHICS_COLOR_WHITE);
        Graphics_drawCircle(g_sContext_p, 64, 64, 18);
        Graphics_drawCircle(g_sContext_p, 64, 64, 20);
    }
    else
    {
        Graphics_setForegroundColor(g_sContext_p, GRAPHICS_COLOR_BLUE);
        Graphics_drawCircle(g_sContext_p, 64, 64, 18);
        Graphics_drawCircle(g_sContext_p, 64, 64, 20);
    }
}

void ClearBPMCircle(Graphics_Context *g_sContext_p, float lux)
{
    if (lux < 7)
    {
        Graphics_setForegroundColor(g_sContext_p, GRAPHICS_COLOR_BLUE);
        Graphics_drawCircle(g_sContext_p, 64, 64, 18);
        Graphics_drawCircle(g_sContext_p, 64, 64, 20);
    }
    else
    {
        Graphics_setForegroundColor(g_sContext_p, GRAPHICS_COLOR_WHITE);
        Graphics_drawCircle(g_sContext_p, 64, 64, 18);
        Graphics_drawCircle(g_sContext_p, 64, 64, 20);
    }
}

void ChangeBPM(Graphics_Context *g_sContext_p, int bpmValue, float lux)
{
    int8_t clearNum[6] = "     ";
    if (lux < 7)
    {
        Graphics_setBackgroundColor(g_sContext_p, GRAPHICS_COLOR_BLUE);
        Graphics_setForegroundColor(g_sContext_p, GRAPHICS_COLOR_WHITE);
        int8_t bpmString[5] = "BPM:";
        Graphics_drawString(g_sContext_p, bpmString, -1, 40, 5, true);
        Graphics_drawString(g_sContext_p, clearNum, -1, 64, 5, true);
    }
    else
    {
        Graphics_setBackgroundColor(g_sContext_p, GRAPHICS_COLOR_WHITE);
        Graphics_setForegroundColor(g_sContext_p, GRAPHICS_COLOR_BLUE);
        int8_t bpmString[5] = "BPM:";
        Graphics_drawString(g_sContext_p, bpmString, -1, 40, 5, true);
        Graphics_drawString(g_sContext_p, clearNum, -1, 64, 5, true);
    }

    if (bpmValue >= 100)
    {
        char string[3];
        string[0] = (bpmValue / 100) + '0';
        string[1] = ((bpmValue % 100) / 10) + '0';
        string[2] = (bpmValue % 10) + '0';
        Graphics_drawString(g_sContext_p, (int8_t*) string, -1, 64, 5, true);
        if (lux < 7)
        {
            Graphics_setForegroundColor(g_sContext_p, GRAPHICS_COLOR_BLUE);
        }
        else
        {
            Graphics_setForegroundColor(g_sContext_p, GRAPHICS_COLOR_WHITE);
        }
        int8_t clearVal[7] = "      ";
        Graphics_drawString(g_sContext_p, clearVal, -1, 84, 5, true);
    }
    else
    {
        if (lux < 7)
        {
            Graphics_setForegroundColor(g_sContext_p, GRAPHICS_COLOR_WHITE);
        }
        else
        {
            Graphics_setForegroundColor(g_sContext_p, GRAPHICS_COLOR_BLUE);
        }
        char string[2];
        string[0] = ((bpmValue % 10) / 10) + '0';
        string[1] = ((bpmValue % 1) / 1) + '0';
        Graphics_drawString(g_sContext_p, (int8_t*) string, -1, 64, 5, true);
        if (lux < 7)
        {
            Graphics_setForegroundColor(g_sContext_p, GRAPHICS_COLOR_BLUE);
        }
        else
        {
            Graphics_setForegroundColor(g_sContext_p, GRAPHICS_COLOR_WHITE);
        }
        int8_t clearVal[7] = "      ";
        Graphics_drawString(g_sContext_p, clearVal, -1, 76, 5, true);
    }

}

void DrawBPM(Graphics_Context *g_sContext_p, int bpmValue)
{
    Graphics_clearDisplay(g_sContext_p);
    Graphics_setForegroundColor(g_sContext_p, GRAPHICS_COLOR_BLUE);
    int8_t bpmString[5] = "BPM:";
    Graphics_drawString(g_sContext_p, bpmString, -1, 40, 5, true);
    if (bpmValue >= 100)
    {
        Graphics_setForegroundColor(g_sContext_p, GRAPHICS_COLOR_BLUE);
        char string[3];
        string[0] = (bpmValue / 100) + '0';
        string[1] = ((bpmValue % 100) / 10) + '0';
        string[2] = (bpmValue % 10) + '0';
        Graphics_drawString(g_sContext_p, (int8_t*) string, -1, 64, 5, true);
        Graphics_setForegroundColor(g_sContext_p, GRAPHICS_COLOR_WHITE);
        int8_t clearPeriod[8] = "       ";
        Graphics_drawString(g_sContext_p, clearPeriod, -1, 84, 5, true);
    }
    else
    {
        Graphics_setForegroundColor(g_sContext_p, GRAPHICS_COLOR_BLUE);
        char string[2];
        string[0] = ((bpmValue % 10) / 10) + '0';
        string[1] = ((bpmValue % 1) / 1) + '0';
        Graphics_drawString(g_sContext_p, (int8_t*) string, -1, 64, 5, true);
        Graphics_setForegroundColor(g_sContext_p, GRAPHICS_COLOR_WHITE);
        int8_t clear[2] = " ";
        Graphics_drawString(g_sContext_p, clear, -1, 76, 5, true);
    }

}
