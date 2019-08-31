#include <ti/grlib/grlib.h>
#include "LcdDriver/Crystalfontz128x128_ST7735.h"

void ChangeBPM(Graphics_Context *g_sContext_p, int bpmValue, float lux);
void DrawBPM(Graphics_Context *g_sContext_p, int bpmValue);
void MakeBPMCircle(Graphics_Context *g_sContext_p, float lux);
void ClearBPMCircle(Graphics_Context *g_sContext_p, float lux);
void WriteFFT(Graphics_Context *g_sContext_p);
void DrawReverseFFT(Graphics_Context *g_sContext_p);
