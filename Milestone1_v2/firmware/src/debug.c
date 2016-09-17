#include "debug.h"

void dbgOutputVal(unsigned char outVal)
{
    SYS_PORTS_Clear(PORTS_ID_0, PORT_CHANNEL_E, 0x00FF);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_E, outVal, 0x00FF);
}

void dbgOutputLoc(unsigned char outVal)
{
    SYS_PORTS_Clear(PORTS_ID_0, PORT_CHANNEL_B, 0x00FF);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_B, outVal, 0x00FF);
}