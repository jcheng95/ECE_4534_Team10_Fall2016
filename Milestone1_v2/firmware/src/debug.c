#include "debug.h"

void dbgOutputVal(unsigned char outVal)
{
    SYS_PORTS_Clear(PORTS_ID_0, PORT_CHANNEL_E, 0x00FF);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_E, outVal, 0x00FF);
}

    // This will output debug values to pins 83, 13, 82, 12, 81, 11, 80, 10
    // The pin order here is:
    // RG12, RA3, RG14, RA2, RA7, RC4, RA6, RD4

    // This will output debug values to pins 40 - 47
    // The pin order here is:
    // RB11, RB13, RB12, RG8, RA10, RF0, RF1, RD6
void dbgOutputLoc(unsigned char outVal)
{
    // Clear ports
    // The hex values are based on position
    /*SYS_PORTS_Clear(PORTS_ID_0, PORT_CHANNEL_A, 0x00CC);
    SYS_PORTS_Clear(PORTS_ID_0, PORT_CHANNEL_C, 0x0010);
    SYS_PORTS_Clear(PORTS_ID_0, PORT_CHANNEL_D, 0x0010);
    SYS_PORTS_Clear(PORTS_ID_0, PORT_CHANNEL_G, 0x5000);*/
    SYS_PORTS_Clear(PORTS_ID_0, PORT_CHANNEL_A, 0x0400);
    SYS_PORTS_Clear(PORTS_ID_0, PORT_CHANNEL_B, 0x3800);
    SYS_PORTS_Clear(PORTS_ID_0, PORT_CHANNEL_D, 0x0040);
    SYS_PORTS_Clear(PORTS_ID_0, PORT_CHANNEL_F, 0x0003);
    SYS_PORTS_Clear(PORTS_ID_0, PORT_CHANNEL_G, 0x0100);
    
    // Sets ports
    // The hex values are based on position
    
    /*SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_G, ((outVal & 0x80) >> 7) << 12, 0x1000);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_A, ((outVal & 0x40) >> 6) << 3, 0x0008);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_G, ((outVal & 0x20) >> 5) << 14, 0x4000);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_A, ((outVal & 0x10) >> 4) << 2, 0x0004);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_A, ((outVal & 0x08) >> 3) << 7, 0x0080);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_C, ((outVal & 0x04) >> 2) << 4, 0x0010);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_A, ((outVal & 0x02) >> 1) << 6, 0x0040);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_D, (outVal & 0x01) << 4, 0x0010);*/
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_B, 0, 0x0800);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_B, 0, 0x2000);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_B, 0, 0x1000);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_G, 0, 0x0100);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_A, 0, 0x0400);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_F, 0, 0x0001);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_F, 0, 0x0002);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_D, 0, 0x0040);
    
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_B, ((outVal & 0x80) >> 7) << 11, 0x0800);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_B, ((outVal & 0x40) >> 6) << 13, 0x2000);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_B, ((outVal & 0x20) >> 5) << 12, 0x1000);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_G, ((outVal & 0x10) >> 4) << 8, 0x0100);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_A, ((outVal & 0x08) >> 3) << 10, 0x0400);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_F, ((outVal & 0x04) >> 2), 0x0001);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_F, ((outVal & 0x02) >> 1) << 1, 0x0002);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_D, (outVal & 0x01) << 6, 0x0040);
}

void stopProcess(unsigned int outVal)
{
    if(outVal == ERROR_CODE) {
        // Suspends all tasks without stopping the interrupt
        vTaskSuspendAll();
        
        // Disabling interrupts
        taskENTER_CRITICAL();
        
        dbgOutputLoc(ERROR_STATE);
        
        // Continue to output error state
        while(1);
    }
}