/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _MAINAPP_PUBLIC_H    /* Guard against multiple inclusion */
#define _MAINAPP_PUBLIC_H

#include "common.h"

BaseType_t sendToMainAppQueue(messageStructure msg);
BaseType_t sendToMainAppQueueFromISR(messageStructure msg);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _MAINAPP_PUBLIC_H */

/* *****************************************************************************
 End of File
 */
