#ifndef __GNSS_H__
#define __GNSS_H__

#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "peripherals.h"

#include "UBX_GNSS.h"
#include "Miscellaneous.h"

#ifdef __cplusplus
extern "C" {
#endif


extern UBXGNSS_Def_t GNSS_DEV;

void gnss_init(void);


#ifdef __cplusplus
}
#endif

#endif /* __GNSS_H__ */