#ifndef _ECONPASS_H_
#define _ECONPASS_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif


void ecompass_init(void);
void ecompass_get_heading(float *heading);


#ifdef __cplusplus
}
#endif

#endif /* _ECONPASS_H_ */