#ifndef _epistasis_H
#define _epistasis_H

#include "headfile.h"
#include "common.h"

void vcan_sendimg(uint8 *imgaddr, uint32_t imgsize);
void vcan_sendccd(uint8 *ccdaddr, uint32_t ccdsize);
void vcan_sendware(uint8 *wareaddr, uint32_t waresize);

#endif
