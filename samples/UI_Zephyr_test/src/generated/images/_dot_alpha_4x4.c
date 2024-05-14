#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
#include <zephyr/lvgl.h>
#else
#include "lvgl/lvgl.h"
#endif


#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif

#ifndef LV_ATTRIBUTE_IMG__DOT_ALPHA_4X4
#define LV_ATTRIBUTE_IMG__DOT_ALPHA_4X4
#endif

const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST LV_ATTRIBUTE_IMG__DOT_ALPHA_4X4 uint8_t _dot_alpha_4x4_map[] = {
#if LV_COLOR_DEPTH == 1 || LV_COLOR_DEPTH == 8
  /*Pixel format: Alpha 8 bit, Red: 3 bit, Green: 3 bit, Blue: 2 bit*/
  0xdb, 0xff, 0xff, 0xff, 0xff, 0xff, 0xdb, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xdb, 0xff, 0xff, 0xff, 0xff, 0xff, 0xdb, 0xff, 
#endif
#if LV_COLOR_DEPTH == 16 && LV_COLOR_16_SWAP == 0
  /*Pixel format: Alpha 8 bit, Red: 5 bit, Green: 6 bit, Blue: 5 bit*/
  0x18, 0xc6, 0xff, 0xdf, 0xff, 0xff, 0xdf, 0xff, 0xff, 0xf8, 0xc5, 0xff, 
  0xdf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xdf, 0xff, 0xff, 
  0xdf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xdf, 0xff, 0xff, 
  0x18, 0xc6, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x18, 0xc6, 0xff, 
#endif
#if LV_COLOR_DEPTH == 16 && LV_COLOR_16_SWAP != 0
  /*Pixel format: Alpha 8 bit, Red: 5 bit, Green: 6 bit, Blue: 5 bit  BUT the 2  color bytes are swapped*/
  0xc6, 0x18, 0xff, 0xff, 0xdf, 0xff, 0xff, 0xdf, 0xff, 0xc5, 0xf8, 0xff, 
  0xff, 0xdf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xdf, 0xff, 
  0xff, 0xdf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xdf, 0xff, 
  0xc6, 0x18, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc6, 0x18, 0xff, 
#endif
#if LV_COLOR_DEPTH == 32
  /*Pixel format: Alpha 8 bit, Red: 8 bit, Green: 8 bit, Blue: 8 bit*/
  0xbd, 0xbe, 0xbd, 0xff, 0xf9, 0xf9, 0xf9, 0xff, 0xf9, 0xf9, 0xf9, 0xff, 0xbd, 0xbd, 0xbd, 0xff, 
  0xf9, 0xf9, 0xf9, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xf9, 0xf9, 0xff, 
  0xf9, 0xf9, 0xf9, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xf9, 0xf9, 0xff, 
  0xc0, 0xc0, 0xc0, 0xff, 0xfa, 0xfa, 0xfa, 0xff, 0xf9, 0xfa, 0xf9, 0xff, 0xbe, 0xbf, 0xbe, 0xff, 
#endif
};

const lv_img_dsc_t _dot_alpha_4x4 = {
  .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
  .header.always_zero = 0,
  .header.reserved = 0,
  .header.w = 4,
  .header.h = 4,
  .data_size = 16 * LV_IMG_PX_SIZE_ALPHA_BYTE,
  .data = _dot_alpha_4x4_map,
};
