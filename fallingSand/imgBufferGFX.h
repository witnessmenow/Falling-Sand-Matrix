#pragma once
#include "Adafruit_GFX.h"

class ImgBufferGFX : public Adafruit_GFX
{
  public:
    uint16_t *imgBuffer;
    int imgBufferSize;
    int resX;
    int resY;
    
    ImgBufferGFX(uint16_t *img, const int xres, const int yres)
        :Adafruit_GFX(xres, yres)
    {
      imgBuffer = img;
      imgBufferSize = xres * yres;
      resX = xres;
      resY = yres;
    }

    virtual void drawPixel(int16_t x, int16_t y, uint16_t color)
    {
      int16_t index = (y * resX) + x;
      if(index < imgBufferSize){
         imgBuffer[index] = color;
      }     
    }
};
