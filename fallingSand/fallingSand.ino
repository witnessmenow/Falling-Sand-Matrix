/*******************************************************************
    Falling sand animation using a 64x64 RGB Led Matrix,an ESP32 and
    a MPU6050 module.
 *                                                                 *
    Built using an ESP32 and using my own ESP32 Matrix Shield
    https://www.tindie.com/products/brianlough/esp32-matrix-shield-mini-32/
    
    Adapted for the Matrix and MPU6050 by Brian Lough
    YouTube: https://www.youtube.com/brianlough
    Tindie: https://www.tindie.com/stores/brianlough/
    Twitter: https://twitter.com/witnessmenow
 *******************************************************************/

// This was the original header from the Adafruit Learn guide this project 
//  was based on
//  https://learn.adafruit.com/matrix-led-sand/overview
// --------------------------------------------------------------------------
//  Animated 'sand' for Adafruit Feather.  Uses the following parts:
//   - Feather 32u4 Basic Proto (adafruit.com/product/2771)
//   - Charlieplex FeatherWing (adafruit.com/product/2965 - any color!)
//   - LIS3DH accelerometer (2809)
//   - 350 mAh LiPoly battery (2750)
//   - SPDT Slide Switch (805)
//
// This is NOT good "learn from" code for the IS31FL3731; it is "squeeze
// every last byte from the microcontroller" code.  If you're starting out,
// download the Adafruit_IS31FL3731 and Adafruit_GFX libraries, which
// provide functions for drawing pixels, lines, etc.
//--------------------------------------------------------------------------

#include <Wire.h>            // For I2C communication


//Adtional Libraries to install

#include <MPU6050.h>
// For communicating with the MPU6050
//
// You need to install my version from GitHub
// https://github.com/witnessmenow/Arduino-MPU6050

#define double_buffer // this must be enabled to stop flickering
#include <PxMatrix.h>
// The library for controlling the LED Matrix
//
// Can be installed from the library manager
// https://github.com/2dom/PxMatrix

// Adafruit GFX library is a dependancy for the PxMatrix Library
// Can be installed from the library manager
// https://github.com/adafruit/Adafruit-GFX-Library

#include "imgBufferGFX.h"

#define N_GRAINS     1000 // Number of grains of sand
#define WIDTH        64 // Display width in pixels
#define HEIGHT       64 // Display height in pixels
#define MAX_FPS      45 // Maximum redraw rate, frames/second

// The 'sand' grains exist in an integer coordinate space that's 256X
// the scale of the pixel grid, allowing them to move and interact at
// less than whole-pixel increments.
#define MAX_X (WIDTH  * 256 - 1) // Maximum X coordinate in grain space
#define MAX_Y (HEIGHT * 256 - 1) // Maximum Y coordinate
struct Grain {
  int16_t  x,  y; // Position
  int16_t vx, vy; // Velocity
  uint16_t pos;
  uint16_t colour;
} grain[N_GRAINS];

// ----------------------------
// Wiring and Display setup
// ----------------------------

#define P_LAT 22
#define P_A 19
#define P_B 23
#define P_C 18
#define P_D 5
#define P_E 15
// #define P_OE 2
#define P_OE 21 // Feather Huzzah
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// This defines the 'on' time of the display is us. The larger this number,
// the brighter the display. If too large the ESP will crash
uint8_t display_draw_time = 10; //10-50 is usually fine

//PxMATRIX display(matrix_width, matrix_height, P_LAT, P_OE, P_A, P_B, P_C);
//PxMATRIX display(64,32,P_LAT, P_OE,P_A,P_B,P_C,P_D);
PxMATRIX display(64, 64, P_LAT, P_OE, P_A, P_B, P_C, P_D, P_E);

#define NUM_COLOURS 5

uint16_t myRED = display.color565(255, 0, 0);
uint16_t myGREEN = display.color565(0, 255, 0);
uint16_t myBLUE = display.color565(0, 0, 255);
uint16_t myMAGENTA = display.color565(255, 0, 255);
uint16_t myYELLOW = display.color565(255, 255, 0);
uint16_t myCYAN = display.color565(0, 255, 255);

uint16_t myCOLORS[6]={myRED,myGREEN,myCYAN,myMAGENTA,myYELLOW,myBLUE};

MPU6050 mpu;
uint32_t        prevTime   = 0;      // Used for frames-per-second throttle
uint16_t         backbuffer = 0,      // Index for double-buffered animation
                 img[WIDTH * HEIGHT]; // Internal 'map' of pixels

ImgBufferGFX imgWrapper(img, WIDTH, HEIGHT);

float xOffset = -1350; 
float yOffset = -2590;

void pixelTask(void *param) {

  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_4G, MPU6050_ADDRESS, 27, 33))
  {
    //Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

 Vector accelVector = mpu.readRawAccel();

  float xOffset = (accelVector.XAxis * -1) * -1;
  float yOffset = (accelVector.YAxis * -1) * -1;
  
  
  while (true) {
      // Limit the animation frame rate to MAX_FPS.  Because the subsequent sand
  // calculations are non-deterministic (don't always take the same amount
  // of time, depending on their current states), this helps ensure that
  // things like gravity appear constant in the simulation.
  uint32_t t;
  while (((t = micros()) - prevTime) < (1000000L / MAX_FPS));
  prevTime = t;

  //  // Display frame rendered on prior pass.  It's done immediately after the
  //  // FPS sync (rather than after rendering) for consistent animation timing.
  //  pageSelect(0x0B);       // Function registers
  //  writeRegister(0x01);    // Picture Display reg
  //  Wire.write(backbuffer); // Page # to display
  //  Wire.endTransmission();
  //  backbuffer = 1 - backbuffer; // Swap front/back buffer index

  // Read accelerometer...
  //Vector accelVector = mpu.readNormalizeAccel();
  Vector accelVector = mpu.readRawAccel();

  float accelX = (accelVector.XAxis * -1) + xOffset;
  float accelY = (accelVector.YAxis * -1) + yOffset;
  float accelZ = accelVector.ZAxis;

  int16_t ax = -accelY / 256,      // Transform accelerometer axes
          ay =  accelX / 256,      // to grain coordinate space
          az = abs(accelZ) / 2048; // Random motion factor
  az = (az >= 3) ? 1 : 4 - az;      // Clip & invert
  ax -= az;                         // Subtract motion factor from X, Y
  ay -= az;
  int16_t az2 = az * 2 + 1;         // Range of random motion to add back in

  // ...and apply 2D accel vector to grain velocities...
  int32_t v2; // Velocity squared
  float   v;  // Absolute velocity
  for (int i = 0; i < N_GRAINS; i++) {
    grain[i].vx += ax + random(az2); // A little randomness makes
    grain[i].vy += ay + random(az2); // tall stacks topple better!
    // Terminal velocity (in any direction) is 256 units -- equal to
    // 1 pixel -- which keeps moving grains from passing through each other
    // and other such mayhem.  Though it takes some extra math, velocity is
    // clipped as a 2D vector (not separately-limited X & Y) so that
    // diagonal movement isn't faster
    v2 = (int32_t)grain[i].vx * grain[i].vx + (int32_t)grain[i].vy * grain[i].vy;
    if (v2 > 65536) { // If v^2 > 65536, then v > 256
      v = sqrt((float)v2); // Velocity vector magnitude
      grain[i].vx = (int)(256.0 * (float)grain[i].vx / v); // Maintain heading
      grain[i].vy = (int)(256.0 * (float)grain[i].vy / v); // Limit magnitude
    }
  }

  // ...then update position of each grain, one at a time, checking for
  // collisions and having them react.  This really seems like it shouldn't
  // work, as only one grain is considered at a time while the rest are
  // regarded as stationary.  Yet this naive algorithm, taking many not-
  // technically-quite-correct steps, and repeated quickly enough,
  // visually integrates into something that somewhat resembles physics.
  // (I'd initially tried implementing this as a bunch of concurrent and
  // "realistic" elastic collisions among circular grains, but the
  // calculations and volument of code quickly got out of hand for both
  // the tiny 8-bit AVR microcontroller and my tiny dinosaur brain.)

  uint16_t        i, bytes, oldidx, newidx, delta;
  int16_t        newx, newy;

  for (i = 0; i < N_GRAINS; i++) {
    newx = grain[i].x + grain[i].vx; // New position in grain space
    newy = grain[i].y + grain[i].vy;
    if (newx > MAX_X) {              // If grain would go out of bounds
      newx         = MAX_X;          // keep it inside, and
      grain[i].vx /= -2;             // give a slight bounce off the wall
    } else if (newx < 0) {
      newx         = 0;
      grain[i].vx /= -2;
    }
    if (newy > MAX_Y) {
      newy         = MAX_Y;
      grain[i].vy /= -2;
    } else if (newy < 0) {
      newy         = 0;
      grain[i].vy /= -2;
    }

    oldidx = (grain[i].y / 256) * WIDTH + (grain[i].x / 256); // Prior pixel #
    newidx = (newy      / 256) * WIDTH + (newx      / 256); // New pixel #
    if ((oldidx != newidx) && // If grain is moving to a new pixel...
        img[newidx]) {       // but if that pixel is already occupied...
      delta = abs(newidx - oldidx); // What direction when blocked?
      if (delta == 1) {           // 1 pixel left or right)
        newx         = grain[i].x;  // Cancel X motion
        grain[i].vx /= -2;          // and bounce X velocity (Y is OK)
        newidx       = oldidx;      // No pixel change
      } else if (delta == WIDTH) { // 1 pixel up or down
        newy         = grain[i].y;  // Cancel Y motion
        grain[i].vy /= -2;          // and bounce Y velocity (X is OK)
        newidx       = oldidx;      // No pixel change
      } else { // Diagonal intersection is more tricky...
        // Try skidding along just one axis of motion if possible (start w/
        // faster axis).  Because we've already established that diagonal
        // (both-axis) motion is occurring, moving on either axis alone WILL
        // change the pixel index, no need to check that again.
        if ((abs(grain[i].vx) - abs(grain[i].vy)) >= 0) { // X axis is faster
          newidx = (grain[i].y / 256) * WIDTH + (newx / 256);
          if (!img[newidx]) { // That pixel's free!  Take it!  But...
            newy         = grain[i].y; // Cancel Y motion
            grain[i].vy /= -2;         // and bounce Y velocity
          } else { // X pixel is taken, so try Y...
            newidx = (newy / 256) * WIDTH + (grain[i].x / 256);
            if (!img[newidx]) { // Pixel is free, take it, but first...
              newx         = grain[i].x; // Cancel X motion
              grain[i].vx /= -2;         // and bounce X velocity
            } else { // Both spots are occupied
              newx         = grain[i].x; // Cancel X & Y motion
              newy         = grain[i].y;
              grain[i].vx /= -2;         // Bounce X & Y velocity
              grain[i].vy /= -2;
              newidx       = oldidx;     // Not moving
            }
          }
        } else { // Y axis is faster, start there
          newidx = (newy / 256) * WIDTH + (grain[i].x / 256);
          if (!img[newidx]) { // Pixel's free!  Take it!  But...
            newx         = grain[i].x; // Cancel X motion
            grain[i].vy /= -2;         // and bounce X velocity
          } else { // Y pixel is taken, so try X...
            newidx = (grain[i].y / 256) * WIDTH + (newx / 256);
            if (!img[newidx]) { // Pixel is free, take it, but first...
              newy         = grain[i].y; // Cancel Y motion
              grain[i].vy /= -2;         // and bounce Y velocity
            } else { // Both spots are occupied
              newx         = grain[i].x; // Cancel X & Y motion
              newy         = grain[i].y;
              grain[i].vx /= -2;         // Bounce X & Y velocity
              grain[i].vy /= -2;
              newidx       = oldidx;     // Not moving
            }
          }
        }
      }
    }
    grain[i].x  = newx; // Update grain position
    grain[i].y  = newy;
    img[oldidx] = 0;    // Clear old spot (might be same as new, that's OK)
    img[newidx] = 255;  // Set new spot
    grain[i].pos = newidx;
    //Serial.println(newidx);
  }
  }
}

void IRAM_ATTR display_updater() {
  display.display(display_draw_time);
}

void display_update_enable(bool is_enable)
{
  if (is_enable)
  {
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &display_updater, true);
    timerAlarmWrite(timer, 2000, true);
    timerAlarmEnable(timer);
  }
  else
  {
    timerDetachInterrupt(timer);
    timerAlarmDisable(timer);
  }
}

// SETUP - RUNS ONCE AT PROGRAM START --------------------------------------

void setup(void) {
  int i, j, bytes;


  //Serial.begin(115200);

  //Serial.println("Initialize MPU6050");

  // Define your display layout here, e.g. 1/8 step
  display.begin(32);

  display.setFastUpdate(true);
  display.clearDisplay();
  display_update_enable(true);
  display.showBuffer();

  memset(img, 0, sizeof(img)); // Clear the img[] array

  imgWrapper.setCursor(12, 18);
  imgWrapper.setTextColor(myBLUE);
  imgWrapper.setTextSize(4);
  imgWrapper.print("HI");
  
  for (i = 0; i < N_GRAINS; i++) { // For each sand grain...

    int imgIndex = 0;
    do {
      grain[i].x = random(WIDTH  * 256); // Assign random position within
      grain[i].y = random(HEIGHT * 256); // the 'grain' coordinate space
      // Check if corresponding pixel position is already occupied...
      for (j = 0; (j < i) && (((grain[i].x / 256) != (grain[j].x / 256)) ||
                              ((grain[i].y / 256) != (grain[j].y / 256))); j++);
      imgIndex = (grain[i].y / 256) * WIDTH + (grain[i].x / 256);
    } while (img[imgIndex] != 0); // Keep retrying until a clear spot is found
    img[imgIndex] = 255; // Mark it
    grain[i].pos = (grain[i].y / 256) * WIDTH + (grain[i].x / 256);
    grain[i].vx = grain[i].vy = 0; // Initial velocity is zero
    
    grain[i].colour = myCOLORS[i%NUM_COLOURS];
  }

  //imgWrapper.drawChar(24,24, 'u', myBLUE, 0, 3);
  

  //imgWrapper.fillRect(24, 24, 16, 16, myBLUE);

  TaskHandle_t xHandle = NULL;
  xTaskCreatePinnedToCore(pixelTask, "PixelTask1", 5000, 0, (2 | portPRIVILEGE_BIT), &xHandle, 0);
}

// MAIN LOOP - RUNS ONCE PER FRAME OF ANIMATION ----------------------------

void loop() {

  display.clearDisplay();
  //display.drawChar(24,24, 'u', myBLUE, 0, 3);
  for (int i = 0; i < N_GRAINS; i++) {
    int yPos = grain[i].pos / WIDTH;
    int xPos = grain[i].pos % WIDTH;
    display.drawPixel(xPos , yPos, grain[i].colour);
  }
  //display.drawChar(24,24, 'u', myBLUE, 0, 3);
  //display.fillRect(24, 24, 16, 16, myBLUE);

  display.setCursor(12, 18);
  display.setTextColor(myBLUE);
  display.setTextSize(4);
  display.print("HI");
  display.showBuffer();
}

