// Interpolate grid if sensor data, interpolate it to a larger grid and use it to light a bunch of Adafruit Neopixels.
// In this case, the sensor is an 8x8 AMG88XX IR camera.

#include <Adafruit_NeoPixel_ZeroDMA.h>
#define PIN        5
#define NUM_PIXELS 116

#define CAMERA_RES    8
#define INTERP_RES    23

Adafruit_NeoPixel_ZeroDMA strip(NUM_PIXELS, PIN, NEO_GRB);

#include <Wire.h>
#include <Adafruit_AMG88xx.h>

Adafruit_AMG88xx amg;

float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float pixelsGrid[CAMERA_RES][CAMERA_RES];

// Test vars
byte rainbowDelay = 20;
byte colorAdvance = 0;
unsigned long previousRainbowMillis = 0;
int column;
int row;

#define MINTEMP 19 //low range of the sensor (this will be blue on the screen)
#define MAXTEMP 32 //high range of the sensor (this will be red on the screen)

// LED array. Defines pixels a grid rather than chain
int ledArray[23][23] =
{
  { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { -1, -1, -1, -1, -1, -1, -1, -1, -1, 42, 32, -1, 20, 19, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { -1, -1, -1, -1, -1, -1, -1, -1, 43, 41, 33, 31, 21, 18, 12, -1, -1, -1, -1, -1, -1, -1, -1},
  { -1, -1, -1, -1, -1, -1, -1, -1, 44, 40, 34, 30, 22, 17, 13, -1, -1, -1, -1, -1, -1, -1, -1},
  { -1, -1, -1, -1, -1, -1, -1, -1, 45, 39, 35, 29, 23, 16, 14, -1, -1, -1, -1, -1, -1, -1, -1},
  { -1, -1, -1, -1, -1, -1, -1, -1, -1, 38, 36, 28, 24, 15, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 37, 27, 25, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 26, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { -1, 51, 52, -1, -1, -1, 62, 70, -1, -1, 76, -1, 80, -1, 91, 95, -1, -1, -1, 106, 107, 112, -1},
  { 46, 50, 53, 59, 60, -1, 63, 69, -1, -1, 77, -1, 81, 90, -1, 94, 96, -1, 99, 105, -1, -1, 113},
  { -1, 49, 54, -1, -1, -1, 64, 68, 71, -1, 78, -1, 82, 89, -1, -1, 97, -1, 100, 104, 108, 111, 114},
  { -1, 48, 55, -1, -1, -1, -1, 67, 72, -1, 79, -1, 83, 88, -1, -1, 98, -1, 101, 103, -1, -1, -1},
  { -1, 47, 56, -1, -1, -1, -1, -1, 73, 75, -1, -1, 84, 87, 92, 93, -1, -1, -1, 102, 109, 110, 115},
  { -1, -1, 57, 58, 61, -1, 65, 66, 74, -1, -1, -1, 85, 86, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},

};

void setup() {
  
  // Lights
  strip.begin();
  strip.setBrightness(60);
  strip.show();
  
  // Camera
  Serial.begin(9600);
  Serial.println(F("AMG88xx pixels"));

  bool status;
    
  // default camera settings
  status = amg.begin();
  if (!status) {
      Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
      while (1);
  }

  delay(100); // let sensor boot up

}

void loop() {
  
  // Read all the camera pixels
  amg.readPixels(pixels);
  uint8_t camPixelColors[AMG88xx_PIXEL_ARRAY_SIZE];
  
  // Iterate through 2d array converting temperature value to RGB
  int i;
  for(i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    
    // Map temp to 0-255
    uint8_t colorIndex = map(pixels[i], MINTEMP, MAXTEMP, 0, 255);
    colorIndex = constrain(colorIndex, 0, 255);
      
    //draw the pixels!
    strip.setPixelColor(i, Rainbow(colorIndex));
    strip.show();
    
  }
  
}

// Input a value 0 to 256 to get a color value.
// The colors are a transition b - g - r.
uint32_t Rainbow(byte RainbowPos) {
  if(RainbowPos <= 128) {
    return strip.Color(0, RainbowPos * 2, 255 - RainbowPos * 2);
  }
  else {
    RainbowPos = 256 - RainbowPos;
    return strip.Color(RainbowPos * 2, 255 - RainbowPos * 2, 0);
  }
}

//Roland's function
int val(int camera[CAMERA_RES * CAMERA_RES], int x, int y) {
  return camera[y * CAMERA_RES + x];
}

int set(int out[INTERP_RES * INTERP_RES], int x, int y, int val) {
  out[y * INTERP_RES + x] = val;
}

void interpolate(int camera[CAMERA_RES * CAMERA_RES], int out[INTERP_RES * INTERP_RES]) {
  int ox, oy;
  int lx, ly;
  int sum;
  int wt;

  for (ox = 0; ox < INTERP_RES; ++ox) {
    for (oy = 0; oy < INTERP_RES; ++oy) {
      // Four cases: corners get exact value
      // edges just interpolate along one axis
      // otherwise weighted average of four points
      sum = 0;
      lx = ox * (CAMERA_RES - 1) / (INTERP_RES - 1);
      ly = oy * (CAMERA_RES - 1) / (INTERP_RES - 1);
      if (ox == 0 || ox == INTERP_RES - 1) {
        if (oy == 0 || oy == INTERP_RES - 1) {
          set(out, ox, oy, val(camera, lx, ly));
        } else {
          wt = (INTERP_RES - 1) - oy * (CAMERA_RES - 1) + ly * (INTERP_RES - 1);
          sum = val(camera, lx, ly) * wt + val(camera, lx, ly + 1) * (INTERP_RES - 1 - wt);
          set(out, ox, oy, sum / (INTERP_RES - 1));
        }
      } else {
        if (oy == 0 || oy == INTERP_RES - 1) {
          wt = (INTERP_RES - 1) - ox * (CAMERA_RES - 1) + lx * (INTERP_RES - 1);
          sum = val(camera, lx, ly) * wt + val(camera, lx + 1, ly) * (INTERP_RES - 1 - wt);
          set(out, ox, oy, sum / (INTERP_RES - 1));
        } else {
          // the complicated case, find 4 corners
          // the last case, the four points are (lx, ly), (lx + 1, ly), (lx, ly + 1)
          // and (lx + 1, ly + 1) - if you just average those 4 it will probably look fine.
          // I tried to be fancy with weighted averages for the other cases.

        }
      }

    }
  }
}