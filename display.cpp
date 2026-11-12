#include "display.h"
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

namespace {
// Keep the wiring and color configuration from the working prototype.
#define TFT_CS 10
#define TFT_DC 9
#define TFT_RST 8
#define TFT_MOSI 11
#define TFT_SCK 12
Adafruit_ST7789 tft(&SPI, TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 eyeCanvas(80, 130);
// Compose the whole frame before a bulk RGB565 transfer (153600 bytes).
GFXcanvas16 face(320, 240);
// If long jumper wires cause artifacts, reduce this to 10000000.
const uint32_t TFT_SPI_HZ = 20000000;

face_t currentExpression = FACE_IDLE;
bool ready = false, blinking = false;
uint32_t expressionStarted=0, lastBlinkFinished=0, blinkStarted=0, lastEyeFrame=0;
uint32_t timerSeconds=0;
const uint32_t BLINK_INTERVAL=4000, CLOSE_TIME=240, HOLD_TIME=120, OPEN_TIME=160;
void drawStudyScreen(uint32_t seconds);
void drawLabel(const char *label) {
  face.setTextWrap(false); face.setTextColor(ST77XX_CYAN); face.setTextSize(3);
  int width = (int)strlen(label)*18;
  face.setCursor((320-width)/2,109); face.print(label);
}
// Rounded strokes for the reference-style line drawings.
void stroke(int x0, int y0, int x1, int y1, int radius = 2) {
  int dx = abs(x1 - x0), dy = abs(y1 - y0);
  int steps = max(dx, dy);
  if (steps == 0) {
    face.fillCircle(x0, y0, radius, ST77XX_CYAN);
    return;
  }
  for (int i = 0; i <= steps; i++) {
    face.fillCircle(x0 + (x1 - x0) * i / steps,
                    y0 + (y1 - y0) * i / steps, radius, ST77XX_CYAN);
  }
}

// Angles follow screen coordinates: positive sine points downward.
void arcStroke(int cx, int cy, int rx, int ry,
               float startDegrees, float endDegrees, int radius = 2) {
  for (float a = startDegrees; a <= endDegrees; a += 2.0f) {
    float angle = a * PI / 180.0f;
    face.fillCircle(cx + (int)lroundf(rx * cosf(angle)),
                    cy + (int)lroundf(ry * sinf(angle)), radius, ST77XX_CYAN);
  }
}

void drawNormalEyes(int height, bool pushToScreen = true) {
  height = constrain(height, 2, 130);
  eyeCanvas.fillScreen(ST77XX_BLACK);
  for (int y = 0; y < 130; y++) {
    float dy = (y + 0.5f - 65.0f) / (height / 2.0f);
    if (fabsf(dy) >= 1.0f) continue;
    float halfWidth = 40.0f * sqrtf(1.0f - dy * dy);
    int left = (int)ceilf(40.0f - halfWidth - 0.5f);
    int right = (int)floorf(40.0f + halfWidth - 0.5f);
    if (right >= left) {
      eyeCanvas.drawFastHLine(left, y, right - left + 1, ST77XX_CYAN);
    }
  }
  if (pushToScreen) {
    // Blinking only changes the eye regions; keep these updates small.
    tft.drawRGBBitmap(40, 55, eyeCanvas.getBuffer(), 80, 130);
    tft.drawRGBBitmap(200, 55, eyeCanvas.getBuffer(), 80, 130);
  } else {
    // Compose the normal expression without clearing the physical display.
    face.drawRGBBitmap(40, 55, eyeCanvas.getBuffer(), 80, 130);
    face.drawRGBBitmap(200, 55, eyeCanvas.getBuffer(), 80, 130);
  }
}

void drawHappy() {
  // Inward-pointing squeezed eyes: > <.
  stroke(51, 72, 108, 102, 3);
  stroke(108, 102, 56, 119, 3);
  stroke(269, 72, 212, 102, 3);
  stroke(212, 102, 264, 119, 3);
  // Short cheek marks.
  for (int i = 0; i < 4; i++) {
    stroke(62 + i * 10, 135, 58 + i * 10, 151, 1);
    stroke(230 + i * 10, 135, 226 + i * 10, 151, 1);
  }
  // Open smiling mouth, matching the trapezoid in the reference.
  stroke(137, 139, 183, 139, 2);
  stroke(137, 139, 146, 166, 2);
  stroke(146, 166, 174, 166, 2);
  stroke(174, 166, 183, 139, 2);
}

void drawSleepy() {
  // Layered, flattened eyelids with uneven ends.
  for (int side = 0; side < 2; side++) {
    int x = 46 + side * 168;
    stroke(x + 3, 95, x + 61, 95, 3);
    stroke(x, 104, x + 53, 104, 3);
    stroke(x + 5, 113, x + 65, 113, 3);
    stroke(x + 1, 122, x + 58, 122, 3);
  }
  // Small open mouth with a drooping corner.
  stroke(134, 145, 183, 145, 2);
  stroke(134, 145, 134, 164, 2);
  stroke(134, 164, 169, 164, 2);
  stroke(169, 164, 169, 179, 2);
  arcStroke(176, 179, 7, 7, 0, 180, 2);
  stroke(183, 179, 183, 145, 2);
}

void drawShy() {
  // Hollow circular eyes.
  face.fillCircle(66, 105, 27, ST77XX_CYAN);
  face.fillCircle(66, 105, 20, 0);
  face.fillCircle(254, 105, 27, ST77XX_CYAN);
  face.fillCircle(254, 105, 20, 0);
  // Blush marks between the eyes.
  for (int i = 0; i < 7; i++) {
    stroke(119 + i * 12, 108, 126 + i * 12, 95, 1);
  }
  // Rounded W-shaped mouth.
  stroke(132, 133, 132, 143, 2);
  arcStroke(146, 143, 14, 12, 0, 180, 2);
  stroke(160, 133, 160, 143, 2);
  arcStroke(174, 143, 14, 12, 0, 180, 2);
  stroke(188, 133, 188, 143, 2);
}

void drawAngry() {
  // Narrow eyes; inner corners sit lower than outer corners.
  face.fillTriangle(49, 90, 115, 111, 67, 126, ST77XX_CYAN);
  face.fillTriangle(271, 90, 205, 111, 253, 126, ST77XX_CYAN);
  stroke(49, 77, 114, 99, 3);
  stroke(271, 77, 206, 99, 3);
  // Downturned mouth.
  arcStroke(160, 175, 23, 17, 205, 335, 2);
  // Comic anger mark in the upper-right corner.
  stroke(274, 32, 277, 42, 2);
  stroke(277, 42, 287, 45, 2);
  stroke(299, 32, 296, 42, 2);
  stroke(296, 42, 289, 45, 2);
  stroke(274, 58, 277, 49, 2);
  stroke(277, 49, 284, 47, 2);
  stroke(299, 58, 296, 49, 2);
  stroke(296, 49, 289, 47, 2);
}

// Shared outline used by the two supplied reference faces.
void drawFaceOutline() {
  const int x[8] = {105, 215, 250, 250, 215, 105, 70, 70};
  const int y[8] = {20, 20, 55, 185, 220, 220, 185, 55};
  for (int i = 0; i < 8; i++) {
    int j = (i + 1) % 8;
    stroke(x[i], y[i], x[j], y[j], 4);
  }
}

void drawListening() {
  drawFaceOutline();
  // Asymmetric eyes and a small tilted mouth.
  face.fillCircle(126, 89, 19, ST77XX_CYAN);
  face.fillCircle(204, 89, 13, ST77XX_CYAN);
  stroke(176, 123, 191, 115, 3);
  // A simplified hand resting under the chin, as in the reference.
  face.fillTriangle(121, 180, 171, 164, 183, 211, ST77XX_CYAN);
  face.fillTriangle(121, 180, 183, 211, 137, 232, ST77XX_CYAN);
  stroke(150, 183, 136, 155, 8);
  stroke(156, 174, 206, 146, 8);
  stroke(142, 204, 156, 222, 14);
}

// Rotate the eye and mouth geometry together by a small angle.
void flipTransform(int x, int y, int &outX, int &outY) {
  const float angle = -8.0f * PI / 180.0f;
  outX = 160 + (int)lroundf((x - 160) * cosf(angle) - (y - 120) * sinf(angle));
  outY = 120 + (int)lroundf((x - 160) * sinf(angle) + (y - 120) * cosf(angle));
}

void flipStroke(int x0, int y0, int x1, int y1, int radius) {
  int ax, ay, bx, by;
  flipTransform(x0, y0, ax, ay);
  flipTransform(x1, y1, bx, by);
  stroke(ax, ay, bx, by, radius);
}

void flipTriangle(int x0, int y0, int x1, int y1, int x2, int y2) {
  int ax, ay, bx, by, cx, cy;
  flipTransform(x0, y0, ax, ay);
  flipTransform(x1, y1, bx, by);
  flipTransform(x2, y2, cx, cy);
  face.fillTriangle(ax, ay, bx, by, cx, cy, ST77XX_CYAN);
}

// Filled Bezier outline: a sharp tip and a smoothly rounded bottom.
void drawTear(int cx, int cy, float angleDegrees, float scale) {
  const float px[3][4] = {{0, 1, 8, 8}, {8, 8, -8, -8}, {-8, -8, -1, 0}};
  const float py[3][4] = {{-14, -7, -2, 4}, {4, 14, 14, 4}, {4, -2, -7, -14}};
  float a = angleDegrees * PI / 180.0f;
  float c = cosf(a), s = sinf(a);
  int centerX = cx - (int)lroundf(3 * scale * s);
  int centerY = cy + (int)lroundf(3 * scale * c);
  int previousX = cx + (int)lroundf(14 * scale * s);
  int previousY = cy - (int)lroundf(14 * scale * c);
  for (int segment = 0; segment < 3; segment++) {
    for (int i = 1; i <= 16; i++) {
      float t = i / 16.0f, u = 1.0f - t;
      float x = (u*u*u*px[segment][0] + 3*u*u*t*px[segment][1]
                 + 3*u*t*t*px[segment][2] + t*t*t*px[segment][3]) * scale;
      float y = (u*u*u*py[segment][0] + 3*u*u*t*py[segment][1]
                 + 3*u*t*t*py[segment][2] + t*t*t*py[segment][3]) * scale;
      int nextX = cx + (int)lroundf(x*c - y*s);
      int nextY = cy + (int)lroundf(x*s + y*c);
      face.fillTriangle(centerX, centerY, previousX, previousY,
                        nextX, nextY, ST77XX_CYAN);
      previousX = nextX;
      previousY = nextY;
    }
  }
}

void drawFlipped() {
  drawFaceOutline();
  // Slightly tilted pleading eyes and open mouth.
  flipStroke(103, 91, 135, 80, 5);
  flipStroke(135, 80, 135, 68, 5);
  flipStroke(185, 68, 185, 80, 5);
  flipStroke(185, 80, 217, 91, 5);
  flipTriangle(146, 112, 174, 112, 186, 173);
  flipTriangle(146, 112, 186, 173, 134, 173);
  // Tips point toward the face; round ends point away.
  drawTear(48, 98, 75, 1.0f);
  drawTear(52, 129, 35, 0.85f);
  drawTear(272, 98, -75, 1.0f);
  drawTear(268, 129, -35, 0.85f);
}

void drawSleepZ(int x, int y, int width, int height, int radius) {
  stroke(x, y, x + width, y, radius);
  stroke(x + width, y, x, y + height, radius);
  stroke(x, y + height, x + width, y + height, radius);
}

void drawWakeFrame(uint32_t elapsed) {
  face.fillScreen(ST77XX_BLACK);
  int height = 4;
  if (elapsed < 1200) {
    // Three Zs grow in size toward the upper-left area of the screen.
    // They disappear one at a time before the eyes open.
    if (elapsed < 600) drawSleepZ(30, 65, 8, 10, 1);
    if (elapsed < 900) drawSleepZ(49, 32, 13, 17, 1);
    drawSleepZ(76, 10, 19, 25, 2);
  } else if (elapsed < 2300) {
    float t = (elapsed - 1200) / 1100.0f;
    float smooth = t * t * (3.0f - 2.0f * t);
    height = 4 + (int)(126.0f * smooth);
  } else if (elapsed < 2650) {
    height = 130;
  } else if (elapsed < 2890) {
    height = 130 - 126 * (elapsed - 2650) / 240;
  } else if (elapsed < 3010) {
    height = 4;
  } else if (elapsed < 3170) {
    height = 4 + 126 * (elapsed - 3010) / 160;
  } else {
    height = 130;
  }
  drawNormalEyes(height, false);
  tft.drawRGBBitmap(0, 0, face.getBuffer(), 320, 240);
}

// Symbolic charging indicator; this is not a measured battery percentage.
void drawChargingScreen() {
  stroke(99, 65, 209, 65, 3);
  stroke(209, 65, 209, 135, 3);
  stroke(209, 135, 99, 135, 3);
  stroke(99, 135, 99, 65, 3);
  stroke(215, 85, 221, 85, 3);
  stroke(221, 85, 221, 115, 3);
  stroke(221, 115, 215, 115, 3);
  // Lightning bolt inside the battery.
  stroke(167, 77, 145, 101, 3);
  stroke(145, 101, 166, 101, 3);
  stroke(166, 101, 146, 124, 3);
  face.setTextWrap(false);
  face.setTextColor(ST77XX_CYAN);
  face.setTextSize(3);
  face.setCursor(89, 169);
  face.print("CHARGING");
}

// Shared startled face for lifted and edge events, based on the reference.
void drawStartled() {
  drawFaceOutline();
  stroke(121,76,121,122,7);
  stroke(199,76,199,122,7);
  for (int y=151;y<=181;y++) face.drawFastHLine(151,y,19,ST77XX_CYAN);
}
void drawConfused() {
  drawFaceOutline();
  arcStroke(119,101,18,23,0,360,3);
  arcStroke(201,105,14,15,0,360,3);
  stroke(101,64,135,72,3);
  stroke(186,70,217,59,3);
  stroke(141,163,159,156,3);
  stroke(159,156,179,162,3);
  // Small question mark beside the face.
  arcStroke(280,56,10,10,180,360,2);
  stroke(290,56,280,68,2); stroke(280,68,280,75,2);
  face.fillCircle(280,85,2,ST77XX_CYAN);
}
void drawDisconnected() {
  drawFaceOutline();
  stroke(103,88,137,99,4);
  stroke(183,99,217,88,4);
  arcStroke(160,178,22,15,195,345,3);
  // Crossed radio arcs make this distinct from a general error.
  arcStroke(160,150,30,24,215,325,2);
  arcStroke(160,150,18,14,215,325,2);
  face.fillCircle(160,146,3,ST77XX_CYAN);
  stroke(135,116,186,151,3);
}
void drawErrorFace() {
  drawFaceOutline();
  for (int x=118;x<=202;x+=84) {
    stroke(x-15,77,x+15,107,4);
    stroke(x+15,77,x-15,107,4);
  }
  stroke(130,159,145,149,3); stroke(145,149,160,159,3);
  stroke(160,159,175,149,3); stroke(175,149,190,159,3);
  // Exclamation mark outside the border.
  stroke(278,87,278,120,4); face.fillCircle(278,135,4,ST77XX_CYAN);
}

// Fill a convex polygon using a fan of triangles.
void facePolygon(const int (*points)[2], int count, uint16_t color=ST77XX_CYAN) {
  for (int i=1;i<count-1;i++)
    face.fillTriangle(points[0][0],points[0][1],points[i][0],points[i][1],points[i+1][0],points[i+1][1],color);
}
// Scanline fill supports the concave thumb notch and wrist contours.
void contourFill(const int (*p)[2], int n, uint16_t color=ST77XX_CYAN) {
  for(int y=0;y<240;y++) {
    float hits[32];int count=0;float scan=y+0.5f;
    for(int i=0;i<n;i++) {
      int j=(i+1)%n;
      if((p[i][1]<=scan && p[j][1]>scan)||(p[j][1]<=scan && p[i][1]>scan))
        hits[count++]=p[i][0]+(scan-p[i][1])*(p[j][0]-p[i][0])/(p[j][1]-p[i][1]);
    }
    for(int i=1;i<count;i++){float v=hits[i];int j=i;while(j>0&&hits[j-1]>v){hits[j]=hits[j-1];j--;}hits[j]=v;}
    for(int i=0;i+1<count;i+=2){int x=(int)ceilf(hits[i]-0.5f),end=(int)floorf(hits[i+1]-0.5f);if(end>=x)face.drawFastHLine(x,y,end-x+1,color);}
  }
}
void drawAckFace() {
  drawFaceOutline();
  const int leftEye[][2]={{138,78},{164,93},{164,130},{146,130}};
  contourFill(leftEye,4);
  const int rightEye[][2]={{220,78},{193,93},{193,130},{211,130}};
  contourFill(rightEye,4);
  const int mouth[][2]={{160,147},{197,147},{197,161},{160,161}};
  contourFill(mouth,4);
  // One traced outline retains the V-shaped gap beside the thumb.
  const int salute[][2]={{54,122},{99,72},{107,72},{123,89},{123,102},{103,124},{111,121},{118,129},{96,155},{65,155},{54,133}};
  contourFill(salute,11);
}
void drawLiftedFace() {
  drawFaceOutline();
  // Three columns of dots in each eye; the center column is longer.
  for (int cx=123;cx<=197;cx+=74) {
    for(int col=-1;col<=1;col++) {
      int first=col==0 ? 70:81, last=col==0 ? 127:116;
      stroke(cx+col*11,first,cx+col*11,last,1);
      for(int y=first;y<=last;y+=12)face.fillCircle(cx+col*11,y,4,ST77XX_CYAN);
    }
  }
  stroke(155,151,155,180,7);stroke(165,151,165,180,7);
}
void drawPrayingFlip() {
  drawFaceOutline();
  const int leftEye[][2]={{111,86},{147,119},{103,134},{100,122},{124,115},{104,95}};
  contourFill(leftEye,6);
  const int rightEye[][2]={{209,86},{173,119},{218,134},{221,122},{197,115},{217,95}};
  contourFill(rightEye,6);
  const int leftClear[][2]={{112,242},{112,216},{124,204},{131,185},{140,152},{159,152},{159,230},{151,240}};
  contourFill(leftClear,8,ST77XX_BLACK);
  const int rightClear[][2]={{206,242},{206,216},{195,204},{188,185},{179,152},{159,152},{159,230},{168,240}};
  contourFill(rightClear,8,ST77XX_BLACK);
  const int leftPalm[][2]={{116,240},{116,218},{127,207},{134,187},{143,157},{144,155},{155,155},{157,158},{157,227},{152,232},{147,236}};
  contourFill(leftPalm,11);
  const int rightPalm[][2]={{161,158},{163,155},{173,155},{175,157},{183,187},{191,207},{202,218},{202,240},{171,236},{166,232},{161,227}};
  contourFill(rightPalm,11);
  const int tear[][2]={{204,136},{206,144},{209,154},{211,161},{211,168},{209,172},{206,174},{202,174},{198,172},{196,168},{196,161},{198,154},{200,144}};
  contourFill(tear,13);
}
void drawHeart(int cx,int cy,int size) {
  const uint16_t red=0xf800;
  face.fillCircle(cx-size/2,cy,size/2,red);
  face.fillCircle(cx+size/2,cy,size/2,red);
  face.fillTriangle(cx-size,cy,cx+size,cy,cx,cy+size+size/2,red);
}
void drawSittingFace() {
  drawFaceOutline();
  stroke(112,83,137,106,5);stroke(137,106,109,114,5);
  stroke(208,83,183,106,5);stroke(183,106,211,114,5);
  stroke(133,153,146,165,5);stroke(146,165,177,165,5);stroke(177,165,190,153,5);
  drawHeart(247,31,18);drawHeart(75,192,17);
}
void drawWifiDisconnected() {
  // Full Wi-Fi arcs with one diagonal cancellation slash.
  for(int ring=0;ring<3;ring++)
    arcStroke(160,173,83-ring*25,72-ring*21,210,330,4);
  face.fillCircle(160,174,7,ST77XX_CYAN);
  // Black clearance makes the cyan slash readable across the arcs.
  for(int i=0;i<=112;i++)
    face.fillCircle(104+i,190-i,8,ST77XX_BLACK);
  stroke(104,190,216,78,4);
}
void drawReferenceError() {
  const int x[8]={105,215,250,250,215,105,70,70};
  const int y[8]={20,20,55,185,220,220,185,55};
  for(int i=0;i<8;i++) {
    if(i==2)continue; // Open right side of the frame.
    int j=(i+1)%8;stroke(x[i],y[i],x[j],y[j],4);
  }
  stroke(116,78,204,166,10);stroke(204,78,116,166,10);
}

void renderFace(face_t expression) {
  currentExpression = expression;
  blinking = false;
  face.fillScreen(0);
  switch (expression) {
    case FACE_HAPPY: drawHappy(); break;
    case FACE_SLEEP: drawSleepy(); break;
    case FACE_SHY: drawShy(); break;
    case FACE_ANGRY: drawAngry(); break;
    case FACE_IDLE: drawNormalEyes(130, false); break;
    case FACE_LISTENING: drawListening(); break;
    case FACE_FLIPPED: drawPrayingFlip(); break;
    case FACE_CHARGING: drawChargingScreen(); break;
    case FACE_LOW_BATTERY:
      face.setTextWrap(false);
      face.setTextColor(ST77XX_CYAN);
      face.setTextSize(3);
      face.setCursor(61, 109);
      face.print("LOW BATTERY");
      break;
    case FACE_STUDY: drawStudyScreen(timerSeconds); return;
    case FACE_CONFUSED: drawConfused(); break;
    case FACE_SITTING: drawSittingFace(); break;
    case FACE_LIFTED: drawLiftedFace(); break;
    case FACE_EDGE: drawFlipped(); break;
    case FACE_NO_CONNECTION: drawWifiDisconnected(); break;
    case FACE_ERROR: drawReferenceError(); break;
    case FACE_ACK: drawAckFace(); break;
    case FACE_WAKING:
      drawWakeFrame(0);
      expressionStarted = millis();
      lastEyeFrame = expressionStarted;
      return;
  }
  // One full-frame transfer, including background, eyes and mouth.
  // SPI transfer is faster, but is not an atomic hardware frame swap.
  tft.drawRGBBitmap(0, 0, face.getBuffer(), 320, 240);
  expressionStarted = millis();
  lastBlinkFinished = expressionStarted;
}

void drawStudyScreen(uint32_t seconds) {
  face.fillScreen(ST77XX_BLACK);
  face.setTextWrap(false);
  face.setTextColor(ST77XX_CYAN);
  char timeText[6];
  snprintf(timeText, sizeof(timeText), "%02u:%02u",
           (unsigned)(seconds / 60), (unsigned)(seconds % 60));
  face.setTextSize(6);
  // Center the visible 5-character default-font area (174 x 42 pixels).
  face.setCursor(73, 99);
  face.print(timeText);
  tft.drawRGBBitmap(0, 0, face.getBuffer(), 320, 240);
}

void updateAnimation() {
  uint32_t now = millis();
  if (currentExpression == FACE_WAKING) {
    uint32_t elapsed = now - expressionStarted;
    if (elapsed <= 3380 && now - lastEyeFrame >= 80) {
      lastEyeFrame = now;
      drawWakeFrame(elapsed > 3300 ? 3300 : elapsed);
    }
    return;
  }
  if (currentExpression != FACE_IDLE) return;
  if (!blinking) {
    if (now - lastBlinkFinished < BLINK_INTERVAL) return;
    blinking = true;
    blinkStarted = now;
    lastEyeFrame = now - 25;
  }
  if (now - lastEyeFrame < 25) return;
  lastEyeFrame = now;
  uint32_t elapsed = now - blinkStarted;
  int height;
  if (elapsed < CLOSE_TIME) {
    height = 130 - 126 * elapsed / CLOSE_TIME;
  } else if (elapsed < CLOSE_TIME + HOLD_TIME) {
    height = 4;
  } else if (elapsed < CLOSE_TIME + HOLD_TIME + OPEN_TIME) {
    height = 4 + 126 * (elapsed - CLOSE_TIME - HOLD_TIME) / OPEN_TIME;
  } else {
    drawNormalEyes(130);
    blinking = false;
    lastBlinkFinished = millis();
    return;
  }
  drawNormalEyes(height);
}

} // namespace

bool display_init(void) {
  if (ready) return true;
  if (!eyeCanvas.getBuffer() || !face.getBuffer()) return false;
  SPI.begin(TFT_SCK,-1,TFT_MOSI,TFT_CS);
  tft.init(240,320); tft.setSPISpeed(TFT_SPI_HZ); tft.setRotation(1);
  tft.invertDisplay(false); tft.fillScreen(ST77XX_BLACK);
  ready=true; renderFace(FACE_IDLE); return true;
}
void display_set_face(face_t selected) {
  if (!ready || selected<FACE_IDLE || selected>FACE_ACK) return;
  if (selected==currentExpression) return;
  renderFace(selected);
}
void display_show_timer(uint32_t seconds_left) {
  if (!ready) return;
  if (seconds_left>5999) seconds_left=5999;
  bool redraw=currentExpression!=FACE_STUDY || seconds_left!=timerSeconds;
  timerSeconds=seconds_left;
  if (currentExpression!=FACE_STUDY) renderFace(FACE_STUDY);
  else if (redraw) drawStudyScreen(timerSeconds);
}
void display_update(void) { if (ready) updateAnimation(); }
