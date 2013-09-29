#ifndef VISCOLORS_H_
#define VISCOLORS_H_

#include "Color.h"

namespace Vis {

namespace Colors {

// rgb colors
inline RGB<float>  rgbRed       () { return RGB<float> (1.f, 0.f, 0.f); }
inline RGB<float>  rgbGreen     () { return RGB<float> (0.f, 1.f, 0.f); }
inline RGB<float>  rgbBlue      () { return RGB<float> (0.f, 0.f, 1.f); }
inline RGB<float>  rgbYellow    () { return RGB<float> (1.f, 1.f, 0.f); }
inline RGB<float>  rgbCyan      () { return RGB<float> (0.f, 1.f, 1.f); }
inline RGB<float>  rgbMagenta   () { return RGB<float> (1.f, 0.f, 1.f); }
inline RGB<float>  rgbWhite     () { return RGB<float> (1.f, 1.f, 1.f); }
inline RGB<float>  rgbBlack     () { return RGB<float> (0.f, 0.f, 0.f); }
inline RGB<float>  rgbGrey      () { return RGB<float> (0.5f, 0.5f, 0.5f); }
inline RGB<float>  rgbLightGrey () { return RGB<float> (0.7f, 0.7f, 0.7f); }
inline RGB<float>  rgbDarkGrey  () { return RGB<float> (0.3f, 0.3f, 0.3f); }

// rgba colors
inline RGBA<float> rgbaRed       (float alpha = 1.f) { return RGBA<float>(1.f, 0.f, 0.f, alpha); }
inline RGBA<float> rgbaGreen     (float alpha = 1.f) { return RGBA<float>(0.f, 1.f, 0.f, alpha); }
inline RGBA<float> rgbaBlue      (float alpha = 1.f) { return RGBA<float>(0.f, 0.f, 1.f, alpha); }
inline RGBA<float> rgbaYellow    (float alpha = 1.f) { return RGBA<float>(1.f, 1.f, 0.f, alpha); }
inline RGBA<float> rgbaCyan      (float alpha = 1.f) { return RGBA<float>(0.f, 1.f, 1.f, alpha); }
inline RGBA<float> rgbaMagenta   (float alpha = 1.f) { return RGBA<float>(1.f, 0.f, 1.f, alpha); }
inline RGBA<float> rgbaWhite     (float alpha = 1.f) { return RGBA<float>(1.f, 1.f, 1.f, alpha); }
inline RGBA<float> rgbaBlack     (float alpha = 1.f) { return RGBA<float>(0.f, 0.f, 0.f, alpha); }
inline RGBA<float> rgbaGrey      (float alpha = 1.f) { return RGBA<float>(0.5f, 0.5f, 0.5f, alpha); }
inline RGBA<float> rgbaLightGrey (float alpha = 1.f) { return RGBA<float>(0.7f, 0.7f, 0.7f, alpha); }
inline RGBA<float> rgbaDarkGrey  (float alpha = 1.f) { return RGBA<float>(0.3f, 0.3f, 0.3f, alpha); }

inline RGBA<float> rgbaInvisible ()                  { return RGBA<float>(0.f, 0.f, 0.f, 0.f);; }




} // Vis
} // Colors

#endif /* VISCOLORS_H_ */
