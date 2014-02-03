#ifndef COLORCONVERSION_H
#define COLORCONVERSION_H

#include <cmath>

#include "Types.h"

namespace Colors {

struct Conversion {
//	template <class Scalar>
	static RGB  hsv2rgb   (HSV  hsv);
//	template <class Scalar>
	static RGBA hsva2rgba (HSVA hsva);
//	template <class Scalar>
	static HSV  rgb2hsv   (RGB  rgb);
//	template <class Scalar>
	static HSVA rgba2hsva (RGBA rgba);

	static RGB rgbHue(RGB rgb, float h);
	static RGBA rgbaHue(RGBA rgba, float h);
	static RGB rgbSaturation(RGB rgb, float s);
	static RGBA rgbaSaturation(RGBA rgba, float s);
	static RGB rgbValue(RGB rgb, float v);
	static RGBA rgbaValue(RGBA rgba, float v);
};

//template <class Scalar>
inline RGB Conversion::hsv2rgb(HSV hsv) {
	float a = hsv[0] / (static_cast<float>(M_PI) / 3.f);
	float c = std::floor(a);
	float f = a - c;
	float p = hsv[2] * (1-hsv[1]);
	float q = hsv[2] * (1-hsv[1]*f);
	float t = hsv[2] * (1-hsv[1]*(1-f));

	float r, g, b;

	if (c == 1) {
		r = q; g = hsv[2]; b = p;
	} else if (c == 2) {
		r = p; g = hsv[2]; b = t;
	} else if (c == 3) {
		r = p; g = q; b = hsv[2];
	} else if (c == 4) {
		r = t; g = p; b = hsv[2];
	} else if (c == 5) {
		r = hsv[2]; g = p; b = q;
	} else {
		r = hsv[2]; g = t; b = p;
	}

	return RGB(r,g,b);
}

//template <class Scalar>
inline RGBA Conversion::hsva2rgba(HSVA hsva) {
	float a = hsva[0] / (static_cast<float>(M_PI) / 3.f);
	float c = std::floor(a);
	float f = a - c;
	float p = hsva[2] * (1-hsva[1]);
	float q = hsva[2] * (1-hsva[1]*f);
	float t = hsva[2] * (1-hsva[1]*(1-f));

	float r, g, b;

	if (c == 1) {
		r = q; g = hsva[2]; b = p;
	} else if (c == 2) {
		r = p; g = hsva[2]; b = t;
	} else if (c == 3) {
		r = p; g = q; b = hsva[2];
	} else if (c == 4) {
		r = t; g = p; b = hsva[2];
	} else if (c == 5) {
		r = hsva[2]; g = p; b = q;
	} else {
		r = hsva[2]; g = t; b = p;
	}

	return RGBA(r,g,b,hsva[3]);
}

//template <class Scalar>
inline HSV Conversion::rgb2hsv(RGB rgb) {
	float min = std::min(std::min(rgb[0], rgb[1]), rgb[2]);
	float max = std::max(std::max(rgb[0], rgb[1]), rgb[2]);

	float h;
	if (min == max) {
		h = 0.0;
	} else if (max == rgb[0]) {
		h = static_cast<float>(M_PI) / 3.f * ((rgb[1]-rgb[2]) / (max-min));
	} else if (max == rgb[1]) {
		h = static_cast<float>(M_PI) / 3.f * (2.f + (rgb[2]-rgb[0]) / (max-min));
	} else {
		h = static_cast<float>(M_PI) / 3.f * (4.f + (rgb[0]-rgb[1]) / (max-min));
	}
	if (h < 0.0) {
		h += static_cast<float>(M_PI) * 2.f;
	}

	float s;
	if (max == 0.0) {
		s = 0.0;
	} else {
		s = (max-min) / max;
	}

	float v = max;

	return HSV(h,s,v);
}

//template <class Scalar>
inline HSVA Conversion::rgba2hsva(RGBA rgba) {
	float min = std::min(std::min(rgba[0], rgba[1]), rgba[2]);
	float max = std::max(std::max(rgba[0], rgba[1]), rgba[2]);

	float h;
	if (min == max) {
		h = 0.0;
	} else if (max == rgba[0]) {
		h = static_cast<float>(M_PI) / 3.f * ((rgba[1]-rgba[2]) / (max-min));
	} else if (max == rgba[1]) {
		h = static_cast<float>(M_PI) / 3.f * (2.f + (rgba[2]-rgba[0]) / (max-min));
	} else {
		h = static_cast<float>(M_PI) / 3.f * (4.f + (rgba[0]-rgba[1]) / (max-min));
	}
	if (h < 0.0) {
		h += static_cast<float>(M_PI) * 2.f;
	}

	float s;
	if (max == 0.0) {
		s = 0.0;
	} else {
		s = (max-min) / max;
	}

	float v = max;

	return HSVA(h,s,v,rgba[3]);
}

inline RGB Conversion::rgbHue(RGB rgb, float h) {
	auto hsv = rgb2hsv(rgb);
	hsv[0] = h;
	return hsv2rgb(hsv);
}

inline RGBA Conversion::rgbaHue(RGBA rgba, float h) {
	rgba.head(3) = rgbHue(rgba.head(3), h);
	return rgba;
}

inline RGB Conversion::rgbSaturation(RGB rgb, float s) {
	auto hsv = rgb2hsv(rgb);
	hsv[1] = s;
	return hsv2rgb(hsv);
}

inline RGBA Conversion::rgbaSaturation(RGBA rgba, float s) {
	rgba.head(3) = rgbSaturation(rgba.head(3), s);
	return rgba;
}

inline RGB Conversion::rgbValue(RGB rgb, float v) {
	auto hsv = rgb2hsv(rgb);
	hsv[2] = v;
	return hsv2rgb(hsv);
}

inline RGBA Conversion::rgbaValue(RGBA rgba, float v) {
	rgba.head(3) = rgbValue(rgba.head(3), v);
	return rgba;
}


} /* Colors */

#endif // COLORCONVERSION_H
