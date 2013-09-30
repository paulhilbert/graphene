#ifndef COLORCONVERSION_H
#define COLORCONVERSION_H

#include <cmath>

#include "Color.h"

namespace Vis {

struct ColorConversion {
	template <class Scalar>
	static RGB  hsv2rgb   (HSV  hsv);
	template <class Scalar>
	static RGBA hsva2rgba (HSVA hsva);
	template <class Scalar>
	static HSV  rgb2hsv   (RGB  rgb);
	template <class Scalar>
	static HSVA rgba2hsva (RGBA rgba);
};

template <class Scalar>
RGB ColorConversion::hsv2rgb(HSV hsv) {
	Scalar a = hsv[0] / (M_PI / 3.0);
	Scalar c = std::floor(a);
	Scalar f = a - c;
	Scalar p = hsv[2] * (1-hsv[1]);
	Scalar q = hsv[2] * (1-hsv[1]*f);
	Scalar t = hsv[2] * (1-hsv[1]*(1-f));

	Scalar r, g, b;

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

template <class Scalar>
RGBA ColorConversion::hsva2rgba(HSVA hsva) {
	Scalar a = hsva[0] / (M_PI / 3.0);
	Scalar c = std::floor(a);
	Scalar f = a - c;
	Scalar p = hsva[2] * (1-hsva[1]);
	Scalar q = hsva[2] * (1-hsva[1]*f);
	Scalar t = hsva[2] * (1-hsva[1]*(1-f));

	Scalar r, g, b;

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

template <class Scalar>
HSV ColorConversion::rgb2hsv(RGB rgb) {
	Scalar min = std::min(std::min(rgb[0], rgb[1]), rgb[2]);
	Scalar max = std::max(std::max(rgb[0], rgb[1]), rgb[2]);

	Scalar h;
	if (min == max) {
		h = 0.0;
	} else if (max == rgb[0]) {
		h = M_PI / 3.0 * ((rgb[1]-rgb[2]) / (max-min));
	} else if (max == rgb[1]) {
		h = M_PI / 3.0 * (2.0 + (rgb[2]-rgb[0]) / (max-min));
	} else {
		h = M_PI / 3.0 * (4.0 + (rgb[0]-rgb[1]) / (max-min));
	}
	if (h < 0.0) {
		h += M_PI * 2.0;
	}

	Scalar s;
	if (max == 0.0) {
		s = 0.0;
	} else {
		s = (max-min) / max;
	}

	Scalar v = max;

	return HSV(h,s,v);
}

template <class Scalar>
HSVA ColorConversion::rgba2hsva(RGBA rgba) {
	Scalar min = std::min(std::min(rgba[0], rgba[1]), rgba[2]);
	Scalar max = std::max(std::max(rgba[0], rgba[1]), rgba[2]);

	Scalar h;
	if (min == max) {
		h = 0.0;
	} else if (max == rgba[0]) {
		h = M_PI / 3.0 * ((rgba[1]-rgba[2]) / (max-min));
	} else if (max == rgba[1]) {
		h = M_PI / 3.0 * (2.0 + (rgba[2]-rgba[0]) / (max-min));
	} else {
		h = M_PI / 3.0 * (4.0 + (rgba[0]-rgba[1]) / (max-min));
	}
	if (h < 0.0) {
		h += static_cast<float>(M_PI * 2.0);
	}

	Scalar s;
	if (max == 0.0) {
		s = 0.0;
	} else {
		s = (max-min) / max;
	}

	Scalar v = max;

	return HSVA(h,s,v,rgba[3]);
}

} /* Vis */

#endif // COLORCONVERSION_H
