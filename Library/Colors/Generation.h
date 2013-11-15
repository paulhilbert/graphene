#ifndef COLORGENERATION_H
#define COLORGENERATION_H

#include <vector>
#include <algorithm>
#include <random>
#include <chrono>

#include "Conversion.h"
#include <Library/Random/RNG.h>
using Random::RNG;

namespace Colors {

struct Generation {
	static RGB  randomHueRGB();
	static RGBA randomHueRGBA();
	static std::vector<RGB>  randomHuesRGB(unsigned int count);
	static std::vector<RGBA> randomHuesRGBA(unsigned int count);
	static std::vector<RGB>  uniformHuesRGB(unsigned int count);
	static std::vector<RGBA> uniformHuesRGBA(unsigned int count);
};


inline RGB Generation::randomHueRGB() {
	float hue = RNG::uniformAB(0.f, static_cast<float>(2.f*M_PI));
	return Conversion::hsv2rgb(HSV(hue, 1.f, 1.f));
}

inline RGBA Generation::randomHueRGBA() {
	RGBA rgba;
	rgba << randomHueRGB(), 1.f;
	return rgba;
}

inline std::vector<RGB> Generation::randomHuesRGB(unsigned int count) {
	std::vector<RGB> result;
	std::generate_n(std::back_inserter(result), count, &Generation::randomHueRGB);
	return result;
}

inline std::vector<RGBA> Generation::randomHuesRGBA(unsigned int count) {
	std::vector<RGBA> result;
	std::generate_n(std::back_inserter(result), count, &Generation::randomHueRGBA);
	return result;
}

inline std::vector<RGB> Generation::uniformHuesRGB(unsigned int count) {
	float hueStep = static_cast<float>(2.0*M_PI / count);
	std::vector<RGB> result;
	for (unsigned int i=0; i<count; ++i) result.push_back(Conversion::hsv2rgb(HSV(hueStep*i, 1.f, 1.f)));
	return result;
}

inline std::vector<RGBA> Generation::uniformHuesRGBA(unsigned int count) {
	float hueStep = static_cast<float>(2.0*M_PI / count);
	std::vector<RGBA> result;
	for (unsigned int i=0; i<count; ++i) result.push_back(Conversion::hsva2rgba(HSVA(hueStep*i, 1.f, 1.f, 1.f)));
	return result;
}

} /* Colors */

#endif // COLORGENERATION_H
