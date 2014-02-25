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
	typedef std::pair<float, float> Range;

	static RGB  randomHueRGB();
	static RGBA randomHueRGBA();
	static std::vector<RGB>  randomHuesRGB(unsigned int count);
	static std::vector<RGBA> randomHuesRGBA(unsigned int count);
	static std::vector<RGB>  uniformHuesRGB(unsigned int count);
	static std::vector<RGBA> uniformHuesRGBA(unsigned int count);
	static std::vector<RGBA> uniformColorsRGBA(unsigned int count, const Range& hueRange = {0.f, static_cast<float>(2*M_PI)}, const Range& satRange = {0.5f, 1.f}, const Range& lightRange = {1.f, 1.f}, float alpha = 1.f);
	static std::vector<RGBA> shuffledColorsRGBA(unsigned int count, const Range& hueRange = {0.f, static_cast<float>(2*M_PI)}, const Range& satRange = {0.5f, 1.f}, const Range& lightRange = {1.f, 1.f}, float alpha = 1.f);
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

inline std::vector<RGBA> Generation::uniformColorsRGBA(unsigned int count, const Range& hueRange, const Range& satRange, const Range& lightRange, float alpha) {
	float rHue = hueRange.second - hueRange.first;
	float rSat = satRange.second - satRange.first;
	float rLight = lightRange.second - lightRange.first;
	if (rHue < 0.f || rSat < 0.f || rLight < 0.f) {
		throw std::runtime_error("Generation::uniformColorsRGBA: Invalid range input.");
	}
	float sum = rHue/(2.f*M_PI) + rSat + rLight;
	Vector3f w(rHue / (2.f * M_PI * sum), rSat / sum, rLight / sum);
	float eps = Eigen::NumTraits<float>::dummy_precision();
	float prod = (w[0] < eps ? 1.f : w[0]) * (w[1] < eps ? 1.f : w[1]) * (w[2] < eps ? 1.f : w[2]);

	float factor = cbrt(count) / cbrt(prod);
	for (int i=0; i<3; ++i) {
		w[i] = std::max(w[i] * factor, 1.f);
	}
	unsigned int cS = std::floor(w[1]);
	unsigned int cL = std::floor(w[2]);
	unsigned int cH = std::ceil(static_cast<float>(count) / (cS*cL));

	float sH = rHue / (cH > 0.f ? cH : 1.f);
	float sS = rSat / (cS > 1.f ? cS - 1.f : 1.f);
	float sL = rLight / (cL > 1.f ? cL - 1.f : 2.f);
	std::vector<RGBA> result;
	for (unsigned int i=0; i<cH; ++i) {
		float h = hueRange.first + static_cast<float>(i) * sH;
		for (unsigned int j=0; j<cS; ++j) {
			float s = satRange.first + static_cast<float>(j) * sS;
			for (unsigned int k=0; k<cL; ++k) {
				float l = lightRange.first + static_cast<float>(k) * sL;
				HSVA hsva(h, s, l, alpha);
				RGBA rgba = Conversion::hsva2rgba(hsva);
				result.push_back(rgba);
			}
		}
	}
	result.resize(count);
	return result;
}

inline std::vector<RGBA> Generation::shuffledColorsRGBA(unsigned int count, const Range& hueRange, const Range& satRange, const Range& lightRange, float alpha) {
	std::vector<RGBA> result = uniformColorsRGBA(count, hueRange, satRange, lightRange, alpha);
	std::random_shuffle(result.begin(), result.end());
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::shuffle(result.begin(), result.end(), std::default_random_engine(seed));
	return result;
}

} /* Colors */

#endif // COLORGENERATION_H
