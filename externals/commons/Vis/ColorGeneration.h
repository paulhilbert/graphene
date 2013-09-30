#ifndef COLORGENERATION_H
#define COLORGENERATION_H

#include <vector>
#include <algorithm>

#include "../Random/RNG.h"
using namespace Random;

#include "ColorConversion.h"

namespace Vis {

struct ColorGeneration {
	template <class Scalar>
	static RGB<Scalar>  randomHueRGB();
	template <class Scalar>
	static RGBA<Scalar> randomHueRGBA();
	template <class Scalar>
	static std::vector<RGB<Scalar>>  randomHuesRGB(unsigned int count);
	template <class Scalar>
	static std::vector<RGBA<Scalar>> randomHuesRGBA(unsigned int count);
	template <class Scalar>
	static std::vector<RGB<Scalar>>  uniformHuesRGB(unsigned int count);
	template <class Scalar>
	static std::vector<RGBA<Scalar>> uniformHuesRGBA(unsigned int count);
};


template <class Scalar>
RGB<Scalar> ColorGeneration::randomHueRGB() {
	RNG* rng = RNG::instance();
	Scalar hue = rng->uniformAB<Scalar>(0.f, 2.f*M_PI);
	return ColorConversion::hsv2rgb(HSV<Scalar>(hue, 1.f, 1.f));
}

template <class Scalar>
RGBA<Scalar> ColorGeneration::randomHueRGBA() {
	RGBA<Scalar> rgba;
	rgba << randomHueRGB<Scalar>(), 1.f;
	return rgba;
}

template <class Scalar>
std::vector<RGB<Scalar>> ColorGeneration::randomHuesRGB(unsigned int count) {
	std::vector<RGB<Scalar>> result;
	std::generate_n(std::back_inserter(result), count, &ColorGeneration::randomHueRGB<Scalar>);
	return result;
}

template <class Scalar>
std::vector<RGBA<Scalar>> ColorGeneration::randomHuesRGBA(unsigned int count) {
	std::vector<RGBA<Scalar>> result;
	std::generate_n(std::back_inserter(result), count, &ColorGeneration::randomHueRGBA<Scalar>);
	return result;
}

template <class Scalar>
std::vector<RGB<Scalar>> ColorGeneration::uniformHuesRGB(unsigned int count) {
	Scalar hueStep = Scalar(2)*M_PI / static_cast<Scalar>(count);
	std::vector<RGB<Scalar>> result;
	for (unsigned int i=0; i<count; ++i) result.push_back(ColorConversion::hsv2rgb(HSV<Scalar>(hueStep*i, 1.f, 1.f)));
	return result;
}

template <class Scalar>
std::vector<RGBA<Scalar>> ColorGeneration::uniformHuesRGBA(unsigned int count) {
	Scalar hueStep = Scalar(2)*M_PI / static_cast<Scalar>(count);
	std::vector<RGBA<Scalar>> result;
	for (unsigned int i=0; i<count; ++i) result.push_back(ColorConversion::hsva2rgba(HSVA<Scalar>(hueStep*i, 1.f, 1.f, 1.f)));
	return result;
}

} /* Vis */

#endif // COLORGENERATION_H
