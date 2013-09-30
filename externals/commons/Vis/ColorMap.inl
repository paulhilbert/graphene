template <class Scalar>
ColorMap<RGB<Scalar>, Scalar> rgbJet() {
	ColorMap<RGBA<Scalar>, Scalar> rgbaMap = rgbaJet<Scalar>();
	ColorMap<RGB<Scalar>, Scalar> map = [=] (Scalar value) {
		RGBA<Scalar> rgba = rgbaMap(value);
		return RGB<Scalar>(rgba[0], rgba[1], rgba[2]);
	};
	return map;
}

template <class Scalar>
ColorMap<RGBA<Scalar>, Scalar> rgbaJet() {
	ColorMap<RGBA<Scalar>, Scalar> map = [] (Scalar value) {
		Scalar fourValue = 4.f * value;
		Scalar red   = std::min(fourValue - 1.5, -fourValue + 4.5);
		Scalar green = std::min(fourValue - 0.5, -fourValue + 3.5);
		Scalar blue  = std::min(fourValue + 0.5, -fourValue + 2.5);

		RGBA<Scalar> result(red, green, blue, Scalar(1));
		Eigen::clamp(result, Scalar(0), Scalar(1));
		return result;
	};
	return map;
}

template <class Scalar>
ColorMap<HSV<Scalar>, Scalar> hsvJet() {
	ColorMap<RGB<Scalar>, Scalar> rgbMap = rgbJet<Scalar>();
	ColorMap<HSV<Scalar>, Scalar> map = [=] (Scalar value) {
		RGB<Scalar> rgb = rgbMap(value);
		return ColorConversion::rgb2hsv<Scalar>(rgb);
	};
	return map;
}

template <class Scalar>
ColorMap<HSVA<Scalar>, Scalar> hsvaJet() {
	ColorMap<RGBA<Scalar>, Scalar> rgbaMap = rgbaJet<Scalar>();
	ColorMap<HSVA<Scalar>, Scalar> map = [=] (Scalar value) {
		RGBA<Scalar> rgba = rgbaMap(value);
		return ColorConversion::rgba2hsva<Scalar>(rgba);
	};
	return map;
}
