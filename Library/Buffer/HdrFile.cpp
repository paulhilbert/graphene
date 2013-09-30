#include <stdexcept>
#include <string>
#include <cstdio>

#include "HdrFile.h"
extern "C" {
#include "rgbe/rgbe.h"
}

using std::string;
using std::invalid_argument;

#include <iostream>
using std::cout;
using std::endl;

namespace Buffer {


void HdrFile::Load(const string& filename) {
	if (filename.compare(filename.length()-4, 4, string(".hdr")) == 0) {
		LoadRgbe(filename);
	}
	else if (filename.compare(filename.length()-4, 4, string(".exr")) == 0) {
//		LoadExr(filename);
	}
	else {
		throw invalid_argument("HdrFile::Load - unrecognized format");
	}
}


void HdrFile::LoadRgbe(const string& filename) {
	FILE *f;
	rgbe_header_info header;

	f = fopen(filename.c_str(), "rb");
	if (f == NULL) {
		throw invalid_argument("HdrFile::LoadRgbe - cannot open the file");
	}

	RGBE_ReadHeader(f, &_width, &_height, &header);
	_image = new float[3 * _width * _height];
	RGBE_ReadPixels_RLE(f, _image, _width, _height);
	fclose(f);

	/*
	cout << "Name: " << filename << endl;
	cout << "Dimensions: " << _width << "x" << _height << endl;
	cout << "Valid: " << header.valid << endl;
	cout << "Program type: " << header.programtype << endl;
	cout << "Gamma: " << header.gamma << endl;
	cout << "Exposure: " << header.exposure << endl;
	*/
}


//void HdrFile::LoadExr(const string& filename)
//{
//}


void HdrFile::Free() {
	if (_image) {
		delete[] _image;
	}

	_image = NULL;
}

} // Buffer
