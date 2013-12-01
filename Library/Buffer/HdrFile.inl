inline HdrFile::HdrFile() : m_width(0), m_height(0), m_data(NULL) {
};

inline HdrFile::HdrFile(const std::string& filename) : m_width(0), m_height(0), m_data(nullptr) {
	load(filename);
};

inline HdrFile::~HdrFile() {
	free();
};

inline void HdrFile::load(const std::string& filename) {
	if (filename.compare(filename.length()-4, 4, std::string(".hdr")) == 0) {
		loadRgbe(filename);
	}
	//else if (filename.compare(filename.length()-4, 4, std::string(".exr")) == 0) {
//		LoadExr(filename);
	//}
	else {
		throw std::invalid_argument("HdrFile::load - unrecognized format");
	}
}

//inline void HdrFile::saveRgbe(const std::string& filename) {
//}
//
//inline void HdrFile::saveExr(const std::string& filename) {
//}

inline void HdrFile::free() {
	if (m_data) {
		delete[] m_data;
	}

	m_data = nullptr;
}

inline int HdrFile::width() const {
	return m_width;
};

inline int HdrFile::height() const {
	return m_height;
};

inline float* HdrFile::data() {
	return m_data;
};

inline const float* HdrFile::data() const {
	return m_data;
};

inline void HdrFile::loadRgbe(const std::string& filename) {
	FILE *f;
	rgbe_header_info header;

	f = fopen(filename.c_str(), "rb");
	if (f == nullptr) {
		throw std::invalid_argument("HdrFile::loadRgbe - cannot open the file");
	}

	RGBE_ReadHeader(f, &m_width, &m_height, &header);
	m_data = new float[3 * m_width * m_height];
	RGBE_ReadPixels_RLE(f, m_data, m_width, m_height);
	fclose(f);
}

//inline void HdrFile::loadExr(const std::string& filename) {
//}
