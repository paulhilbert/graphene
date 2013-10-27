#ifndef CLASS_HDR_FILE
#define CLASS_HDR_FILE

#include <stdexcept>
#include <string>

namespace Buffer {

class HdrFile {
	private:
		int _width;
		int _height;
		float *_image;

		void LoadRgbe(const std::string& filename);
		void LoadExr(const std::string& filename);
	public:
		/// Default Constructor
		HdrFile() : _width(0), _height(0), _image(NULL) { };
		/// Constructor from file
		HdrFile(const std::string& filename) : _width(0), _height(0), _image(NULL) { Load(filename); };
		/// Destructor
		~HdrFile() { Free(); };
		/// Load a file in the buffer
		void Load(const std::string& filename);
		/// Save the buffer to a Radiance file
		void SaveRgbe(const std::string& filename);
		/// Save the buffer to a OpenEXR file
		void SaveExr(const std::string& filename);
		/// Release the buffer
		void Free();
		/// Width getter
		int Width() { return _width; };
		/// Height getter
		int Height() { return _height; };
		/// Image data getter
		float *Image() { return _image; };
};

} // Buffer

#endif
