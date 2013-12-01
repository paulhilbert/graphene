#ifndef _HDRFILE_H_
#define _HDRFILE_H_

#include <stdexcept>
#include <string>
#include <cstdio>

#include "HdrFile.h"
extern "C" {
#include "rgbe/rgbe.h"
}

namespace Buffer {

class HdrFile {
	public:
		typedef std::shared_ptr<HdrFile>        Ptr;
		typedef std::weak_ptr<HdrFile>          WPtr;
		typedef std::shared_ptr<const HdrFile>  ConstPtr;
		typedef std::weak_ptr<const HdrFile>    ConstWPtr;

	public:
		HdrFile();
		HdrFile(const std::string& filename);
		~HdrFile();

		void load(const std::string& filename);
		//void saveRgbe(const std::string& filename);
		//void saveExr(const std::string& filename);

		void free();

		int width() const;
		int height() const;

		float* data();
		const float* data() const;

	protected:
		void loadRgbe(const std::string& filename);
		//void loadExr(const std::string& filename);

	protected:
		int    m_width;
		int    m_height;
		float* m_data;

};

#include "HdrFile.inl"

} // Buffer

#endif // _HDRFILE_H_
