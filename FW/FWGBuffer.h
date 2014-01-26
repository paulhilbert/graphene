#ifndef FWGBUFFER_H_
#define FWGBUFFER_H_

#include <memory>
#include <include/ogl.h>

namespace FW {

class GBuffer {
	public:
		typedef std::shared_ptr<GBuffer>        Ptr;
		typedef std::weak_ptr<GBuffer>          WPtr;
		typedef std::shared_ptr<const GBuffer>  ConstPtr;
		typedef std::weak_ptr<const GBuffer>    ConstWPtr;

		enum TEX_TYPE {
			TEX_POSITION,
			TEX_COLOR,
			TEX_NORMAL,
			NUM_TEX
		};

	public:
		GBuffer();
		GBuffer(int width, int height);
		virtual ~GBuffer();

		void init(int width, int height);

		bool initialized() const;

		void bindWrite();
		void bindRead();
		void release();

		void blitTo(TEX_TYPE tex, GLsizei x, GLsizei y, GLsizei w, GLsizei h);

	protected:
		void clearBuffers();

	protected:
		int   m_width;
		int   m_height;
		bool  m_initialized;

		GLuint  m_fbo;
		GLuint  m_tex[NUM_TEX];
		GLuint  m_depth;
};


} // FW

#endif /* FWGBUFFER_H_ */
