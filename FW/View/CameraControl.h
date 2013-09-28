#ifndef CAMERACONTROL_H_
#define CAMERACONTROL_H_

#include <memory>

#include <Eigen/Dense>

#ifdef MOD_CTRL
#undef MOD_CTRL
#endif
#ifdef MOD_SHIFT
#undef MOD_SHIFT
#endif
#ifdef MOD_ALT
#undef MOD_ALT
#endif

namespace FW {
namespace View {

static const int MOD_CTRL  = 1 << 0;
static const int MOD_SHIFT = 1 << 1;
static const int MOD_ALT   = 1 << 2;

class CameraControl {
	public:
		typedef std::shared_ptr<CameraControl>  Ptr;
		typedef std::weak_ptr<CameraControl>    WPtr;

	public:
		CameraControl();
		virtual ~CameraControl();

		Eigen::Matrix4f getViewTransformation() const;
		Eigen::Vector3f getRightDirection() const;
		Eigen::Vector3f getUpDirection() const;
		Eigen::Vector3f getViewDirection() const;

		virtual Eigen::Vector3f getPosition() = 0;
		virtual Eigen::Vector3f getLookAt() = 0;
		
		virtual void  update(int dX, int dY, int mod) = 0;
		virtual void  moveTo(Ptr other) = 0;

	protected:
		// caching variables
		Eigen::Matrix4f m_viewTransform;
		Eigen::Matrix4f m_coordsMathToOGL;
		Eigen::Matrix4f m_coordsOGLToMath;
};

} // VIEW
} // FW

#endif /* CAMERACONTROL_H_ */
