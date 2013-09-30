#include "CameraControl.h"

namespace FW {
namespace View {

CameraControl::CameraControl() {
	m_coordsMathToOGL << 1.f, 0.f, 0.f, 0.f,
	                     0.f, 0.f, 1.f, 0.f,
	                     0.f, -1.f, 0.f, 0.f,
	                     0.f, 0.f, 0.f, 1.f;
	m_coordsOGLToMath << 1.f, 0.f, 0.f, 0.f,
	                     0.f, 0.f, -1.f, 0.f,
	                     0.f, 1.f, 0.f, 0.f,
	                     0.f, 0.f, 0.f, 1.f;
}

CameraControl::~CameraControl() {
}

Eigen::Matrix4f CameraControl::getViewTransformation() const {
	return m_viewTransform * m_coordsMathToOGL;
}

Eigen::Vector3f CameraControl::getRightDirection() const {
	return m_coordsOGLToMath.block<3,3>(0,0)*(m_viewTransform.transpose().block<3,1>(0,0));
}

Eigen::Vector3f CameraControl::getUpDirection() const {
	return m_coordsOGLToMath.block<3,3>(0,0)*(m_viewTransform.transpose().block<3,1>(0,1));
}

Eigen::Vector3f CameraControl::getViewDirection() const {
	return m_coordsOGLToMath.block<3,3>(0,0)*(-m_viewTransform.transpose().block<3,1>(0,2));
}

} // View
} // FW
