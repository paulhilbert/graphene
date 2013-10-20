#ifndef ORBITCAMERACONTROL_H_
#define ORBITCAMERACONTROL_H_

#include <include/common.h>

#include "CameraControl.h"

namespace FW {
namespace View {

class OrbitCameraControl : public CameraControl {
	public:
		typedef std::shared_ptr<OrbitCameraControl>  Ptr;
		typedef std::weak_ptr<OrbitCameraControl>    WPtr;

	public:
		OrbitCameraControl();
		virtual ~OrbitCameraControl();

		Eigen::Vector3f getPosition();
		Eigen::Vector3f getLookAt();

		void  update(int dX, int dY, int mod);
		void  moveTo(std::shared_ptr<OrbitCameraControl> other);
		void  moveTo(std::shared_ptr<CameraControl> other);

	protected:
		void pan(float dX, float dY, bool stayInPlane = false);
		void zoom(float delta);
		void rotP(float delta);
		void rotT(float delta);
		void determineMatrix();

		// state variables
		float            m_theta;
		float            m_phi;
		float            m_radius;
		Eigen::Vector3f  m_center;
};


} // View
} // FW

#endif /* ORBITCAMERACONTROL_H_ */
