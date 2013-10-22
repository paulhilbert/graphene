#ifndef ORBITCAMERACONTROL_H_
#define ORBITCAMERACONTROL_H_

#include <include/common.h>

#include "CameraControl.h"

namespace FW {
namespace View {

/**
 *  @internal OrbitCameraControl
 *
 *  @brief Camera orbit control class.
 *
 *  Classes derived from CameraControl in order to implement orbit camera control mode.
 */
class OrbitCameraControl : public CameraControl {
	public:
		typedef std::shared_ptr<OrbitCameraControl>  Ptr;
		typedef std::weak_ptr<OrbitCameraControl>    WPtr;

	public:
		/**
		 *  Constructor.
		 */
		OrbitCameraControl();

		/**
		 *  Destructor.
		 */
		virtual ~OrbitCameraControl();

		/**
		 *  Returns the camera position.
		 *  @return 3D vector representing the camera position in world coordinates.
		 *  @see Camera::getPosition()
		 *  @see CameraControl::getPosition()
		 */
		Vector3f getPosition();

		/**
		 *  Returns the camera look-at position.
		 *  @return 3D vector representing the camera look-at position in world coordinates.
		 *  @see Camera::getLookAt()
		 *  @see CameraControl::getLookAt()
		 */
		Vector3f getLookAt();

		/**
		 *  Trigger transformation updates given mouse coordinate changes
		 *  and modifier states.
		 *  @param dX Change of mouse pointer's x-coordinate.
		 *  @param dY Change of mouse pointer's x-coordinate.
		 *  @param mod Modifier state.
		 *  @see CameraControl::update(int dX, int dY, int mod)
		 */
		void  update(int dX, int dY, int mod);

		/**
		 *  Align this control with another orbit control.
		 *  @param other Pointer to OrbitCameraControl.
		 */
		void  moveTo(std::shared_ptr<OrbitCameraControl> other);

		/**
		 *  Align this control with another camera control.
		 *  @param other Pointer to CameraControl.
		 */
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
