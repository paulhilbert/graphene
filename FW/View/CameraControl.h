#ifndef CAMERACONTROL_H_
#define CAMERACONTROL_H_

/**
 *  @internal @file CameraControl.h
 *
 *  @brief Contains base class for view control modes.
 *
 */

#include <include/common.h>

#ifdef MODIF_CTRL
#undef MODIF_CTRL
#endif
#ifdef MODIF_SHIFT
#undef MODIF_SHIFT
#endif
#ifdef MODIF_ALT
#undef MODIF_ALT
#endif

namespace FW {
namespace View {

static const int MODIF_CTRL  = 1 << 0;
static const int MODIF_SHIFT = 1 << 1;
static const int MODIF_ALT   = 1 << 2;

/**
 *  @internal CameraControl
 *
 *  @brief Camera control/transformation base class.
 *
 *  Classes derived from this base class can be used in conjunction with
 *  Camera in order to implement camera control modes.
 */
class CameraControl {
	public:
		typedef std::shared_ptr<CameraControl>  Ptr;
		typedef std::weak_ptr<CameraControl>    WPtr;

	public:
		/**
		 *  Constructor.
		 */
		CameraControl();

		/**
		 *  Destructor.
		 */
		virtual ~CameraControl();

		/**
		 *  Get view transformation.
		 *  @return 4x4 matrix used as OpenGL view transformation.
		 */
		Matrix4f getViewTransformation() const;

		/**
		 *  Get "right" direction.
		 *  Given viewing direction and up direction, the "right" is orthogonal to the former ones
		 *  pointing right. More precisely the right direction is the direction projected onto the
		 *  negative x-axis by view transformation.
		 *
		 *  @return 3D vector representing the "right" direction.
		 *  @see getViewDirection()
		 *  @see getUpDirection()
		 */
		Vector3f getRightDirection() const;

		/**
		 *  Get "up" direction.
		 *  Returns direction projected onto z-axis by view transformation.
		 *  @return 3D vector representing the "up" direction.
		 *  @see getViewDirection()
		 *  @see getRightDirection()
		 */
		Vector3f getUpDirection() const;

		/**
		 *  Get view direction.
		 *  Returns direction from camera position to look-at position.
		 *  More precisely the view direction is the direction projected onto the negative y-axis
		 *  by view direction.
		 *  @return 3D vector representing the view direction.
		 *  @see getUpDirection()
		 *  @see getRightDirection()
		 */
		Vector3f getViewDirection() const;

		/**
		 *  Pure virtual function returning the camera position.
		 *  @return 3D vector representing the camera position in world coordinates.
		 *  @see Camera::getPosition()
		 */
		virtual Vector3f getPosition() = 0;

		/**
		 *  Pure virtual function returning the camera look-at position.
		 *  @return 3D vector representing the camera look-at position in world coordinates.
		 *  @see Camera::getLookAt()
		 */
		virtual Vector3f getLookAt() = 0;
		
		/**
		 *  Pure virtual function triggering transformation updates given mouse coordinate changes
		 *  and modifier states.
		 *  @param dX Change of mouse pointer's x-coordinate.
		 *  @param dY Change of mouse pointer's x-coordinate.
		 *  @param mod Modifier state.
		 */
		virtual void  update(int dX, int dY, int mod) = 0;

		/**
		 *  Pure virtual function aligning this control with another control.
		 *  @param other Pointer to CameraControl.
		 */
		virtual void  moveTo(Ptr other) = 0;

	protected:
		// caching variables
		Matrix4f m_viewTransform;
		Matrix4f m_coordsMathToOGL;
		Matrix4f m_coordsOGLToMath;
};

} // VIEW
} // FW

#endif /* CAMERACONTROL_H_ */
