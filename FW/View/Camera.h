#ifndef CAMERA_H_
#define CAMERA_H_

#include <include/common.h>

#include <FW/Events/EventHandler.h>
#include <FW/View/CameraControl.h>
#include <Geometry/Ray.h>
#include "Transforms.h"

namespace FW {
namespace View {

/**
 *  Management class for camera parameters and transformations.
 *
 *  This class uses the CameraControl class in order to manage
 *  camera transformation modules.
 *  It is primarily used by the core graphene code and therefore
 *  cannot be accessed from visualizer code.
 *
*/
class Camera {
	public:
		typedef std::shared_ptr<Camera> Ptr;
		typedef std::weak_ptr<Camera>   WPtr;

	public:
		/**
		 *  Constructor which is given initial control and access pointers.
		 *  @param control Initial camera control to use as camera transformation.
		 *  @param eventHandler Access pointer to central Events::EventHandler class.
		 *  @param transforms Access weak-pointer to central Transforms class.
		 */
		Camera(CameraControl::Ptr control, Events::EventHandler::Ptr eventHandler, Transforms::WPtr transforms);

		/**
		 *  Destructor.
		 */
		virtual ~Camera();

		/**
		 *  Get camera position.
		 *  Returns camera position in world coordinates (i.e. those coordinates transformed to (0, 0, 0) by modelview matrix)
		 *  @return 3D vector representing the camera position in world coordinates.
		 */
		Vector3f getPosition() const;

		/**
		 *  Get camera look-at position.
		 *  Returns look-at position in world coordinates (e.g. those coordinates OrbitCameraControl orbits around).
		 *  @return 3D vector representing the camera look-at position in world coordinates.
		 */
		Vector3f getLookAt() const;

		/**
		 *  Toggle between orthogonal and perspective projection.
		 */
		void toggleOrtho();

		/**
		 *  Set projection method.
		 *  @param active If true, orthogonal projection is enabled. Otherwise perspective transformation will be used.
		 */
		void setOrtho(bool active);

		/**
		 *  Get projection method.
		 *  @return If trie, orthogonal projection is enabled. Otherwise perspective transformation will be used.
		 */
		bool getOrtho() const;

		/**
		 *  Set active camera control.
		 *  @param control Pointer to class inheriting CameraControl to be used as control mode.
		 */
		void setControl(CameraControl::Ptr control);

		/**
		 *  Get active camera control.
		 *  @return Pointer to class inheriting CameraControl used as control mode.
		 */
		CameraControl::Ptr getControl();

		/**
		 *  Get current pick ray.
		 *  Returns pointer to Geometry::Ray class representing the pick ray (i.e. given (x,y) coordinates the mouse pointer is over,
		 *  ray through the corresponding points in near and far plane).
		 *  @return Pointer to Geometry::Ray representing the pick ray.
		 */
		Geometry::Ray::Ptr getPickRay();

		/**
		 *  Trigger transformation update.
		 *  On call all transformation matrices and corresponding access classes are updated.
		 */
		void updateTransforms();

	protected:
		void updatePickRay(int x, int y);

	protected:
		Matrix4f getProjectionMatrix(int w, int h);
		void registerEvents();

	protected:
		CameraControl::Ptr         m_control;
		Events::EventHandler::Ptr  m_eventHandler;
		Transforms::WPtr           m_transforms;
		Geometry::Ray::Ptr         m_pickRay;

		bool                       m_ortho;
};

} // View
} // FW

#endif /* CAMERA_H_ */
