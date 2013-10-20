#ifndef CAMERA_H_
#define CAMERA_H_

#include <include/common.h>

#include <FW/Events/EventHandler.h>
#include <FW/View/CameraControl.h>
#include <Geometry/Ray.h>
#include "Transforms.h"

namespace FW {
namespace View {

class Camera {
	public:
		typedef std::shared_ptr<Camera> Ptr;
		typedef std::weak_ptr<Camera>   WPtr;

	public:
		Camera(CameraControl::Ptr control, Events::EventHandler::Ptr eventHandler, Transforms::WPtr transforms);
		virtual ~Camera();

		Eigen::Vector3f getPosition() const;
		Eigen::Vector3f getLookAt() const;

		void toggleOrtho();
		void setOrtho(bool active);

		bool getOrtho() const;
		float getOrthoScale() const;

		void setControl(CameraControl::Ptr control);
		CameraControl::Ptr getControl();

		Geometry::Ray::Ptr  getPickRay();
		void                updatePickRay(int x, int y);

		void             updateTransforms();

	protected:
		Eigen::Matrix4f  getProjectionMatrix(int w, int h);
		void registerEvents();

		CameraControl::Ptr         m_control;
		Events::EventHandler::Ptr  m_eventHandler;
		Transforms::WPtr           m_transforms;
		Geometry::Ray::Ptr         m_pickRay;

		bool           m_ortho;
};

} // View
} // FW

#endif /* CAMERA_H_ */
