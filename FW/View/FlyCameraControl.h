#ifndef FLYCAMERACONTROL_H_
#define FLYCAMERACONTROL_H_

#include <include/common.h>

#include "CameraControl.h"

namespace FW {
namespace View {

class FlyCameraControl : public CameraControl {
	public:
		typedef std::shared_ptr<FlyCameraControl>  Ptr;
		typedef std::weak_ptr<FlyCameraControl>    WPtr;

	public:
		FlyCameraControl();
		virtual ~FlyCameraControl();

		Eigen::Vector3f getPosition();
		Eigen::Vector3f getLookAt();

		void  update(int dX, int dY, int mod);
		void  moveTo(std::shared_ptr<FlyCameraControl> other);
		void  moveTo(std::shared_ptr<CameraControl> other);

	protected:
		void pan(float dX, float dY);
		void zoom(float delta);
		void pitch(float delta);
		void yaw(float delta);
		void determineMatrix();

		// state variables
		Eigen::Vector3f  m_position;
		float  m_pitch;
		float  m_yaw;
		float  m_focalLength;
};


} // View
} // FW

#endif /* FLYCAMERACONTROL_H_ */
