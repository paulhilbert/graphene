/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef FLYCAMERACONTROL_H_
#define FLYCAMERACONTROL_H_

/**
 *  @internal @file FlyCameraControl.h
 *
 *  @brief Implementation of fly-style camera control mode.
 *
 */

#include <include/common.h>

#include "CameraControl.h"

namespace FW {
namespace View {

/**
 *  @internal FlyCameraControl
 *
 *  @brief Camera fly control class.
 *
 *  Classes derived from CameraControl in order to implement fly camera control mode.
 */
class FlyCameraControl : public CameraControl {
	public:
		typedef std::shared_ptr<FlyCameraControl>  Ptr;
		typedef std::weak_ptr<FlyCameraControl>    WPtr;

	public:
		/**
		 *  Constructor.
		 */
		FlyCameraControl();

		/**
		 *  Destructor.
		 */
		virtual ~FlyCameraControl();

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
		 *  Align this control with another fly control.
		 *  @param other Pointer to FlyCameraControl.
		 */
		void  moveTo(std::shared_ptr<FlyCameraControl> other);

		/**
		 *  Align this control with another camera control.
		 *  @param other Pointer to CameraControl.
		 */
		void  moveTo(std::shared_ptr<CameraControl> other);

	protected:
		void pan(float dX, float dY);
		void zoom(float delta);
		void pitch(float delta);
		void yaw(float delta);
		void determineMatrix();

		// state variables
		Vector3f  m_position;
		float     m_pitch;
		float     m_yaw;
		float     m_focalLength;
};


} // View
} // FW

#endif /* FLYCAMERACONTROL_H_ */
