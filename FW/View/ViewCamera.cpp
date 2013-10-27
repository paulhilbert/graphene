/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "ViewCamera.h"

#include <include/common.h>
#include <include/ogl.h>

#include <FW/Events/EventsEventHandler.h>
#include <FW/View/ViewCameraControl.h>

namespace FW {
namespace View {

Camera::Camera(CameraControl::Ptr control, Events::EventHandler::Ptr eventHandler, Transforms::WPtr transforms) : m_control(control), m_eventHandler(eventHandler), m_transforms(transforms), m_pickRay(new Geometry::Ray()), m_ortho(false) {
	registerEvents();
	updateTransforms();
	updatePickRay(0, 0);
}

Camera::~Camera() {
}

Eigen::Vector3f Camera::getPosition() const {
	return m_control->getPosition();
}

Eigen::Vector3f Camera::getLookAt() const {
	return m_control->getLookAt();
}

void Camera::toggleOrtho() {
	setOrtho(!m_ortho);
}

void Camera::setOrtho(bool active) {
	//if (active && !m_ortho) {
	//	auto trans = m_transforms.lock();
	//	Eigen::Vector4f lookAt;
	//	lookAt << m_control->getLookAt(), 1.f;
	//	lookAt = trans->modelview()*lookAt;
	//	Eigen::Vector4f right = lookAt;
	//	lookAt[0] += 1.f;

	//	auto persp = trans->projection();
	//	int w = trans->viewport()[2], h = trans->viewport()[3];
	//	float aspect = (float)std::max(w, h) / std::min(w, h);
	//	float factor = tan(40*M_PI/180.f) * 20.f;
	//	auto ortho = Eigen::ortho(-aspect*factor, aspect*factor, -factor, factor, 0.1f, 500.0f);
	//	Eigen::Vector4f p0 = persp*lookAt, p1 = persp*right, o0 = ortho*lookAt, o1 = ortho*right;
	//	p0.head(3) /= p0[3]; p1.head(3) /= p1[3];
	//	m_orthoScale = (p1.head(3)-p0.head(3)).norm() / (o1-o0).norm();
	//}
	m_ortho = active;
	updateTransforms();
}

bool Camera::getOrtho() const {
	return m_ortho;
}

void Camera::setControl(CameraControl::Ptr control) {
	// move new camera control to current position
	control->moveTo(m_control);
	m_control = control;
}

CameraControl::Ptr Camera::getControl() {
	return m_control;
}

Geometry::Ray::Ptr Camera::getPickRay() {
	return m_pickRay;
}

void Camera::updatePickRay(int x, int y) {
	auto trans = m_transforms.lock();
	Eigen::Vector3f farPoint  = Eigen::unProject( Eigen::Vector3f(static_cast<float>(x), static_cast<float>(y), 1.f), trans->modelview(), trans->projection(), trans->viewport() );
	Eigen::Vector3f nearPoint = Eigen::unProject( Eigen::Vector3f(static_cast<float>(x), static_cast<float>(y), 0.f), trans->modelview(), trans->projection(), trans->viewport() );
	m_pickRay->setPoints(nearPoint, farPoint);
}

void Camera::updateTransforms() {
	float near = 0.1f;
	float far = 400.f;
	auto trans = m_transforms.lock();
	trans->modelview() = m_control->getViewTransformation();
	trans->projection() = getProjectionMatrix(trans->viewport()[2], trans->viewport()[3], near, far);
	trans->normal() = trans->modelview().block<3,3>(0,0).inverse().transpose();
	trans->near() = near;
	trans->far() = far;
}

Eigen::Matrix4f Camera::getProjectionMatrix(int w, int h, float near, float far) {
	float aspect = (float)(w > h ? w : h) / (w > h ? h : w);
	Eigen::Matrix4f pr = Eigen::perspective(40.f, aspect, near, far);
	if (m_ortho) {
		auto trans = m_transforms.lock();
		Eigen::Vector4f lookAt;
		lookAt << m_control->getLookAt(), 1.f;
		lookAt = trans->modelview()*lookAt;
		Eigen::Vector4f right = lookAt;
		right[0] += 1.f;

		float factor = static_cast<float>(tan(40*M_PI/180.f) * 20.0);
		Eigen::Matrix4f ortho = Eigen::ortho(-aspect*factor, aspect*factor, -factor, factor, 0.1f, 500.0f);
		Eigen::Vector4f p0 = pr*lookAt, p1 = pr*right, o0 = ortho*lookAt, o1 = ortho*right;
		p0.head(3) /= p0[3]; p1.head(3) /= p1[3];
		float scale = (p1.head(3)-p0.head(3)).norm() / (o1.head(3)-o0.head(3)).norm();
		Eigen::Matrix4f scaleMat = Eigen::Matrix4f::Identity();
		scaleMat.block<3,3>(0,0) *= scale;
		pr = ortho * scaleMat;
		//pr = Eigen::ortho(-aspect, aspect, -1, 1, 0.1f, 500.0f);
	}
	return pr;
}

void Camera::registerEvents() {
	m_eventHandler->registerReceiver(
		"RIGHT_DRAG",
		"Camera",
		std::function<void (int,int,int,int)>(
			[&](int dx, int dy, int x, int y) {
				int m = 0;
				if (m_eventHandler->modifier()->ctrl() && m_eventHandler->modifier()->shift()) return;
				if (m_eventHandler->modifier()->ctrl() )  { m |= MODIF_CTRL; dx *= 3; dy *= 3; }
				if (m_eventHandler->modifier()->alt()  )  { m |= MODIF_ALT; }
				if (m_eventHandler->modifier()->shift())  { m |= MODIF_SHIFT;	}
				m_control->update(dx, dy, m);
				updateTransforms();
			}
		)
	);
	m_eventHandler->registerReceiver(
		"SCROLL",
		"Camera",
		std::function<void (int)>(
			[&](int d) {
				m_control->update(0, -50*d, MODIF_SHIFT);
				updateTransforms();
			}
		)
	);
	std::function<void (int,int,int,int)> rayUpdate = [&] (int dx, int dy, int x, int y) { updatePickRay(x,y); };
	m_eventHandler->registerReceiver("MOVE", "Camera", rayUpdate);
	m_eventHandler->registerReceiver("LEFT_DRAG", "Camera", rayUpdate);
	m_eventHandler->registerReceiver("MIDDLE_DRAG", "Camera", rayUpdate);
	m_eventHandler->registerReceiver("RIGHT_DRAG", "Camera", rayUpdate);
}

} // View
} // FW
