#include "FlyCameraControl.h"

namespace FW {
namespace View {


FlyCameraControl::FlyCameraControl() : CameraControl(),	m_position(0.f, -20.f, 0.f), m_pitch(0.f), m_yaw(0.f), m_focalLength(20.f) {
	determineMatrix();
}

FlyCameraControl::~FlyCameraControl() {
}

Eigen::Vector3f FlyCameraControl::getPosition() {
	return m_position;
}

Eigen::Vector3f FlyCameraControl::getLookAt() {
	return m_position + (getViewDirection() * m_focalLength);
}

void FlyCameraControl::update(int dX, int dY, int mod) {
	switch (mod) {
		case MODIF_SHIFT:  zoom(-0.05f * dY);
		                 determineMatrix();
		                 break;
		case MODIF_CTRL:   pan(0.005f * dX, -0.005f * dY);
		                 determineMatrix();
		                 break;
		default:         yaw(-0.01f * dX);
		                 pitch(-0.01f * dY);
		                 determineMatrix();
	}
}

void FlyCameraControl::moveTo(std::shared_ptr<FlyCameraControl> other) {
	m_position    = other->m_position;
	m_focalLength = other->m_focalLength;
	m_yaw         = other->m_yaw;
	m_pitch       = other->m_pitch;

	determineMatrix();
}

void FlyCameraControl::moveTo(std::shared_ptr<CameraControl> other) {
	m_position = other->getPosition();
	Eigen::Vector3f dir = other->getLookAt()-m_position;
	m_focalLength = dir.norm();

	if (m_focalLength < 0.0001f) {
		m_yaw = 0.f;
		m_pitch = 0.f;
		return;
	}
	m_yaw = atan2(-dir[0], dir[1]);
	if (m_yaw < 0.f) m_yaw += static_cast<float>(2.f*M_PI);
	m_pitch = static_cast<float>(acos(-dir[2] / m_focalLength) - 0.5f*M_PI);

	determineMatrix();
}

void FlyCameraControl::pan(float dX, float dY) {
	m_position += Eigen::Vector3f(dX*getRightDirection() + dY*getUpDirection());
}


void FlyCameraControl::zoom(float delta) {
	m_position += delta * getViewDirection();
}

void FlyCameraControl::pitch(float delta) {
	m_pitch += delta;
	// constrain pitch to [-pi/2,pi/2]
	// by clamping
	float pidiv2 = static_cast<float>(0.5f * M_PI);
	if (m_pitch < -pidiv2) m_pitch = -pidiv2;
	if (m_pitch >  pidiv2) m_pitch = pidiv2;
}

void FlyCameraControl::yaw(float delta) {
	m_yaw += delta;
	// constrain yaw to [0,2*pi)
	// by looping if interval bound is reached
	float pi2 = static_cast<float>(2.f * M_PI);
	while (m_yaw < 0) m_yaw += pi2;
	while (m_yaw >= pi2) m_yaw -= pi2;
}

void FlyCameraControl::determineMatrix() {
	Eigen::Affine3f t; // (T(pos) * R_y(yaw) * R_x(pitch))^(-1) = R_x(-pitch) * R_y(-yaw) * T(-pos)
	t = Eigen::AngleAxisf(-m_pitch, Eigen::Vector3f::UnitX())
	  * Eigen::AngleAxisf(-m_yaw,   Eigen::Vector3f::UnitY())
	  * Eigen::Translation<float,3>(m_coordsMathToOGL.block<3,3>(0,0)*(-m_position));
	m_viewTransform = t.matrix();
}

} // View
} // FW
