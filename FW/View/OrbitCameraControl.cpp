#include "OrbitCameraControl.h"

#include <iostream>

namespace FW {
namespace View {


OrbitCameraControl::OrbitCameraControl() : CameraControl(),	m_theta(0.f), m_phi(0.f),	m_radius(20.f), m_center(0.f, 0.f, 0.f) {
	m_viewTransform = Eigen::Matrix4f::Identity();
	determineMatrix();
}

OrbitCameraControl::~OrbitCameraControl() {
}

Eigen::Vector3f OrbitCameraControl::getPosition() {
	return m_coordsOGLToMath.block<3,3>(0,0)
		  * ((Eigen::AngleAxisf(m_phi  , Eigen::Vector3f::UnitY())
	     *   Eigen::AngleAxisf(m_theta, Eigen::Vector3f::UnitX()))
	     *   Eigen::Vector3f(0.f, 0.f, m_radius)) + m_center;
	//Eigen::Vector3f position( m_sinPhi * m_sinTheta, -m_cosPhi * m_sinTheta, m_cosTheta );
	//return (m_radius * position) + m_center;
}

Eigen::Vector3f OrbitCameraControl::getLookAt() {
	return m_center;
}

void OrbitCameraControl::update(int dX, int dY, int mod) {
	switch (mod) {
		case MODIF_SHIFT:  zoom(0.005f * dY);
		                 determineMatrix();
		                 break;
		case MODIF_CTRL:   pan(-0.005f * dX, 0.005f * dY);
		                 determineMatrix();
		                 break;
		case (MODIF_CTRL | MODIF_SHIFT):
		                 pan(-0.005f * dX, 0.005f * dY, true);
		                 determineMatrix();
		                 break;
		default:         rotP(-0.005f * dX);
		                 rotT(-0.005f * dY);
		                 determineMatrix();
	}
}

void OrbitCameraControl::moveTo(std::shared_ptr<OrbitCameraControl> other) {
	m_center = other->m_center;
	m_radius = other->m_radius;
	m_phi    = other->m_phi;
	m_theta  = other->m_theta;

	determineMatrix();
}

void OrbitCameraControl::moveTo(std::shared_ptr<CameraControl> other) {
	m_center = other->getLookAt();
	Eigen::Vector3f dir = other->getPosition() - m_center;
	m_radius = dir.norm();

	if (m_radius < 0.0001f) {
		m_phi = 0.f;
		m_theta = 0.f;
		return;
	}
	m_phi = atan2(dir[0], -dir[1]);
	if (m_phi < 0.f) m_phi += static_cast<float>(2.f*M_PI);
	m_theta = static_cast<float>(acos(dir[2] / m_radius) - 0.5f*M_PI);

	determineMatrix();
}

void OrbitCameraControl::pan(float dX, float dY, bool stayInPlane) {
	Eigen::Vector3f dirX = getRightDirection();
	Eigen::Vector3f dirY = getUpDirection();

	if (stayInPlane) {
		// we want to stay in the xy-plane - drop z-coord
		dirX[2] = 0.f;
		dirY[2] = 0.f;
		// dropping z generally changes the length of dirY
		dirX.normalize();
		dirY.normalize();
	}
	m_center += dX * dirX + dY * dirY;
}


void OrbitCameraControl::zoom(float delta) {
	m_radius += delta * m_radius;
	if (m_radius < 0.1f) m_radius = 0.1f;
}

void OrbitCameraControl::rotP(float delta) {
	m_phi += delta;
	// constrain phi to [0,2*pi)
	// by looping if interval bound is reached
	float pi2 = static_cast<float>(2.f * M_PI);
	while (m_phi < 0) m_phi += pi2;
	while (m_phi >= pi2) m_phi -= pi2;
}

void OrbitCameraControl::rotT(float delta) {
	m_theta += delta;
	// constrain theta to [-pi/2,pi/2]
	// by clamping
	float pdiv2 = static_cast<float>(0.5f * M_PI);
	if (m_theta < -pdiv2) m_theta = -pdiv2;
	if (m_theta >  pdiv2) m_theta =  pdiv2;
}

void OrbitCameraControl::determineMatrix() {
	Eigen::Affine3f t;
	t = Eigen::Translation<float,3>(-m_radius*Eigen::Vector3f::UnitZ())
	  * Eigen::AngleAxisf(-m_theta, Eigen::Vector3f::UnitX())
	  * Eigen::AngleAxisf(-m_phi  , Eigen::Vector3f::UnitY())
	  * Eigen::Translation<float,3>(m_coordsMathToOGL.block<3,3>(0,0) * (-m_center));
	m_viewTransform = t.matrix();
}


} // View
} // FW
