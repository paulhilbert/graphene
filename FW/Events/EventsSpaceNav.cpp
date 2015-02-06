#ifdef USE_SPACENAV

#include "EventsSpaceNav.h"

#include <cmath>
#include <iostream>

namespace FW {
namespace Events {

SpaceNav::SpaceNav() : m_running(false), m_threshold(0.f), m_divisor(400.f), m_exponent(2.f), m_lastPoll(std::chrono::system_clock::now()), m_leftDown(false), m_rightDown(false) {
	if (spnav_open()) {
		std::cout << "No Space Navigator detected." << std::endl;
		return;
	}
	m_running = true;
	std::cout << "Space Navigator started." << std::endl;
}

SpaceNav::~SpaceNav() {
	if (!m_running) return;
	m_running = false;
	spnav_close();
	std::cout << "Space Navigator stopped." << std::endl;
}

float SpaceNav::threshold() {
	return m_threshold;
}

float SpaceNav::divisor() {
	return m_divisor;
}

float SpaceNav::exponent() {
	return m_exponent;
}

void SpaceNav::setThreshold(float threshold) {
	m_threshold = threshold;
}

void SpaceNav::setDivisor(float divisor) {
	m_divisor = divisor;
}

void SpaceNav::setExponent(float exponent) {
	m_exponent = exponent;
}

void SpaceNav::setCallbackPressLeft(ButtonCallback cb) {
    m_callbackPressLeft = std::move(cb);
}

void SpaceNav::setCallbackPressRight(ButtonCallback cb) {
    m_callbackPressRight = std::move(cb);
}

void SpaceNav::setCallbackReleaseLeft(ButtonCallback cb) {
    m_callbackReleaseLeft = std::move(cb);
}

void SpaceNav::setCallbackReleaseRight(ButtonCallback cb) {
    m_callbackReleaseRight = std::move(cb);
}

Eigen::Matrix<float, 6, 1> SpaceNav::motion() {
    if (!m_running) return Eigen::Matrix<float, 6, 1>::Zero();
	auto now = std::chrono::system_clock::now();
	auto timeDelta = std::chrono::duration_cast<std::chrono::microseconds>(now - m_lastPoll).count();
	// Cap time delta to one second to avoid too large jumps.
	if (timeDelta > 1000000) {
		timeDelta = 1000000;
	}
	m_lastPoll = now;
	
	Eigen::Matrix<float, 6, 1> posData = Eigen::Matrix<float, 6, 1>::Zero();
	
	spnav_event ev;
	if (spnav_poll_event(&ev)) {
        if (ev.type == 1) {
            posData << ev.motion.x, ev.motion.z, ev.motion.y, ev.motion.rx, ev.motion.rz, ev.motion.ry;
            for (int i = 0; i < 6; ++i) {
                if (fabs(posData[i]) < m_threshold) {
                    posData[i] = 0.f;
                }
            }
            posData *= (timeDelta / 16000.f) / m_divisor;
            for (int i = 0; i < 6; ++i) {
                float before = posData[i];
                posData[i] = std::copysign(std::pow(posData[i], m_exponent), posData[i]);
            }
            spnav_remove_events(SPNAV_EVENT_MOTION);
        } else {
            bool leftDown = ev.button.bnum == 0 && ev.button.press;
            bool rightDown = ev.button.bnum == 1 && ev.button.press;
            bool leftUp = ev.button.bnum == 0 && !ev.button.press;
            bool rightUp = ev.button.bnum == 1 && !ev.button.press;
            if (leftDown && !m_leftDown) {
                if (m_callbackPressLeft)   m_callbackPressLeft();
                m_leftDown = true;
            }
            if (leftUp   &&  m_leftDown) {
                if (m_callbackReleaseLeft) m_callbackReleaseLeft();
                m_leftDown = false;
            }
            if (rightDown && !m_rightDown) {
                if (m_callbackPressRight)   m_callbackPressRight();
                m_rightDown = true;
            }
            if (rightUp   &&  m_rightDown) {
                if (m_callbackReleaseRight) m_callbackReleaseRight();
                m_rightDown = false;
            }
        }
	}
	return posData;
}

} // namespace Events
} // namespace FW

#endif // USE_SPACENAV
