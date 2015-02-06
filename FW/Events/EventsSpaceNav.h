#ifndef USE_SPACENAV
#error "You cannot use EventsSpaceNav.h without defining USE_SPACENAV."
#endif

#ifndef EVENTSSPACENAV_H
#define EVENTSSPACENAV_H

#include <chrono>
#include <memory>

#include <Eigen/Dense>

#include <spnav.h>

namespace FW {
namespace Events {

class SpaceNav {
	public:
		typedef std::shared_ptr<SpaceNav> Ptr;
        typedef std::function<void ()>    ButtonCallback;

	public:
		SpaceNav();
		~SpaceNav();

		float threshold();
		float divisor();
		float exponent();
		void setThreshold(float threshold);
		void setDivisor(float divisor);
		void setExponent(float exponent);

        void setCallbackPressLeft(ButtonCallback cb);
        void setCallbackPressRight(ButtonCallback cb);
        void setCallbackReleaseLeft(ButtonCallback cb);
        void setCallbackReleaseRight(ButtonCallback cb);

		Eigen::Matrix<float, 6, 1> motion();

	protected:
		bool  m_running;
		float m_threshold;
		float m_divisor;
		float m_exponent;
        bool  m_leftDown;
        bool  m_rightDown;
        ButtonCallback m_callbackPressLeft;
        ButtonCallback m_callbackPressRight;
        ButtonCallback m_callbackReleaseLeft;
        ButtonCallback m_callbackReleaseRight;

		std::chrono::system_clock::time_point m_lastPoll;
};

} // namespace Events
} // namespace FW

#endif // EVENTSSPACENAV_H
