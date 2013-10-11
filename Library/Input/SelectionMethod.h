#ifndef SELECTIONMETHOD_H_
#define SELECTIONMETHOD_H_

#include <memory>
#include <functional>

#include <Eigen/Dense>

#include <FW/FWVisualizerHandle.h>

namespace Input {

class SelectionMethod {
	public:
		typedef std::shared_ptr<SelectionMethod> Ptr;
		typedef std::weak_ptr<SelectionMethod> WPtr;

	public:
		SelectionMethod(FW::VisualizerHandle::Ptr handle);
		virtual ~SelectionMethod();

		virtual void init() = 0;
		virtual void render() = 0;

		void enable();
		void disable();

		void setRadius(unsigned int radius);

		void setStartCallback(std::function<void ()> func);
		void setDragCallback(std::function<void ()> func);
		void setStopCallback(std::function<void ()> func);
		void setUnselectCallback(std::function<void ()> func);

		bool pointInSelection(Eigen::Vector3f point);

	protected:
		FW::VisualizerHandle::Ptr m_handle;
		bool m_enabled;
		bool m_dragging;

		std::function<void ()> m_start;
		std::function<void ()> m_drag;
		std::function<void ()> m_stop;
		std::function<void ()> m_unselect;
};

#include "SelectionMethod.inl"

} // Input

#endif /* SELECTIONMETHOD_H_ */
