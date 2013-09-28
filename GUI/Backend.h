#ifndef BACKEND_H_
#define BACKEND_H_

#include <memory>
#include <functional>
#include <tuple>

#include <FW/Events/EventHandler.h>
#include <FW/Events/Modifier.h>

#include "FactoryHandle.h"
#include <FW/Factory.h>
#include "Log.h"


namespace GUI {

class Backend {
	public:
		typedef std::shared_ptr<Backend> Ptr;

	public:
		Backend();
		virtual ~Backend();

		virtual void init(int argc, char* argv[], FW::Events::EventHandler::Ptr eventHandler, bool verbose) = 0;
		virtual int  run(int fps) = 0;

		virtual FactoryHandle::Ptr addFactory(std::string name) = 0;
		virtual VisualizerHandle::Ptr addVisualizer(std::string name) = 0;

		virtual Log::Ptr getLog() = 0;
		virtual Status::Ptr getStatus() = 0;

		virtual void setWindowTitle(const char* title) = 0;
		virtual void setWindowSize(int width, int height) = 0;
		virtual Eigen::Vector2i getGLSize() = 0;

		virtual Eigen::Vector4f getBackgroundColor() const = 0;
		virtual std::vector<std::string> getActiveVisualizerNames() = 0;

		void setAddVisCallback(const std::function<void (std::string, std::string)>& onAddVis);
		void setDelVisCallback(const std::function<void (std::string)>& onDelVis);
		virtual void setRenderCallback(std::function<void ()> func) = 0;
		virtual void setExitCallback(std::function<void ()> func) = 0;
		void setCamControlCallback(std::function<void (std::string)> onSetCamControl);
		void setOrthoCallback(std::function<void (bool)> onSetOrtho);

#ifdef ENABLE_SCREENCAST
		void setScreencastStartCallback(const std::function<void (fs::path)>& func);
		void setScreencastPauseCallback(const std::function<void (void)>& func);
		void setScreencastResumeCallback(const std::function<void (void)>& func);
		void setScreencastStopCallback(const std::function<void (void)>& func);
#endif // ENABLE_SCREENCAST

	protected:
		std::map<std::string, FW::Factory::WPtr>       m_factories;
		std::function<void (std::string,std::string)>  m_onAddVis;
		std::function<void (std::string)>              m_onDelVis;
		std::function<void (std::string)>              m_onSetCamControl;
		std::function<void (bool)>                     m_onSetOrtho;

#ifdef ENABLE_SCREENCAST
		std::function<void (fs::path)>                 m_onStartScreencast;
		std::function<void (void)>                     m_onPauseScreencast;
		std::function<void (void)>                     m_onResumeScreencast;
		std::function<void (void)>                     m_onStopScreencast;
#endif // ENABLE_SCREENCAST
};

} // GUI

#endif /* BACKEND_H_ */
