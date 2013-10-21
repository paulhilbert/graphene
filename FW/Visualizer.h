#ifndef FWVISUALIZER_H_
#define FWVISUALIZER_H_

#include <include/common.h>
#include <include/ogl.h>
#include <future>

#include <FW/FWVisualizerHandle.h>
#include <GUI/GUIVisualizerHandle.h>

#include <IO/AbstractProgressBarPool.h>

namespace FW {

class Graphene;

class Visualizer {
	public:
		typedef std::shared_ptr<Visualizer>   Ptr;
		typedef std::function<void (void)>    Job;
		typedef std::function<void (IO::AbstractProgressBar::Ptr)>        JobWithBar;
//		typedef std::function<void (IO::AbstractProgressBarPool::Ptr)>    JobWithPool;
		typedef std::pair<std::future<void>, Job>                         Task;
		friend class Graphene;

	public:
		Visualizer(std::string id);
		virtual ~Visualizer();

		std::string id() const;

		FW::VisualizerHandle::Ptr  fw();
		GUI::VisualizerHandle::Ptr gui();

		virtual void init() = 0;
		virtual void render() = 0;

		void execute(Job task, Job finally);
		void execute(JobWithBar task, Job finally, std::string taskName, int steps = 1);
		//void execute(JobWithPool task, Job finally);

		static std::vector<std::string> path(std::string&& s0);
		static std::vector<std::string> path(std::string&& s0, std::string&& s1);
		static std::vector<std::string> path(std::string&& s0, std::string&& s1, std::string&& s2);

	protected:
		void setHandles(FW::VisualizerHandle::Ptr fw, GUI::VisualizerHandle::Ptr gui);
		void setProgressBarPool(IO::AbstractProgressBarPool::Ptr m_pool);
		void waitForTasks();

	protected:
		std::string                       m_id;
		FW::VisualizerHandle::Ptr         m_fw;
		GUI::VisualizerHandle::Ptr        m_gui;
		std::vector<Task>                 m_tasks;
		IO::AbstractProgressBarPool::Ptr  m_pool;
};


} // FW

#endif /* FWVISUALIZER_H_ */
