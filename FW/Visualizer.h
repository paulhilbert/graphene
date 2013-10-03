#ifndef FWVISUALIZER_H_
#define FWVISUALIZER_H_

#include <memory>
#include <future>
#include <vector>

#include <boost/lexical_cast.hpp>
using boost::lexical_cast;

#include <FW/FWVisualizerHandle.h>
#include <GUI/GUIVisualizerHandle.h>

#include <GUI/Property/Bool.h>
#include <GUI/Property/Button.h>
#include <GUI/Property/Choice.h>
#include <GUI/Property/Color.h>
#include <GUI/Property/File.h>
#include <GUI/Property/Files.h>
#include <GUI/Property/Folder.h>
#include <GUI/Property/PropGroup.h>
#include <GUI/Property/Number.h>
#include <GUI/Property/Range.h>
#include <GUI/Property/Section.h>
#include <GUI/Property/String.h>
#include <GUI/Property/Separator.h>
#include <GUI/Property/ToggleButton.h>
#include <GUI/Property/Tree.h>
using namespace GUI::Property;

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

		std::vector<std::string> path(std::string&& s0);
		std::vector<std::string> path(std::string&& s0, std::string&& s1);
		std::vector<std::string> path(std::string&& s0, std::string&& s1, std::string&& s2);

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
