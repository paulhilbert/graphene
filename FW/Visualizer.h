#ifndef FWVISUALIZER_H_
#define FWVISUALIZER_H_

#include <memory>
#include <future>
#include <vector>

#include <boost/lexical_cast.hpp>
using boost::lexical_cast;

#include <FW/VisualizerHandle.h>
#include <GUI/VisualizerHandle.h>

#include <GUI/Property/Bool.h>
#include <GUI/Property/Button.h>
#include <GUI/Property/Choice.h>
#include <GUI/Property/Color.h>
#include <GUI/Property/File.h>
#include <GUI/Property/Files.h>
#include <GUI/Property/Folder.h>
#include <GUI/Property/Group.h>
#include <GUI/Property/Number.h>
#include <GUI/Property/Range.h>
#include <GUI/Property/Section.h>
#include <GUI/Property/String.h>
#include <GUI/Property/Separator.h>
#include <GUI/Property/ToggleButton.h>
#include <GUI/Property/Tree.h>
using namespace GUI::Property;

namespace FW {

class Graphene;

class Visualizer {
	public:
		typedef std::shared_ptr<Visualizer>   Ptr;
		typedef std::function<void (void)>    Job;
		typedef std::pair<std::future<void>, Job> Task;
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

	protected:
		void setHandles(FW::VisualizerHandle::Ptr fw, GUI::VisualizerHandle::Ptr gui);
		void waitForTasks();

	protected:
		std::string                 m_id;
		FW::VisualizerHandle::Ptr   m_fw;
		GUI::VisualizerHandle::Ptr  m_gui;
		std::vector<Task>           m_tasks;
};


} // FW

#endif /* FWVISUALIZER_H_ */
