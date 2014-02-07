#ifndef FWTASK_H_
#define FWTASK_H_

#include <memory>
#include <functional>
#include <string>
#include <future>
#include <boost/variant.hpp>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include <GUI/GUIProgressBarPool.h>

namespace FW {

class Visualizer;

class Task {
	public:
		friend class Visualizer;

	public:
		typedef std::shared_ptr<Task>                                     Ptr;
		typedef std::weak_ptr<Task>                                       WPtr;
		typedef std::shared_ptr<const Task>                               ConstPtr;
		typedef std::weak_ptr<const Task>                                 ConstWPtr;
		typedef std::function<void (int, int)>                            ProgressBar;
		typedef std::function<ProgressBar (std::string name, int steps)>  RequestProgressBar;
		typedef std::function<void (void)>                                Computation;
		typedef std::function<void (::GUI::ProgressBarPool::Ptr)>         IOComputation;
		typedef std::string                                               Id;
		typedef std::function<bool (const fs::path&)>                     IFunc;
		typedef std::function<bool (const fs::path&)>                     OFunc;

	public:
		Task(Id id, Computation computation);
		virtual ~Task();

		void setPre(Computation pre);
		void setPost(Computation post);

		void dependsOn(Task::Ptr dependency);

		void makePersistent(const fs::path& file, OFunc serialize, IFunc deserialize);

		void run();
		void runInThread();

		bool computed() const;

		Id id() const;

	protected:
		void poll();
		bool serialize() const;
		bool deserialize();

	protected:
		Id           m_id;
		Computation  m_comp;

		Computation             m_pre  = nullptr;
		Computation             m_post = nullptr;
		std::vector<Task::Ptr>  m_dependencies;
		bool                    m_done = false;
		std::future<void>       m_depFuture;
		std::future<void>       m_future;
		bool                    m_persistent = false;
		fs::path                m_file;
		OFunc                   m_ofunc;
		IFunc                   m_ifunc;
};


} // FW

#endif /* FWTASK_H_ */
