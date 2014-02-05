#ifndef FWTASK_H_
#define FWTASK_H_

#include <memory>
#include <functional>
#include <string>
#include <future>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include <include/config.h>
#ifdef USE_BOOST_SERIALIZATION
#include <Library/IO/serialization_inc.h>
#endif

namespace FW {

class Visualizer;

class Task {
	public:
		friend class Visualizer;

	public:
		typedef std::shared_ptr<Task>                        Ptr;
		typedef std::weak_ptr<Task>                          WPtr;
		typedef std::shared_ptr<const Task>                  ConstPtr;
		typedef std::weak_ptr<const Task>                    ConstWPtr;
		typedef std::function<void (void)>                   Computation;
		typedef std::string                                  Id;
#ifdef USE_BOOST_SERIALIZATION
		typedef boost::archive::polymorphic_binary_iarchive  IArchive;
		typedef boost::archive::polymorphic_binary_oarchive  OArchive;
		typedef std::function<void (IArchive&)>              IFunc;
		typedef std::function<void (OArchive&)>              OFunc;
#endif

	public:
		Task(Id id, Computation computation);
		virtual ~Task();

		void setPre(Computation pre);
		void setPost(Computation post);

		void dependsOn(Task::Ptr dependency);

#ifdef USE_BOOST_SERIALIZATION
		void makePersistent(const fs::path& file, OFunc serialize, IFunc deserialize);
#endif

		void run();
		void runInThread();

		bool computed() const;

		Id id() const;

	protected:
		void poll();
#ifdef USE_BOOST_SERIALIZATION
		bool serialize() const;
		bool deserialize();
#endif

	protected:
		Id           m_id;
		Computation  m_comp;
		Computation  m_pre  = nullptr;
		Computation  m_post = nullptr;
		std::vector<Task::Ptr>        m_dependencies;
		bool                          m_done = false;
		std::future<void>             m_depFuture;
		std::future<void>             m_future;
#ifdef USE_BOOST_SERIALIZATION
		bool      m_persistent = false;
		fs::path  m_file;
		OFunc     m_ofunc;
		IFunc     m_ifunc;
#endif
};


} // FW

#endif /* FWTASK_H_ */
