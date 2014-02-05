#include "FWTask.h"

#include <fstream>

namespace FW {


Task::Task(Id id, Computation computation) : m_id(id), m_comp(computation) {
}

Task::~Task() {
}

void Task::setPre(Computation pre) {
	m_pre = pre;
}

void Task::setPost(Computation post) {
	m_post = post;
}

void Task::dependsOn(Task::Ptr dependency) {
	m_dependencies.push_back(dependency);
}

#ifdef USE_BOOST_SERIALIZATION
void Task::makePersistent(const fs::path& file, OFunc serialize, IFunc deserialize) {
	m_persistent = true;
	m_file       =  file;
	m_ofunc      = serialize;
	m_ifunc      = deserialize;
}

#endif

void Task::run() {
#ifdef USE_BOOST_SERIALIZATION
	if (m_persistent && fs::exists(m_file)) {
		if (deserialize()) {
			m_done = true;
			return;
		}
	}
#endif
	for (auto d : m_dependencies) {
		if (!d->computed()) d->run();
	}
	if (m_pre) m_pre();
	m_comp();
	if (m_post) m_post();
#ifdef USE_BOOST_SERIALIZATION
	if (m_persistent) serialize();
#endif
	m_done = true;
}

void Task::runInThread() {
#ifdef USE_BOOST_SERIALIZATION
	if (m_persistent && fs::exists(m_file)) {
		if (deserialize()) {
			m_done = true;
			return;
		}
	}
#endif
	for (auto d : m_dependencies) {
		if (!d->computed()) d->runInThread();
	}
	m_depFuture = std::async(std::launch::async,
	                      [&] () {
	                         // wait for dependencies to finish
	                         bool  allDone = false;
	                         while (!allDone) {
	                            allDone = true;
	                            for (auto d : m_dependencies) {
	                               if (!d->m_done) {
	                                  allDone = false;
	                                  break;
											 }
										 }
									 }
								 });
}

bool Task::computed() const {
	return m_done;
}

Task::Id Task::id() const {
	return m_id;
}

void Task::poll() {
	if (m_depFuture.valid()) { // still waiting for dependencies?
		// if we are still waiting return
		if (m_depFuture.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) return;
		else {
			m_depFuture.get(); // get() in order to invalidate the future
			// after waiting for dependencies we call pre() in this thread
			if (m_pre) m_pre();
			if (m_comp) m_future = std::async(std::launch::async, m_comp);
		}
	}
	if (m_future.valid()) { // still computing?
		if (m_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
			m_future.get(); // get in order to invalidate the future
			m_done    = true;
			if (m_post) m_post();
#ifdef USE_BOOST_SERIALIZATION
			if (m_persistent) serialize();
#endif
		}
	}
}

#ifdef USE_BOOST_SERIALIZATION
bool Task::serialize() const {
	std::ofstream  out(m_file.string().c_str());
	if (!out.good()) {
		throw std::runtime_error("Could not open output data stream.");
	}
	OArchive  ar(out);
	m_ofunc(ar);
	out.close();
	return true;
}

bool Task::deserialize() {
	std::ifstream  in(m_file.string().c_str());
	if (!in.good()) {
		throw std::runtime_error("Could not open output data stream.");
	}
	IArchive  ar(in);
	m_ifunc(ar);
	in.close();
	return true;
}

#endif

} // FW
