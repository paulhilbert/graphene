#include "FWTask.h"

#include <GUI/GUIBackend.h>


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

void Task::makePersistent(const fs::path& file, OFunc serialize, IFunc deserialize) {
	m_persistent = true;
	m_file       =  file;
	m_ofunc      = serialize;
	m_ifunc      = deserialize;
}

void Task::run(bool ignorePersist, bool ignorePersistDepends) {
	m_ignorePersist = ignorePersist;
	for (auto d : m_dependencies) {
		if (!d->computed()) d->run(ignorePersistDepends, ignorePersistDepends);
	}

	if (!m_ignorePersist && m_persistent && fs::exists(m_file) && m_ifunc(m_file)) {
		m_done = true;
		return;
	}

	if (m_pre) m_pre();
	m_profiling.profile("execution");
	m_comp();
	m_profiling.profile("execution");
	if (m_post) m_post();
	if (m_persistent) m_ofunc(m_file);
	m_done = true;
}

void Task::runInThread(bool ignorePersist, bool ignorePersistDepends) {
	m_ignorePersist = ignorePersist;
	for (auto d : m_dependencies) {
		if (!d->computed()) d->runInThread(ignorePersistDepends, ignorePersistDepends);
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

template <typename Duration>
unsigned long Task::duration() const {
	return static_cast<unsigned long>(m_profiling.duration<Duration>("execution"));
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
			m_future = std::async(std::launch::async,
				                               [&] () {
				                                  if (!m_ignorePersist && m_persistent && fs::exists(m_file) && m_ifunc(m_file)) return;
															 m_profiling.profile("execution");
															 m_comp();
															 m_profiling.profile("execution");
														 });
		}
	}
	if (m_future.valid()) { // still computing?
		if (m_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
			m_future.get(); // get in order to invalidate the future
			m_done = true;
			if (m_post) m_post();
			if (m_persistent) m_ofunc(m_file);
		}
	}
}


template unsigned long Task::duration<std::chrono::hours>() const;
template unsigned long Task::duration<std::chrono::minutes>() const;
template unsigned long Task::duration<std::chrono::seconds>() const;
template unsigned long Task::duration<std::chrono::milliseconds>() const;
template unsigned long Task::duration<std::chrono::microseconds>() const;
template unsigned long Task::duration<std::chrono::nanoseconds>() const;

} // FW
