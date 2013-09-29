#ifndef PROFILING_H
#define PROFILING_H

#include <iostream> // for graphics
#include <map>

#include <IO/Log.h>
using namespace IO;


#if defined(PROFILING) && defined(DISABLE_BOOST_TIMING) && defined(_WIN32)
#undef PROFILING
#endif

namespace Testing {
enum ProfilingUnit { PROFILE_H, PROFILE_M, PROFILE_S, PROFILE_MS, PROFILE_US, PROFILE_NS };
} // Testing


#ifdef PROFILING

#ifdef DISABLE_BOOST_TIMING
// use native posix timing
#include <sys/time.h>

namespace Testing {
	struct TimeInfo {
		unsigned long sec_start;
		unsigned long sec_end;
		unsigned long usec_start;
		unsigned long usec_end;
		unsigned long usec_diff;
		bool finished;
	};
} // Testing
#else // !DISABLE_BOOST_TIMING
// use boost timing
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/optional.hpp>
using boost::posix_time::ptime;
using boost::posix_time::time_period;
using boost::posix_time::time_duration;
using boost::posix_time::microsec_clock;
using boost::optional;

namespace Testing {
	struct TimeInfo {
		ptime                  start;
		optional<ptime>        end;
		optional<time_period>  period;
	};
} // Testing

#endif // DISABLE_BOOST_TIMING

#else // !PROFILING
namespace Testing { struct TimeInfo {}; }
#endif // PROFILING

namespace Testing {

class Profiling {
	public:
		Profiling(ProfilingUnit profilingUnit = PROFILE_MS);

		ProfilingUnit getProfilingUnit() const;
		void setProfilingUnit(ProfilingUnit unit);

		void startProfile(std::string name);
		void endProfile(std::string name);
		void profile(std::string name);

		void summarizeProfiles();

	protected:
		std::map< std::string, TimeInfo >  m_profiles;
		ProfilingUnit                      m_profilingUnit;
};


inline Profiling::Profiling(ProfilingUnit profilingUnit) : m_profilingUnit(profilingUnit) {
}

inline ProfilingUnit Profiling::getProfilingUnit() const {
	return m_profilingUnit;
}

inline void Profiling::setProfilingUnit(ProfilingUnit unit) {
	m_profilingUnit = unit;
}

inline void Profiling::startProfile(std::string name) {
#ifdef PROFILING
#ifdef DISABLE_BOOST_TIMING
	struct timeval start;
	gettimeofday(&start, NULL);
	TimeInfo t;
	t.sec_start = start.tv_sec;
	t.usec_start = start.tv_usec;
	t.finished = false;

	m_profiles[name] = t;
#else // use boost
	TimeInfo t;
	t.start = ptime(microsec_clock::local_time());

	m_profiles[name] = t;
#endif // DISABLE_BOOST_TIMING
#endif // PROFILING
}

inline void Profiling::endProfile(std::string name) {
#ifdef PROFILING
	if (!m_profiles.count(name)) return;

#ifdef DISABLE_BOOST_TIMING
	struct timeval end;
	gettimeofday(&end, NULL);
	m_profiles[name].sec_end  = end.tv_sec;
	m_profiles[name].usec_end = end.tv_usec;

	m_profiles[name].usec_diff = ((m_profiles[name].sec_end - m_profiles[name].sec_start) * 1000000 +
		                           (m_profiles[name].usec_end - m_profiles[name].usec_start) ) + 0.5;

	m_profiles[name].finished = true;

	//std::stringstream str; str << "Measured time for " << name << ": " << m_profiles[name].usec_diff << "us";
	//info(str.str());
#else // use boost
	m_profiles[name].end = ptime(microsec_clock::local_time());
	m_profiles[name].period = time_period( m_profiles[name].start, m_profiles[name].end.get() );

	//time_duration diff = m_profiles[name].period.get().length();
	//std::stringstream str; str << "Measured time for " << name << ": ";
	
	/*
	switch (m_profilingUnit) {
		case PROFILE_H:   str << diff.hours() << "h"; break;
		case PROFILE_M:   str << diff.minutes() << "h"; break;
		case PROFILE_S:   str << diff.total_seconds() << "h"; break;
		case PROFILE_MS:  str << diff.total_milliseconds() << "ms"; break;
		case PROFILE_US:  str << diff.total_microseconds() << "us"; break;
		case PROFILE_NS:  str << diff.total_nanoseconds() << "ns"; break;
	}
	*/
	
	//info(str.str());
#endif // DISABLE_BOOST_TIMING
#endif // PROFILING
}

inline void Profiling::profile(std::string name) {
	if (m_profiles.count(name))
		endProfile(name);
	else
		startProfile(name);
}

inline void Profiling::summarizeProfiles() {
	if (!m_profiles.size()) return;

#ifdef PROFILING
#ifdef DISABLE_BOOST_TIMING
	struct timeval now;
	gettimeofday(&now, NULL);

	unsigned long max_sec = 0;
	unsigned long max_usec = 0;
	unsigned long min_sec = now.tv_sec;
	unsigned long min_usec = now.tv_usec;
	for (std::map< std::string, TimeInfo >::iterator it = m_profiles.begin(); it != m_profiles.end(); ++it) {
		if (!(*it).second.finished) {
			Log::warn("Unfinished profile block: " + (*it).first);
			endProfile((*it).first);
		}


		if ((*it).second.sec_start <= min_sec) {
			min_sec = (*it).second.sec_start;
			// set min_usec only if current time is earlier than saved time
			min_usec = ((*it).second.sec_start < min_sec || (*it).second.usec_start < min_usec) ? (*it).second.usec_start : min_usec;
		}
		if ((*it).second.sec_end >= max_sec) {
			max_sec = (*it).second.sec_end;
			// set max_usec only if current time is later than saved time
			max_usec = ((*it).second.sec_end > max_sec || (*it).second.usec_end > max_usec) ? (*it).second.usec_end : max_usec;
		}
//		long start = (*it).second.sec_start * 1000000 + (*it).second.usec_start + 0.5;
//		long end = (*it).second.sec_end * 1000000 + (*it).second.usec_end + 0.5;
//		min = (min < start) ? min : start;
//		max = (max > end) ? max : end;
	}
	long time_range = (max_sec - min_sec) * 1000000 + (max_usec - min_usec) + 0.5;

	// print empty lines before summary
	Log::info("");
	Log::info("Profiling summary");


	for (std::map< std::string, TimeInfo >::iterator it = m_profiles.begin(); it != m_profiles.end(); ++it) {
		double fraction = (double)((*it).second.usec_diff) / time_range;

		char output[255];
		sprintf(output, "  %-25s%11dus   %13.3f%%", (*it).first.c_str(), (int)(*it).second.usec_diff, fraction*100.0);
		Log::info(std::string(output));
	}
	//Log::info("-----------------------------------------------------------", false);
	char output[255];
	sprintf(output, "  Total measured time:     %11dus   %13.3f%%", (int)time_range, 100.0);
	Log::info(std::string(output));
//	Log::info("-----------------------------------------------------------", false);
#else // use boost
	optional<time_period> profileSpan;

	for (std::map< std::string, TimeInfo >::iterator it = m_profiles.begin(); it != m_profiles.end(); ++it) {
		if (!(*it).second.end) {
			Log::warn("Unfinished profile block: " + (*it).first);
			endProfile((*it).first);
		}

		if (!profileSpan)
			profileSpan = (*it).second.period.get();
		else
			profileSpan.get() = profileSpan.get().span((*it).second.period.get());
	}

	// print empty lines before summary
	Log::info("");
	Log::info("Profiling summary");

	for (std::map< std::string, TimeInfo >::iterator it = m_profiles.begin(); it != m_profiles.end(); ++it) {
		time_duration duration = (*it).second.period.get().length();
		double fraction = (double)(duration.total_microseconds()) / profileSpan.get().length().total_microseconds();

		char output[255];
		int iDur = 0;
		std::string format;
		switch (m_profilingUnit) {
			case PROFILE_H:   iDur = duration.hours(); format = "  %-25s%11dh   %13.3f%%"; break;
			case PROFILE_M:   iDur = duration.minutes(); format = "  %-25s%11dm   %13.3f%%"; break;
			case PROFILE_S:   iDur = duration.total_seconds(); format = "  %-25s%11ds   %13.3f%%"; break;
			case PROFILE_MS:  iDur = static_cast<int>(duration.total_milliseconds()); format = "  %-25s%11dms   %13.3f%%";  break;
			case PROFILE_US:  iDur = static_cast<int>(duration.total_microseconds()); format = "  %-25s%11dus   %13.3f%%";  break;
			case PROFILE_NS:  iDur = static_cast<int>(duration.total_nanoseconds()); format = "  %-25s%11dns   %13.3f%%"; break;
		}
		sprintf(output, format.c_str(), (*it).first.c_str(), iDur, fraction*100.0);
		Log::info(std::string(output));
	}
//	Log::info("-----------------------------------------------------------", false);
	char output[255];
	int iDur = 0;
	std::string format;
	time_duration tDur = profileSpan.get().length();
	switch (m_profilingUnit) {
		case PROFILE_H:   iDur = tDur.hours(); format = "  Total measured time:     %11dh   %13.3f%%"; break;
		case PROFILE_M:   iDur = tDur.minutes(); format = "  Total measured time:     %11dm   %13.3f%%"; break;
		case PROFILE_S:   iDur = tDur.total_seconds(); format = "  Total measured time:     %11ds   %13.3f%%"; break;
		case PROFILE_MS:  iDur = static_cast<int>(tDur.total_milliseconds()); format = "  Total measured time:     %11dms   %13.3f%%";  break;
		case PROFILE_US:  iDur = static_cast<int>(tDur.total_microseconds()); format = "  Total measured time:     %11dus   %13.3f%%";  break;
		case PROFILE_NS:  iDur = static_cast<int>(tDur.total_nanoseconds()); format = "  Total measured time:     %11dns   %13.3f%%"; break;
	}
	sprintf(output, format.c_str(), iDur, 100.0);
	Log::info(std::string(output));
//	Log::info("-----------------------------------------------------------", false);
#endif // DISABLE_BOOST_TIMING
#endif // PROFILING
}

} // Testing

#endif // PROFILING_H
