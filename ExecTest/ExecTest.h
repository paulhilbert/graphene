#ifndef EXECTESTVIS_H_
#define EXECTESTVIS_H_

#include <FW/Visualizer.h>
#include <FW/Factory.h>

namespace FW {


class ExecTest : public Visualizer {
	public:
		class Factory;

	public:
		ExecTest(std::string id);
		virtual ~ExecTest();

		void init();
		void render();
		void addProperties();
		void registerEvents();
};


class ExecTest::Factory : public FW::Factory {
	public:
		Factory();
		virtual ~Factory();

		void init();
		Visualizer::Ptr addVisualizer();
};


} // FW

#endif /* EXECTESTVIS_H_ */
