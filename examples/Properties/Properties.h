#ifndef PROPERTIESVIS_H_
#define PROPERTIESVIS_H_

#include <FW/Visualizer.h>
#include <FW/Factory.h>

namespace FW {


class Properties : public Visualizer {
	public:
		class Factory;

	public:
		Properties(std::string id);
		virtual ~Properties();

		void init();
		void render();

	protected:
		void print();
};


class Properties::Factory : public FW::Factory {
	public:
		Factory();
		virtual ~Factory();

		void init();
		Visualizer::Ptr addVisualizer();
};


} // FW

#endif /* PROPERTIESVIS_H_ */
