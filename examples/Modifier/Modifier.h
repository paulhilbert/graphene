#ifndef MODIFIERVIS_H_
#define MODIFIERVIS_H_

#include <FW/Visualizer.h>
#include <FW/Factory.h>

namespace FW {


class Modifier : public Visualizer {
	public:
		class Factory;

	public:
		Modifier(std::string id);
		virtual ~Modifier();

		void init();
		void render();
		void addProperties();
		void registerEvents();

		void printStatus();
};


class Modifier::Factory : public FW::Factory {
	public:
		Factory();
		virtual ~Factory();

		void init();
		Visualizer::Ptr addVisualizer();
};


} // FW

#endif /* MODIFIERVIS_H_ */
