#ifndef EVENTVIS_H_
#define EVENTVIS_H_

#include <FW/Visualizer.h>
#include <FW/Factory.h>

namespace FW {


class Event : public Visualizer {
	public:
		class Factory;

	public:
		Event(std::string id);
		virtual ~Event();

		void init();
		void render();
		void addProperties();
		void registerEvents();

	protected:
		void move(int dx, int dy, int x, int y);
		void click(int x, int y);
		void resize(int w, int h);
		void scroll(int delta);
		void modifier(Keys::Modifier mod, std::string action);
		void charKey(std::string key, std::string action);
		void specialKey(Keys::Special key, std::string action);
};


class Event::Factory : public FW::Factory {
	public:
		Factory();
		virtual ~Factory();

		void init();
		Visualizer::Ptr addVisualizer();
};


} // FW

#endif /* EVENTVIS_H_ */
