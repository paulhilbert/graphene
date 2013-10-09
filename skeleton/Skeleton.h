#ifndef SKELETONVIS_H_
#define SKELETONVIS_H_

#include <FW/Visualizer.h>
#include <FW/Factory.h>

namespace FW {


class Skeleton : _INHERIT_ {
	public:
		typedef std::shared_ptr<Skeleton> Ptr;
		typedef std::weak_ptr<Skeleton>   WPtr;

	public:
		class Factory;

	public:
		Skeleton(std::string id);
		virtual ~Skeleton();

		void init();
		void render();
		void addProperties();
		void registerEvents();
};


class Skeleton::Factory : public FW::Factory {
	public:
		Factory();
		virtual ~Factory();

		void init();
		Visualizer::Ptr addVisualizer();
};


} // FW

#endif /* SKELETONVIS_H_ */
