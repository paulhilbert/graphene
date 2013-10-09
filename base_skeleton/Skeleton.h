#ifndef SKELETONVIS_H_
#define SKELETONVIS_H_

#include <FW/Visualizer.h>

namespace FW {


class Skeleton : _INHERIT_ {
	public:
		std::shared_ptr<Skeleton> Ptr;
		std::weak_ptr<Skeleton>   WPtr;

	public:
		Skeleton(std::string id);
		virtual ~Skeleton();

		void init();
		void render();
		void addProperties();
		void registerEvents();
};

#include "Skeleton.inl"

} // FW

#endif /* SKELETONVIS_H_ */
