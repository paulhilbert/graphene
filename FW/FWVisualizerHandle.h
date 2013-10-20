#ifndef FWVISUALIZERHANDLE_H_
#define FWVISUALIZERHANDLE_H_

#include <include/common.h>

#include <Geometry/Ray.h>
#include <FW/View/Transforms.h>
#include <FW/Events/Handle.h>
#include <FW/Events/Modifier.h>
#include <FW/Events/Keys.h>
using FW::Events::Keys;

namespace FW {

class Graphene;

class VisualizerHandle {
	public:
		typedef std::shared_ptr<VisualizerHandle> Ptr;
		typedef std::weak_ptr<VisualizerHandle>   WPtr;
		friend class Graphene;

	protected:
		VisualizerHandle(std::string id, View::Transforms::WPtr transforms, Events::EventHandler::Ptr eventHandler, Geometry::Ray::Ptr pickRay);

	public:
		virtual ~VisualizerHandle();

		View::Transforms::Ptr  transforms();
		Events::Handle::Ptr    events();
		Events::Modifier::Ptr  modifier();
		Geometry::Ray::Ptr     pickRay();

	protected:
		std::string            m_id;
		View::Transforms::WPtr m_transforms;
		Events::Handle::Ptr    m_events;
		Events::Modifier::Ptr  m_modifier;
		Geometry::Ray::Ptr     m_pickRay;
};

} // FW

#endif /* FWVISUALIZERHANDLE_H_ */
