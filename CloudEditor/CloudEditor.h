#ifndef CLOUDEDITORVIS_H_
#define CLOUDEDITORVIS_H_

#include <FW/Visualizer.h>
#include <FW/Factory.h>
#include <PointCloudEditor/PointCloudEditor.h>

namespace FW {


class CloudEditor : public PointCloudEditor {
	public:
		typedef std::shared_ptr<CloudEditor> Ptr;
		typedef std::weak_ptr<CloudEditor>   WPtr;

	public:
		class Factory;

	public:
		CloudEditor(std::string id, const Paths& paths);
		virtual ~CloudEditor();

		void init();
		void render();
		void addProperties();
		void registerEvents();
};


class CloudEditor::Factory : public FW::Factory {
	public:
		Factory();
		virtual ~Factory();

		void init();
		Visualizer::Ptr addVisualizer();
};


} // FW

#endif /* CLOUDEDITORVIS_H_ */
