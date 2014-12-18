#ifndef SINGLEMESHVIS_H_
#define SINGLEMESHVIS_H_

#include <FW/FWVisualizer.h>

#include <harmont/openmesh_traits.hpp>

namespace FW {

class SingleMesh : virtual public Visualizer {
	public:
		typedef std::shared_ptr<SingleMesh> Ptr;
		typedef std::weak_ptr<SingleMesh>   WPtr;

        typedef OpenMesh::Vec4f             ColorT;
		typedef harmont::tri_mesh<ColorT>   MeshT;
        typedef harmont::mesh_object<MeshT> RenderedMeshT;

	public:
		SingleMesh(std::string id, fs::path meshFile);
		virtual ~SingleMesh();

		void init();
		void addProperties();
		void addModes();
		void registerEvents();

	protected:
		fs::path              m_meshFile;
        RenderedMeshT::ptr_t  m_mesh;
};

#include "SingleMesh.inl"

} // FW

#endif /* SINGLEMESHVIS_H_ */
