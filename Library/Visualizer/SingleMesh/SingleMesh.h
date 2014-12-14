#ifndef SINGLEMESHVIS_H_
#define SINGLEMESHVIS_H_

#include <FW/FWVisualizer.h>

#include <Library/Rendered/Mesh.h>

namespace FW {

class SingleMesh : virtual public Visualizer {
	public:
		typedef std::shared_ptr<SingleMesh> Ptr;
		typedef std::weak_ptr<SingleMesh>   WPtr;

        typedef OpenMesh::Vec4f           ColorT;
		typedef harmont::tri_mesh<ColorT> MeshT;
        typedef Rendered::Mesh<ColorT>    RenderedMeshT;

	public:
		SingleMesh(std::string id, fs::path meshFile);
		virtual ~SingleMesh();

		void init();
		void initGeometry(harmont::shader_program::ptr program, harmont::pass_type_t type);
		void render(harmont::shader_program::ptr program, harmont::pass_type_t type);
		void addProperties();
		void addModes();
		void registerEvents();

		virtual BoundingBox boundingBox() const;

	protected:
		fs::path                  m_meshFile;
		std::shared_ptr<MeshT>    m_mesh;
		RenderedMeshT::Ptr        m_rm;
		BoundingBox               m_bbox;

		float                     m_clippingHeight = 0.f;
		float                     m_clipRangeMin;
		float                     m_clipRangeMax;
};

#include "SingleMesh.inl"

} // FW

#endif /* SINGLEMESHVIS_H_ */
