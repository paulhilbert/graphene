#ifndef SINGLEMESHVIS_H_
#define SINGLEMESHVIS_H_

#include <FW/FWVisualizer.h>

#include <Library/Geometry/OpenMeshTraits.h>
#include <Library/Rendered/Mesh.h>

namespace FW {

class SingleMesh : virtual public Visualizer {
	public:
		typedef std::shared_ptr<SingleMesh> Ptr;
		typedef std::weak_ptr<SingleMesh>   WPtr;

		typedef Geometry::TriMesh Mesh;
		typedef Geometry::OpenMeshTraits<Mesh> Traits;

	public:
		SingleMesh(std::string id, fs::path meshFile);
		virtual ~SingleMesh();

		void init();
		void render(ShaderProgram& program);
		void addProperties();
		void addModes();
		void registerEvents();

		virtual BoundingBox boundingBox() const;

	protected:
		fs::path                  m_meshFile;
		std::shared_ptr<Mesh>     m_mesh;
		Rendered::Mesh<Mesh>::Ptr m_rm;
		BoundingBox               m_bbox;

		float                     m_clippingHeight = 0.f;
		float                     m_clipRangeMin;
		float                     m_clipRangeMax;
};

#include "SingleMesh.inl"

} // FW

#endif /* SINGLEMESHVIS_H_ */
