#ifndef RENDEREDMESH_H_
#define RENDEREDMESH_H_

#include <OpenMesh/Core/IO/MeshIO.hh>

#include <include/common.h>

#include <harmont/harmont.hpp>
#include <harmont/openmesh_traits.hpp>

namespace Rendered {

template <class ColorT = OpenMesh::Vec4f>
class Mesh {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<Mesh> Ptr;
		/** Weak pointer to this class */
		typedef std::weak_ptr<Mesh>   WPtr;
		/** Const shared pointer to this class */
		typedef std::shared_ptr<const Mesh> ConstPtr;
		/** Const weak pointer to this class */
		typedef std::weak_ptr<const Mesh> ConstWPtr;

        typedef harmont::tri_mesh<ColorT>  MeshT;
        typedef std::shared_ptr<MeshT>     MeshPtrT;

	public:
		Mesh(MeshPtrT mesh, bool smoothNormals);
		virtual ~Mesh();

		void init(harmont::shader_program::ptr program, harmont::pass_type_t type);
		void render(harmont::shader_program::ptr program, harmont::pass_type_t type);

	protected:
		void  upload();

	protected:
		MeshPtrT                              m_mesh;
        harmont::vertex_array::ptr            m_vaoShadow;
        harmont::vertex_array::ptr            m_vaoDisplay;
        harmont::vertex_buffer<float>::ptr    m_vboShadow;
        harmont::vertex_buffer<float>::ptr    m_vboDisplay;
        harmont::index_buffer<uint32_t>::ptr  m_ibo;
        uint32_t                              m_numElements;
        bool                                  m_smooth;
};

#include "Mesh.inl"

} // Rendered

#endif /* RENDEREDMESH_H_ */
