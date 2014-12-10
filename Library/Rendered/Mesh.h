#ifndef RENDEREDMESH_H_
#define RENDEREDMESH_H_

#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <Library/Geometry/OpenMeshTraits.h>

#include <include/common.h>
#include <include/ogl.h>

#include <Library/Buffer/Geometry.h>
#include <Library/Shader/ShaderProgram.h>
using namespace Geometry;
using Shader::ShaderProgram;

namespace Rendered {

template <class MeshType>
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

		/** Shared pointer to MeshType */
		typedef std::shared_ptr<MeshType> MeshPtr;
		/** Traits class for MeshType */
		typedef OpenMeshTraits<MeshType> Traits;

	public:
		/**
		 *  Constructor.
		 *
		 *  @param filePath Path to mesh file.
		 *  @param smoothNormals Smoothly interpolate normals per fragment
		 *  @param If true keeps both representations in order to allow switching later
		 */
		Mesh(MeshPtr mesh, bool smoothNormals, bool allowSwitching = true);

		/** @internal ~Mesh()
		 *
		 *  @brief Destructor.
		 */
		virtual ~Mesh();

		/** Renders mesh */
		void render(ShaderProgram& program);

		/**
		 *  Changes normal interpolation mode.
		 *
		 *  @warning This method throws a runtime error when this class's allowSwitching constructor parameter is false
		 *  @param smoothNormals Smoothly interpolate normals per fragment
		 */
		void setSmoothNormals(bool smoothNormals);

        void setUniformColor(const Eigen::Vector4f& color);
        void setUniformColor(const Eigen::Vector4f& color, const std::vector<int>& vertexSubset);

	protected:
		void  init();
		void  upload();


	protected:
		typedef std::vector<Vector3f>  Vertices;
		typedef std::vector<Vector3f>  Normals;
		typedef std::vector<Vector4f>  Colors;
		typedef std::vector<GLuint>    Indices;
		typedef std::shared_ptr<Buffer::Geometry> GeometryPtr;

	protected:
		MeshPtr             m_mesh;
		bool                m_smooth;
		bool                m_allowSwitching;
		GeometryPtr         m_geometry;

		Vertices            m_smoothVertices;
		Vertices            m_flatVertices;
		Normals             m_smoothNormals;
		Normals             m_flatNormals;
		Colors              m_smoothColors;
		Colors              m_flatColors;
		Indices             m_smoothIndices;
		Indices             m_flatIndices;
};

#include "Mesh.inl"

} // Rendered

#endif /* RENDEREDMESH_H_ */
