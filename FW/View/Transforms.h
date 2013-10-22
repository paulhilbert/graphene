#ifndef FWTRANSFORMS_H_
#define FWTRANSFORMS_H_

#include <include/common.h>

namespace FW {
namespace View {

/**
 *  @brief Access class for OpenGL transformation parameters.
 *
 *  A shared_ptr to this class is supplied to all visualizer handles in order
 *  to access transformation matrices and viewport information.
 *
 *  Example usage in visualizer code may look like:
 *  
 *      void CustomVisualizer::render() {
 *          m_shaderProgram.use();
 *          Matrix4f mvMatrix = fw()->transforms()->modelview();
 *          Matrix4f prMatrix = fw()->transforms()->projection();
 *          Vector4i viewport = fw()->transforms()->viewport();
 *          int wndWidth  = viewport[2];
 *          int wndHeight = viewport[3];
 *          m_shaderProgram.setUniformMat4("mvM", mvMatrix.data());
 *          m_shaderProgram.setUniformMat4("prM", prMatrix.data());
 *          m_shaderProgram.setUniform1i("wndWidth", wndWidth);
 *          m_shaderProgram.setUniform1i("wndHeight", wndHeight);
 *      }
 *
 *  This code would supply transformation matrices and window size to a GLSL shader pair.
*/
struct Transforms {
	typedef std::shared_ptr<Transforms> Ptr;
	typedef std::weak_ptr<Transforms>   WPtr;

	public:
		/**
		 * Access viewport parameters.
		 * Returns a reference to 4D viewport parameter vector.
		 * In graphene viewport parameters have the form (0, 0, wndWidth, wndHeight).
		 * @return 4D integer Eigen-vector representing opengl viewport parameters.
		 *
		*/
		Vector4i&  viewport() { return m_viewport; }

		/**
		 * Access viewport parameters.
		 * Returns a const reference to 4D viewport parameter vector.
		 * In graphene viewport parameters have the form (0, 0, wndWidth, wndHeight).
		 * @return 4D integer vector representing opengl viewport parameters.
		 *
		*/
		const Vector4i&  viewport() const { return m_viewport; }

		/**
		 * Access modelview matrix.
		 * Returns reference to 4x4 float matrix representing the modelview transformation.
		 * For passing the matrix to OpenGL code use the results data() member.
		 * @return 4x4 float matrix representing opengl modelview matrix.
		 *
		*/
		Matrix4f&  modelview() { return m_modelview; }

		/**
		 * Access modelview matrix.
		 * Returns const reference to 4x4 float matrix representing the modelview transformation.
		 * For passing the matrix to OpenGL code use the results data() member.
		 * @return 4x4 float matrix representing opengl modelview matrix.
		 *
		*/
		const Matrix4f&  modelview() const { return m_modelview; }

		/**
		 * Access projection matrix.
		 * Returns reference to 4x4 float matrix representing the projection transformation.
		 * For passing the matrix to OpenGL code use the results data() member.
		 * @return 4x4 float matrix representing opengl projection matrix.
		 *
		*/
		Matrix4f&  projection() { return m_projection; }

		/**
		 * Access projection matrix.
		 * Returns const reference to 4x4 float matrix representing the projection transformation.
		 * For passing the matrix to OpenGL code use the results data() member.
		 * @return 4x4 float matrix representing opengl projection matrix.
		 *
		*/
		const Matrix4f&  projection() const { return m_projection; }

		/**
		 * Access normal matrix.
		 * Returns reference to 4x4 float matrix representing the normal transformation.
		 * The normal transformation is the inverse-transpose of the modelview matrix and
		 * should be used in order to transform normals the same way the modelview matrix
		 * transforms points.
		 * For passing the matrix to OpenGL code use the results data() member.
		 * @return 4x4 float matrix representing opengl normal matrix.
		 *
		*/
		Matrix3f&  normal() { return m_normal; }

		/**
		 * Access normal matrix.
		 * Returns const reference to 4x4 float matrix representing the normal transformation.
		 * The normal transformation is the inverse-transpose of the modelview matrix and
		 * should be used in order to transform normals the same way the modelview matrix
		 * transforms points.
		 * For passing the matrix to OpenGL code use the results data() member.
		 * @return 4x4 float matrix representing opengl normal matrix.
		 *
		*/
		const Matrix3f&  normal() const { return m_normal; }

	protected:
		Vector4i  m_viewport;
		Matrix4f  m_modelview;
		Matrix4f  m_projection;
		Matrix3f  m_normal;
};

} // View
} // FW


#endif /* FWTRANSFORMS_H_ */
