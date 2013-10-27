#ifndef EIGENOPENGL_H_
#define EIGENOPENGL_H_

#include <Eigen/Dense>
using Eigen::Matrix;
using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::Vector3f;
using Eigen::Vector4f;

namespace Eigen {

Matrix4f scale(const Matrix4f& m, const Vector3f& scale);

Matrix4f translate(const Matrix4f& m, const Vector3f& translate);

Matrix4f ortho(float left, float right, float bottom, float top, float zNear, float zFar);

Matrix4f ortho(float left, float right, float bottom, float top);

Matrix4f perspective( float fovy, float aspect,	float zNear, float zFar);

Matrix4f frustum( float left, float right, float bottom, float top, float nearVal, float farVal );

template <class U>
Vector3f project(const Vector3f& obj, const Matrix4f& model, const Matrix4f& proj, const Matrix<U,4,1>& viewport);

template <class U>
Vector3f unProject(const Vector3f& win, const Matrix4f& model, const Matrix4f& proj, const Matrix<U,4,1>& viewport);

#include "OpenGL.inl"

} // Eigen

#endif /* EIGENOPENGL_H_ */
