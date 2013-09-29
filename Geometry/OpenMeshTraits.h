#ifndef OPENMESHTRAITS_H
#define OPENMESHTRAITS_H

#include <vector>
#include <functional>
#include <Eigen/Dense>

namespace Geometry {


template <class OpenMeshType>
struct OpenMeshTraits {
	typedef OpenMeshType MeshType;
	typedef typename OpenMeshType::Scalar ScalarType;
	typedef typename OpenMeshType::Point  PointType;
	typedef int VertexId;
	typedef int FaceId;
	typedef int Size;
	typedef Eigen::Vector3f Normal;

	// points
	static Size                    numPoints(const MeshType& mesh);
	static std::vector<VertexId>   vertexIdSet(const MeshType& mesh);
	static std::vector<PointType>  pointSet(const MeshType& mesh);
	static void                    mapPoints(MeshType& mesh, std::function<void (PointType&)> func);

	// faces
	static Size                    numFaces(const MeshType& mesh);
	static std::set<FaceId>        faceIdSet(const MeshType& mesh);
	static std::vector<VertexId>   faceVertices(const MeshType& mesh, const FaceId& face);
	static std::vector<PointType>  facePoints(const MeshType& mesh, const FaceId& face);
	static Normal                  faceNormal(const MeshType& mesh, const FaceId& face);
	static std::set<FaceId>        adjacentFaces(const MeshType& mesh, FaceId f);

	// operators
	static ScalarType              norm(const PointType& p);
	static PointType               crossP(const PointType& p0, const PointType& p1);
};

#include "OpenMeshTraits.inl"

} // Geometry

#endif // OPENMESHTRAITS_H
