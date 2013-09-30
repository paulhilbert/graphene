#include <iostream> // for graphics
#include <fstream>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
typedef OpenMesh::TriMesh_ArrayKernelT<OpenMesh::DefaultTraits> Mesh;

#include "MeshTransform.h"
#include "OpenMeshTraits.h"
using namespace Geometry;

typedef OpenMeshTraits<Mesh>   Traits;
typedef Traits::PointType      Point;
typedef MeshAnalysis<Traits>   MA;
typedef MeshTransform<Traits>  MT;

int main(int argc, char* argv[]) {
	Mesh mesh;
	mesh.request_vertex_status();
	mesh.request_face_status();
	mesh.request_vertex_normals();
	mesh.request_face_normals();
	if (!OpenMesh::IO::read_mesh(mesh, "sphere.obj")) {
		std::cout << "Could not read mesh\n";
		exit(1);
	}
	mesh.update_normals();
	auto samples = MA::sampleOnSurface(mesh, 20);
	std::ofstream out("samples.obj");
	for (auto it = samples.begin(); it != samples.end(); ++it) {
		out << "v " << *it << "\n";
	}
	out.close();
}
