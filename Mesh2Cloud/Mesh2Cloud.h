#ifndef MESH2CLOUDVIS_H_
#define MESH2CLOUDVIS_H_

#include <FW/Visualizer.h>
#include <FW/Factory.h>

#include <IO/AbstractProgressBar.h>

#include <Geometry/PCLTools.h>
using Geometry::PCLTools;
typedef pcl::PointNormal Point;
typedef PCLTools<Point>  Tools;
typedef Tools::CloudType Cloud;
typedef Tools::Idx       Idx;
typedef Tools::IdxSet    IdxSet;

#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <Geometry/OpenMeshTraits.h>
#include <Geometry/MeshAnalysis.h>
typedef OpenMesh::PolyMesh_ArrayKernelT<> Mesh;
using namespace Geometry;
typedef OpenMeshTraits<Mesh> Traits;
typedef MeshAnalysis<Traits> MA;

#include <Library/Rendered/Cloud.h>


namespace FW {


class Mesh2Cloud : public Visualizer {
	public:
		class Factory;
		typedef IO::AbstractProgressBar PBar;

	public:
		Mesh2Cloud(std::string id, fs::path meshPath);
		virtual ~Mesh2Cloud();

		void init();
		void render();
		void addProperties();
		void registerEvents();

	protected:
		void sample(int samplesPerSquareUnit);

	protected:
		Mesh                  m_mesh;
		Cloud::Ptr            m_cloud;
		Rendered::Cloud::Ptr  m_rc;
};


class Mesh2Cloud::Factory : public FW::Factory {
	public:
		Factory();
		virtual ~Factory();

		void init();
		Visualizer::Ptr addVisualizer();
};


} // FW

#endif /* MESH2CLOUDVIS_H_ */
