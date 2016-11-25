#ifndef RENDEREDCLOUD_H_
#define RENDEREDCLOUD_H_

#include <OpenMesh/Core/IO/MeshIO.hh>

#include <include/common.h>

#include <harmont/harmont.hpp>
#include <harmont/pcl_traits.hpp>

namespace Rendered {

template <typename PointT>
class Cloud {
	public:
		typedef std::shared_ptr<Cloud>        Ptr;
		typedef std::weak_ptr<Cloud>          WPtr;
		typedef std::shared_ptr<const Cloud>  ConstPtr;
		typedef std::weak_ptr<const Cloud>    ConstWPtr;

		typedef harmont::cloud<PointT>        CloudT;

	public:
		Cloud(typename CloudT::Ptr cloud, Eigen::Vector4f default_color = Eigen::Vector4f::Ones());
		virtual ~Cloud();

		void init(harmont::shader_program::ptr program, harmont::pass_type_t type);
		void render(harmont::shader_program::ptr program, harmont::pass_type_t type);

	protected:
		typename CloudT::Ptr                  m_cloud;
		harmont::vertex_array::ptr            m_vaoShadow;
		harmont::vertex_array::ptr            m_vaoDisplay;
		harmont::vertex_buffer<float>::ptr    m_vboShadow;
		harmont::vertex_buffer<float>::ptr    m_vboDisplay;
		harmont::index_buffer<uint32_t>::ptr  m_ibo;
		uint32_t                              m_numElements;
};

#include "Cloud.inl"

} // Rendered

#endif /* RENDEREDCLOUD_H_ */
