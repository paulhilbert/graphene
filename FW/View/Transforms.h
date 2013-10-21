#ifndef FWTRANSFORMS_H_
#define FWTRANSFORMS_H_

#include <include/common.h>

namespace FW {
namespace View {

struct Transforms {
	typedef std::shared_ptr<Transforms> Ptr;
	typedef std::weak_ptr<Transforms>   WPtr;

	public:
		Eigen::Vector4i&  viewport() { return m_viewport; }
		Eigen::Matrix4f&  modelview() { return m_modelview; }
		Eigen::Matrix4f&  projection() { return m_projection; }
		Eigen::Matrix3f&  normal() { return m_normal; }
		float&            near() { return m_near; }
		float&            far() { return m_far; }

	protected:
		Eigen::Vector4i  m_viewport;
		Eigen::Matrix4f  m_modelview;
		Eigen::Matrix4f  m_projection;
		Eigen::Matrix3f  m_normal;
		float            m_near;
		float            m_far;
};

} // View
} // FW


#endif /* FWTRANSFORMS_H_ */
