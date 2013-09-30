#ifndef FWTRANSFORMS_H_
#define FWTRANSFORMS_H_

#include <memory>
#include <Eigen/Dense>

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

	protected:
		Eigen::Vector4i  m_viewport;
		Eigen::Matrix4f  m_modelview;
		Eigen::Matrix4f  m_projection;
		Eigen::Matrix3f  m_normal;
};

} // View
} // FW


#endif /* FWTRANSFORMS_H_ */
