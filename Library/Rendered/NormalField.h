#ifndef NORMALFIELD_H_
#define NORMALFIELD_H_

#include "Field.h"

namespace Rendered {

class NormalField : public Field {
	public:
		typedef std::shared_ptr<NormalField> Ptr;
		typedef std::weak_ptr<NormalField> WPtr;
		typedef std::shared_ptr<const NormalField> ConstPtr;
		typedef std::weak_ptr<const NormalField> ConstWPtr;

	public:
		NormalField(RGBA baseColor, RenderKernel::Ptr kernel);
		virtual ~NormalField();

		void set(const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>& normals);
		void render(const Eigen::Matrix4f& mvMatrix, const Eigen::Matrix4f& prMatrix, const Eigen::Matrix3f& nmMatrix);

	protected:
		void set(const std::vector<Eigen::Vector3f>& points) {}
};

#include "NormalField.inl"

} // Rendered

#endif /* NORMALFIELD_H_ */
