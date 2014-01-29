#ifndef NORMALFIELD_H_
#define NORMALFIELD_H_

#include <boost/optional.hpp>
#include <boost/none.hpp>
using boost::optional;
using boost::none;

#include <include/common.h>

#include <Library/Colors/Map.h>
using Colors::RGBA;

#include "../Buffer/Geometry.h"
#include "../Shader/ShaderProgram.h"
using Shader::ShaderProgram;

namespace Rendered {

class NormalField {
	public:
		typedef std::shared_ptr<NormalField> Ptr;
		typedef std::weak_ptr<NormalField> WPtr;
		typedef std::shared_ptr<const NormalField> ConstPtr;
		typedef std::weak_ptr<const NormalField> ConstWPtr;

	public:
		NormalField(RGBA baseColor);
		virtual ~NormalField();


		void setVisible(bool visible);
		bool getVisible() const;

		template <typename Func>

	protected:
		RGBA m_color;
		unsigned int m_pointCount;
		std::shared_ptr<Buffer::Geometry> m_geometry;
		bool m_visible;
};

#include "NormalField.inl"

} // Rendered

#endif /* NORMALFIELD_H_ */
