#ifndef CUBLASDIST_H
#define CUBLASDIST_H

// STL
#include <vector>

#include "CublasMatrix.h"
#include "CublasOps.h"

namespace GPU {

class CublasDist {
	public:
		CublasDist();
		virtual ~CublasDist();

		void setTargetPoints(std::vector<std::vector<float>> points);
		std::vector<std::vector<float>> dist(std::vector<std::vector<float>> pt);

	protected:
		CublasMatrix m_matT;
		CublasMatrix m_matQ;
		CublasMatrix m_matTSq;
		CublasMatrix m_matQSq;
		CublasMatrix m_matOnesT;
		CublasMatrix m_matOnesQ;
		CublasMatrix m_matOnesD;
		CublasMatrix m_v1;
		CublasMatrix m_v2;
		CublasMatrix m_dists;

		CublasOps m_ops;
};

} // GPU

#endif // CUBLASDIST_H
