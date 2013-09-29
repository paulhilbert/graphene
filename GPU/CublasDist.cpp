#include "CublasDist.h"

namespace GPU {

CublasDist::CublasDist() {

}

CublasDist::~CublasDist() {

}

void CublasDist::setTargetPoints(std::vector<std::vector<float>> points) {
	int num = points.size();
	int dim = points[0].size();

	m_matT.allocate(num, dim);
	m_matTSq.allocate(num, dim);
	m_matOnesT.allocate(1, num);
	m_matOnesD.allocate(1, dim);
	m_v2.allocate(num, 1);

	m_matT.setFromVectors(points);
	m_matTSq.setFromVectors(points, [] (float v) {return v*v;});
	m_matOnesT.fill(1.f);
	m_matOnesD.fill(1.f);

	m_ops.matMatMult(m_matTSq, m_matOnesD, m_v2, false, true);
}

std::vector<std::vector<float>> CublasDist::dist(std::vector<std::vector<float>> pt) {
	unsigned int queryCount = pt.size();

	m_matOnesQ.allocate(1, queryCount);
	m_matOnesQ.fill(1.f);

	m_v1.allocate(queryCount, 1);
	m_dists.allocate(queryCount, m_matT.numRows());

	m_matQ.setFromVectors(pt);
	m_matQSq.setFromVectors(pt, [] (float v) {return v*v;});

	m_ops.matMatMult(m_matQSq, m_matOnesD, m_v1, false, true);

	m_ops.matMatMult(m_matQ, m_matT, m_dists, false, true);
	m_ops.matMatMult(m_v1, m_matOnesT, m_dists, false, false, 1.f, -2.f);
	m_ops.matMatMult(m_matOnesQ, m_v2, m_dists, true, true, 1.f, 1.f);

	return m_dists.getAsVectors();
}

} // GPU
