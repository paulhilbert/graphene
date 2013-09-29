#ifndef CUDACODEBOOK_H_
#define CUDACODEBOOK_H_

#include <vector>
using std::vector;

#include <functional>
#include <algorithm>

#include <boost/algorithm/string.hpp>

#include "Codebook.h"

#include "../IO/Log.h"
using namespace IO;

#include "../GPU/CublasDist.h"
using namespace GPU;

namespace IR {

class CudaCodebook : public Codebook<float, std::vector<float>> {
	public:
		typedef vector<float>     Desc;
		typedef vector<Desc>      Descs;
		typedef vector<float>     Embedding;
		typedef vector<Embedding> Embeddings;
		typedef std::function<Embedding (Embeddings&)>      Combinator;

	public:
		CudaCodebook();

		virtual void deserialize(std::string content);
		void setDescriptors(const std::vector<std::vector<float>>& descs);

		Embedding embedNN(const Descs& descs, const Combinator& combinator);

	protected:
		unsigned int argmin(const vector<float>& values) const;

	protected:
		CublasDist m_dist;
		bool       m_allocated;
};

CudaCodebook::CudaCodebook() : Codebook<float,std::vector<float>>, m_allocated(false) {
}

void CudaCodebook::deserialize(std::string content) {
	this->m_descs.clear();

	vector<std::string> lines;
	boost::split(lines, content, boost::is_any_of("\n\r"));

	unsigned int codebookSize = lexical_cast<unsigned int>(lines[0]);
	unsigned int codebookDim  = lexical_cast<unsigned int>(lines[1]);

	vector<std::string> values;
	Desc desc;
	std::string line;
	for (unsigned int i=2; i<lines.size(); ++i) {
		line = lines[i];
		values.clear();
		boost::split(values, line, boost::is_any_of(" \t"));
		desc = Desc(values.size());

		if (values.size() != codebookDim) continue;
		// check if at least the first value is castable
		try {
			lexical_cast<float>(values[0]);
		} catch (...) {
			continue;
		}
		std::transform(values.begin(), values.end(), desc.begin(), &lexical_cast<float,std::string>);
		this->m_descs.push_back(desc);
	}

	if (this->m_descs.size() != codebookSize) Log::warn("Descriptor count in codebook file is inconsistent!");
}

void CudaCodebook::setDescriptors(const std::vector<std::vector<float>>& descs) {
	m_descs = descs;
}

Embedding CudaCodebook::embedNN(const Descs& descs, const Combinator& combinator) {
	if (!m_allocated) {
		m_dist.setTargetPoints(m_descs);
		m_allocated = true;
	}
	auto dists = m_dists.dist(descs);
	vector<unsigned int> nns(dists.size());
	std::transform(dists.begin(), dists.end(), nns.begin(), &argmin);
	Embeddings es;
	for (unsigned int i=0; i<nns.size(); ++i) {
		Embedding e(m_descs.size(), 0.f);
		e[nns[i]] = 1.f;
		es.push_back(e);
	}
	return combinator(es);
}

unsigned int argmin(const vector<float>& values) const {
	float min = values[0];
	int result = 0;
	for (unsigned int i=1; i<values.size(); ++i) {
		if (values[i] >= min) continue;
		min = values[i];
		result = i;
	}
	return result;
}

} // IR

#endif /* CUDACODEBOOK_H_ */
