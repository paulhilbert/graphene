#ifndef CODEBOOK_H
#define CODEBOOK_H

#include <vector>
using std::vector;

#include <functional>

#include <cmath>
#include <algorithm>
#include <numeric>
#include <limits>

#include <IO/Log.h>
using namespace IO;


namespace IR {

template <class TScalar, class TDesc>
class Codebook {
	public:
		typedef vector<TDesc>     Descs;
		typedef vector<TScalar>   Embedding;
		typedef vector<Embedding> Embeddings;

#ifdef USE_CUDA
		typedef std::function<TScalar (TDesc,TDesc,unsigned int)>  CudaMetric;
#endif
		typedef std::function<TScalar (const TDesc&, const TDesc&)>        Metric;
		typedef std::function<Embedding (Embeddings&)>      Combinator;

	public:
		Codebook();

		virtual void deserialize(std::string content) = 0;

		TDesc& operator[](unsigned int idx);
		const TDesc& operator[](unsigned int idx) const;

		Descs&  getDescriptors();
		const Descs&  getDescriptors() const;

#ifdef USE_CUDA
		Embedding nnEmbedding(const TDesc& desc, const CudaMetric& metric, const vector<unsigned int>& offsets);
		Embedding gibbsEmbedding(const TDesc& desc, const CudaMetric& metric, const vector<unsigned int>& offsets, bool warnOnZeroEmbedding = true);
#endif
		Embedding nnEmbedding(const TDesc& desc, const Metric& metric) const;
		Embedding gibbsEmbedding(const TDesc& desc, const Metric& metric, bool warnOnZeroEmbedding = true) const;

#ifdef USE_CUDA
		Embedding nnEmbedding(const Descs& descs, const CudaMetric& metric, const Combinator& combinator, const vector<unsigned int>& offsets);
		Embedding gibbsEmbedding(const Descs& descs, const CudaMetric& metric, const Combinator& combinator, const vector<unsigned int>& offsets, bool warnOnZeroEmbedding = true);
#endif
		Embedding nnEmbedding(const Descs& descs, const Metric& metric, const Combinator& combinator) const;
		Embedding gibbsEmbedding(const Descs& descs, const Metric& metric, const Combinator& combinator, bool warnOnZeroEmbedding = true) const;

	protected:
		unsigned int argmin(const vector<TScalar>& values) const;

	protected:
		Descs  m_descs;
};


template <class TScalar, class TDesc>
Codebook<TScalar,TDesc>::Codebook() {
}

template <class TScalar, class TDesc>
inline TDesc& Codebook<TScalar,TDesc>::operator[](unsigned int idx) {
	if (idx >= m_descs.size()) {
		Log::error("Codebook index out of range.");
		return;
	}
	return m_descs[idx];
}

template <class TScalar, class TDesc>
inline const TDesc& Codebook<TScalar,TDesc>::operator[](unsigned int idx) const {
	if (idx >= m_descs.size()) {
		Log::error("Codebook index out of range.");
		return;
	}
	return m_descs[idx];
}

template <class TScalar, class TDesc>
typename Codebook<TScalar,TDesc>::Descs& Codebook<TScalar,TDesc>::getDescriptors() {
	return m_descs;
}

template <class TScalar, class TDesc>
const typename Codebook<TScalar,TDesc>::Descs& Codebook<TScalar,TDesc>::getDescriptors() const {
	return m_descs;
}

#ifdef USE_CUDA
template <class TScalar, class TDesc>
typename Codebook<TScalar,TDesc>::Embedding Codebook<TScalar,TDesc>::nnEmbedding(const TDesc& desc, const CudaMetric& metric, const vector<unsigned int>& offsets) {
	vector<TScalar> dists(m_descs.size(), 0.f);
	for (unsigned int i=0; i<m_descs.size(); ++i) {
		dists[i] = metric(desc, m_descs[i], offsets[i]);
	}
	Embedding embedding(m_descs.size(), TScalar(0));
	embedding[argmin(dists)] = TScalar(1);
	
	return embedding;
}
#endif

template <class TScalar, class TDesc>
typename Codebook<TScalar,TDesc>::Embedding Codebook<TScalar,TDesc>::nnEmbedding(const TDesc& desc, const Metric& metric) const {
	vector<TScalar> dists(m_descs.size(), 0.f);
	for (unsigned int i=0; i<m_descs.size(); ++i) {
		dists[i] = metric(desc, m_descs[i]);
	}
	Embedding embedding(m_descs.size(), TScalar(0));
	embedding[argmin(dists)] = TScalar(1);
	
	return embedding;
}

#ifdef USE_CUDA
template <class TScalar, class TDesc>
typename Codebook<TScalar,TDesc>::Embedding Codebook<TScalar,TDesc>::gibbsEmbedding(const TDesc& desc, const CudaMetric& metric, const vector<unsigned int>& offsets, bool warnOnZeroEmbedding) {
	Embedding embedding;
	TScalar normalization = TScalar(0);

	unsigned int i = 0;
	std::for_each(m_descs.begin(), m_descs.end(), [&] (TDesc d) {
		TScalar dist = exp(TScalar(-1) * metric(desc, d, offsets[i++]));
		normalization += dist;
		embedding.push_back(dist);
	});
	if (fabs(normalization) < std::numeric_limits<TScalar>::epsilon()) {
		if (warnOnZeroEmbedding) Log::warn("Possible error while embedding descriptor: All distances are 0.");
		return embedding;
	}
	std::transform(embedding.begin(), embedding.end(), embedding.begin(), [=](TScalar v) { return v/normalization; });
	return embedding;
}
#endif

template <class TScalar, class TDesc>
typename Codebook<TScalar,TDesc>::Embedding Codebook<TScalar,TDesc>::gibbsEmbedding(const TDesc& desc, const Metric& metric, bool warnOnZeroEmbedding) const {
	Embedding embedding;
	TScalar normalization = TScalar(0);

	std::for_each(m_descs.begin(), m_descs.end(), [&] (const TDesc& d) {
		TScalar dist = exp((TScalar(-1) * metric(desc, d)) / TScalar(0.1));
		normalization += dist;
		embedding.push_back(dist);
	});
	if (fabs(normalization) < std::numeric_limits<TScalar>::epsilon()) {
		if (warnOnZeroEmbedding) Log::warn("Possible error while embedding descriptor: All distances are 0.");
		return embedding;
	}
	std::transform(embedding.begin(), embedding.end(), embedding.begin(), [=](TScalar v) { return v/normalization; });
	return embedding;
}

#ifdef USE_CUDA
template <class TScalar, class TDesc>
typename Codebook<TScalar,TDesc>::Embedding Codebook<TScalar,TDesc>::nnEmbedding(const Descs& descs, const CudaMetric& metric, const Combinator& combinator, const vector<unsigned int>& offsets) {
	Embeddings es;
	for (unsigned int i=0; i<descs.size(); ++i) {
		es.push_back(nnEmbedding(descs[i], metric, offsets));
	}
	return combinator(es);
}
#endif

template <class TScalar, class TDesc>
typename Codebook<TScalar,TDesc>::Embedding Codebook<TScalar,TDesc>::nnEmbedding(const Descs& descs, const Metric& metric, const Combinator& combinator) const {
	Embeddings es;
	for (unsigned int i=0; i<descs.size(); ++i) {
		es.push_back(nnEmbedding(descs[i], metric));
	}
	return combinator(es);
}

#ifdef USE_CUDA
template <class TScalar, class TDesc>
typename Codebook<TScalar,TDesc>::Embedding Codebook<TScalar,TDesc>::gibbsEmbedding(const Descs& descs, const CudaMetric& metric, const Combinator& combinator, const vector<unsigned int>& offsets, bool warnOnZeroEmbedding) {
	Embeddings es;
	for (unsigned int i=0; i<descs.size(); ++i) {
		es.push_back(gibbsEmbedding(descs[i], metric, offsets, warnOnZeroEmbedding));
	}
	return combinator(es);
}
#endif

template <class TScalar, class TDesc>
typename Codebook<TScalar,TDesc>::Embedding Codebook<TScalar,TDesc>::gibbsEmbedding(const Descs& descs, const Metric& metric, const Combinator& combinator, bool warnOnZeroEmbedding) const {
	Embeddings es;
	for (unsigned int i=0; i<descs.size(); ++i) {
		es.push_back(gibbsEmbedding(descs[i], metric, warnOnZeroEmbedding));
	}
	return combinator(es);
}

template <class TScalar, class TDesc>
inline unsigned int Codebook<TScalar,TDesc>::argmin(const vector<TScalar>& values) const {
	TScalar min = values[0];
	int result = 0;
	for (unsigned int i=1; i<values.size(); ++i) {
		if (values[i] >= min) continue;
		min = values[i];
		result = i;
	}
	return result;
}

} // IR

#endif // CODEBOOK_H
