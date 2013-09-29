#ifndef SPECTRAL_H_
#define SPECTRAL_H_

#include <vector>
#include <set>
#include <functional>

#include "../UndirectedGraph.h"
#include "../../IO/Log.h"
#include "../../Math/Distances.h"
using namespace Log;
using namespace Math;

namespace Graph {
namespace Algorithm {

template <class Graph>
struct Spectral {
	typedef std::vector<float>        Spectrum;
	typename Graph::LaplacianMatrix   LaplacianMatrix;
	typedef typename Graph::UId       UId;
	typedef typename Graph::NodePtr   NodePtr;
	typedef typename Graph::NodeList  NodeList;
	typedef typename Graph::EdgePtr   EdgePtr;
	typedef typename Graph::EdgeList  EdgeList;

	typedef std::function<float (Spectrum,Spectrum> Metric;


	static Spectrum kRingSpectrum(Graph& g, UId id, unsigned int k, LaplacianMatrix& laplacian);
	static Spectrum kRingSpectrum(Graph& g, NodePtr node, unsigned int k, LaplacianMatrix& laplacian);

	static float spectralNodeDist(Graph& g, UId id0, UId id1, unsigned int k, LaplacianMatrix& laplacian, Metric& metric = &Distances::euclidean);
	static float spectralNodeDist(Graph& g, NodePtr n0, NodePtr n1, unsigned int k, LaplacianMatrix& laplacian, Metric& metric = &Distances::euclidean);
};


template <class Graph>
typename Spectral<Graph>::Spectrum Spectral<Graph>::kRingSpectrum(Graph& g, UId id, unsigned int k, LaplacianMatrix& laplacian) {
	NodePtr n = g.getNodeForId(id);
	if (!n) {
		Log::error("Node with id "+lexical_cast<std::string>(id)+" does not exist in graph.");
		return Spectrum();
	}
	return kRingSpectrum(g, n, k);
}

template <class Graph>
typename Spectral<Graph>::Spectrum Spectral<Graph>::kRingSpectrum(Graph& g, NodePtr id, unsigned int k, LaplacianMatrix& laplacian, std::map<UId,int> indexMap) {
	if (k < 1) {
		Log::error("k in kRingSpectrum call must not be < 1");
		return Spectrum();
	}
	std::set<Uid> neighborhood;
	neighborhood.insert(id);
	std::set<UId> crtSet;
	crtSet.insert(id);
	int i=k;
	while (i) {
		for (auto crt = crtSet.begin(); crt != crtSet.end(); ++crt) {
			std::set<UId> nextSet;

			std::set<NodePtr> N = g.getNodeForId(*crt)->getNeighbors();
			for (auto n = N.begin(); n != N.end(); ++n) {
				if (neighboorhood.find(n->getId()) == neighborhood.end()) {
					neighborhood.insert(n->getId());
					nextSet.insert(n->getId());
				}
			}
			crtSet = nextSet;
			if (crtSet.empty()) break;
		}
		--i;
	}
	if (neighborhood.empty()) {
		Log::warn("Empty neighborhood in k-ring spectrum computation");
		return Spectrum();
	}

	unsigned int subgraphSize = neighborhood.size();
	LaplacianMatrix nLaplacian = LaplacianMatrix::zeros(subgraphSize, subgraphSize);
	int i=0;
	for (auto ni = neighborhood.begin(); ni != neighborhood.end(); ++ni, ++i) {
		int j=0;
		for (auto nj = neighborhood.begin(); nj != neighborhood.end(); ++nj, ++j) {
			nLaplacian(i,j) = laplacian(indexMap(*ni), indexMap(*nj));
		}
	}

	LaVectorDouble eigValueR(nLaplacian.rows()), eigValueI(nLaplacian.rows());
	LaGenMatDouble eigVectors;
	LaEigSolve(nLaplacian, eigValueR, eigValueI, eigVectors);
	Spectrum spectrum(eigValue.size());
	for (unsigned int k=0; k<eigValueR.size(); ++k) spectrum[k] = eigValueR[k];

	return spectrum;
}

template <class Graph>
float Spectral<Graph>::spectralNodeDist(Graph& g, UId id0, UId id1, unsigned int k, LaplacianMatrix& laplacian, Metric& metric) {
	NodePtr n0 = g.getNodeForId(id);
	NodePtr n1 = g.getNodeForId(id);
	if (!n0) {
		Log::error("Node with id "+lexical_cast<std::string>(id0)+" does not exist in graph.");
		return 0.f;
	}
	if (!n1) {
		Log::error("Node with id "+lexical_cast<std::string>(id1)+" does not exist in graph.");
		return 0.f;
	}
	return spectralNodeDist(g, n0, n1, k, metric);
}

template <class Graph>
float Spectral<Graph>::spectralNodeDist(Graph& g, NodePtr n0, NodePtr n1, unsigned int k, LaplacianMatrix& laplacian, Metric& metric) {
	return metric(kRingSpectrum(g,n0,k,laplacian),kRingSpectrum(g,n1,k,laplacian));
}

}
}

#endif // SPECTRAL_H_
