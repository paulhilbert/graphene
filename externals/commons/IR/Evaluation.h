#ifndef EVALUATION_H
#define EVALUATION_H

#include <algorithm> // For sort
using std::sort;
#include <map>
using std::map;
#include <cstring>
using std::string;
#include <utility> // For pair
using std::pair;
using std::make_pair;
#include <vector>
using std::vector;
#include <iomanip>
#include <ctime>
#include <cstdlib>

#include <Random/RNG.h>
using Random::RNG;

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#include <boost/lexical_cast.hpp>

#include "Ranking.h"

//#define COMPUTE_DCG

namespace IR {

template <class TClassId, class TObjId, class TEmbedding, class TScalar>
class Evaluation {
public:
	typedef vector< pair<float, float> > PRData;
	struct PRPlot {
		string name;
		PRData data;
	};
	struct PRPlots {
		vector<PRPlot> perObject;
		vector<PRPlot> perClass;
		PRPlot micro;
		PRPlot macro;
	};
	typedef std::function<TScalar (TEmbedding,TEmbedding)> Metric;

public:

	template <class TClassIter>
	static PRPlots computePRPlots(TClassIter begin, TClassIter end, const Metric& metric, unsigned int numRuns = 1, bool random = false);

	static PRPlot interpolatePlot(const PRPlot& plot, unsigned int resolution = 100);
	static PRPlot meanPlot(const vector<PRPlot>& plots, unsigned int resolution = 0);

	/*
	static void plot(const PRPlot& pr, const fs::path& out, const string title = "Plot");
	static void plot(const PRPlot& pr0, const PRPlot& pr1, const fs::path& out, const string title = "Plot");
	static void plot(const vector<PRPlot>& pr, const fs::path& out, const string title = "Plot");
	*/
	static std::string serializePlot(const PRPlot& pr);
	//static PRPlot deserializePlot(const fs::path& sourceFile);

	template <class TClassIter>
	static float tier(TClassIter begin, TClassIter end, const Metric& metric, bool micro, unsigned int k=1);
protected:
	Evaluation() {}
};

template <class TClassId, class TObjId, class TEmbedding, class TScalar>
template <class TClassIter>
typename Evaluation<TClassId,TObjId,TEmbedding,TScalar>::PRPlots Evaluation<TClassId,TObjId,TEmbedding,TScalar>::computePRPlots(TClassIter begin, TClassIter end, const Metric& metric, unsigned int numRuns, bool random) {
	srand(unsigned(time(NULL)));

	PRPlots prPlots;

	RNG* rng = RNG::instance();

	unsigned int maxClassSize = 0;

#ifdef COMPUTE_DCG
	vector< pair<float, pair<string, string> > > dcgs;
#endif

	for (TClassIter cIt = begin; cIt != end; ++cIt) {
		vector<PRPlot> classPRPlots;
		if (cIt->getNumObjects() > maxClassSize) maxClassSize = cIt->getNumObjects();

		unsigned int numRele = cIt->getNumObjects() - 1;
		TClassId crtClass = cIt->getId();

		cIt->forEachObj([&](typename Class<TClassId,TObjId,TEmbedding>::Object& o) {
			vector<PRPlot> objectPRPlots;
			TObjId crtObj = o.id;

			auto ranking = Ranking<TClassId,TObjId,TEmbedding,TScalar>::rankClasses(crtObj, metric, begin, end);

			for (unsigned int run = 0; run < numRuns; ++run) {
				auto tmpRanking = ranking;
				if (!random) {
					// Check for zero diff scores and perturb if necessary
					bool hasZeroDiff = false;
					float minDiff = 1.f;
					for (size_t i = 0; i < ranking.size()-1; ++i) {
						float diff = ranking[i+1].second - ranking[i].second;
						if (diff > 0.f) {
							if (diff < minDiff) {
								minDiff = diff;
							}
						} else {
							hasZeroDiff = true;
						}
					}

					if (hasZeroDiff) {
						for (size_t i = 0; i < tmpRanking.size(); ++i) {
							tmpRanking[i].second += rng->uniformAB<float>(0.f, 0.5f * minDiff);
						}

						std::sort(tmpRanking.begin(), tmpRanking.end(), [&](std::pair<TClassId,TScalar> a, std::pair<TClassId,TScalar> b) {return a.second < b.second; });
					}
					// END Check for zero diff scores and perturb if necessary
				} else {
					std::random_shuffle(tmpRanking.begin(), tmpRanking.end());
//					std::sort(tmpRanking.begin(), tmpRanking.end(), [&](std::pair<TClassId,TScalar> a, std::pair<TClassId,TScalar> b) {return rng->uniform01<float>() < 0.5f; });
				}

				unsigned int foundRele = 0;

				PRPlot objectPRPlot;
				for (size_t r = 0; r < tmpRanking.size(); ++r) {
					if (tmpRanking[r].first != crtClass) continue;
					++foundRele;
					objectPRPlot.data.push_back( make_pair(static_cast<float>(foundRele) / numRele,
																		static_cast<float>(foundRele) / (r+1)) );
				}

				objectPRPlots.push_back(objectPRPlot);
			}

			PRPlot objectMeanPlot = Evaluation<TClassId,TObjId,TEmbedding,TScalar>::meanPlot(objectPRPlots);
			//if (random) objectMeanPlot.name = "random ";
			objectMeanPlot.name += boost::lexical_cast<std::string>(crtObj);

			prPlots.perObject.push_back(objectMeanPlot);
			classPRPlots.push_back(objectMeanPlot);
		});
		PRPlot classPRPlot = meanPlot(classPRPlots);
		//if (random) classPRPlot.name = "random ";
		classPRPlot.name += boost::lexical_cast<std::string>(crtClass);
		prPlots.perClass.push_back(classPRPlot);
	}

	prPlots.micro = meanPlot(prPlots.perObject, maxClassSize-1);
	//if (random) prPlots.micro.name = "random ";
	prPlots.micro.name += "micro";
	prPlots.macro = meanPlot(prPlots.perClass, maxClassSize-1);
	//if (random) prPlots.macro.name = "random ";
	prPlots.macro.name += "macro";

	return prPlots;
}

template <class TClassId, class TObjId, class TEmbedding, class TScalar>
typename Evaluation<TClassId,TObjId,TEmbedding,TScalar>::PRPlot Evaluation<TClassId,TObjId,TEmbedding,TScalar>::interpolatePlot(const PRPlot& plot, unsigned int resolution) {
	vector<float> domain;
	for (size_t x = 0; x <= resolution; ++x) {
		domain.push_back( static_cast<float>(x) / resolution );
	}

	PRPlot result = plot;
	// Initialize precision values to -1.f -> undefined
	result.data = PRData(domain.size(), pair<float, float>(0.f, -1.f));

	for (size_t i = 0; i < domain.size(); ++i) {
		float t = result.data[i].first = domain[i];
		for (size_t j = 0; j < plot.data.size() - 1; ++j) {
			float x0 = plot.data[j].first;
			float x1 = plot.data[j+1].first;
			if (x0 <= t && x1 >= t) {
				float y0 = plot.data[j].second;
				float y1 = plot.data[j+1].second;
				result.data[i].second = y0 + (y1-y0) * ((t-x0) / (x1-x0));
				break;
			}
		}
	}

	return result;
}

template <class TClassId, class TObjId, class TEmbedding, class TScalar>
typename Evaluation<TClassId,TObjId,TEmbedding,TScalar>::PRPlot Evaluation<TClassId,TObjId,TEmbedding,TScalar>::meanPlot(const vector<PRPlot>& plots, unsigned int resolution) {
	// Assert that there's at least one plot
	assert(plots.size());

	vector<PRPlot> tmpPlots = plots;

	if (resolution) {
		for (size_t i = 0; i < tmpPlots.size(); ++i) {
			tmpPlots[i] = interpolatePlot(tmpPlots[i], resolution);
		}
	}

	// Assert that domains match
	for (size_t i = 0; i < tmpPlots.size(); ++i) {
		for (size_t j = i+1; j < tmpPlots.size(); ++j) {
			assert(tmpPlots[i].data.size() == tmpPlots[j].data.size());
			for (size_t x = 0; x < tmpPlots[i].data.size(); ++x) {
				assert(tmpPlots[i].data[x].first == tmpPlots[j].data[x].first);
			}
		}
	}

	PRPlot result;
	result.data = PRData(tmpPlots[0].data.size(), make_pair(0.f, -1.f));

	for (size_t x = 0; x < tmpPlots[0].data.size(); ++x) {
		result.data[x].first = tmpPlots[0].data[x].first;

		size_t numValidX = 0;
		float sum = 0.f;

		for (size_t plot = 0; plot < tmpPlots.size(); ++plot) {
			if (tmpPlots[plot].data[x].second < 0.f) continue;
			++numValidX;
			sum += tmpPlots[plot].data[x].second;
		}

		if (numValidX) {
			result.data[x].second = sum / numValidX;
		}
	}

	return result;
}

template <class TClassId, class TObjId, class TEmbedding, class TScalar>
std::string Evaluation<TClassId,TObjId,TEmbedding,TScalar>::serializePlot(const PRPlot& pr) {
	std::stringstream out;
	for (size_t j = 0; j < pr.data.size(); ++j) {
		if (pr.data[j].second < 0.f) continue;
		out << pr.data[j].first << " " << pr.data[j].second << "\n";
	}
	std::string result = out.str();
	return result;
}

/*
template <class TClassId, class TObjId, class TEmbedding, class TScalar>
typename Evaluation<TClassId,TObjId,TEmbedding,TScalar>::PRPlot Evaluation<TClassId,TObjId,TEmbedding,TScalar>::deserializePlot(const fs::path& sourceFile) {
	PRPlot resultPlot;
	if (!fs::exists(sourceFile)) {
		std::cout << "WARNING: Source file not found for plot deserialization" << std::endl;
		return resultPlot;
	}

	resultPlot.name = sourceFile.stem();
	ifstream in(sourceFile.file_string().c_str());
	string line;
	while (getline(in, line)) {
		vector<string> tokens = splitString(line);
		if (!tokens.size()) continue;
		resultPlot.data.push_back(make_pair(boost::lexical_cast<float>(tokens[0]), boost::lexical_cast<float>(tokens[1])));
	}
	in.close();

	return resultPlot;
}

template <class TClassId, class TObjId, class TEmbedding, class TScalar>
void Evaluation<UId, ObjectType>::plot(const PRPlot& pr, const fs::path& out, const string title) {
	vector<PRPlot> tmpPlotVec;
	tmpPlotVec.push_back(pr);
	plot(tmpPlotVec, out, title);
}

template <class TClassId, class TObjId, class TEmbedding, class TScalar>
void Evaluation<UId, ObjectType>::plot(const PRPlot& pr0, const PRPlot& pr1, const fs::path& out, const string title) {
	vector<PRPlot> tmpPlotVec;
	tmpPlotVec.push_back(pr0);
	tmpPlotVec.push_back(pr1);
	plot(tmpPlotVec, out, title);
}

template <class TClassId, class TObjId, class TEmbedding, class TScalar>
void Evaluation<UId, ObjectType>::plot(const vector<PRPlot>& pr, const fs::path& out, const string title) {
	fs::path tmpBasePath = fs::system_complete(out.parent_path());

	fs::path tmpCmdPath = tmpBasePath / "tmpGnuPlotCmd";

	ofstream tmpOut(tmpCmdPath.file_string().c_str());
	if (!tmpOut.good()) {
		cout << "Error while plotting to " << out << endl;
		return;
	}

	fs::path tmpDataDir = tmpBasePath / "tmpGnuPlotData";
	fs::remove_all(tmpDataDir);
	fs::create_directories(tmpDataDir);

	vector<fs::path> plotFiles;
	for (size_t p = 0; p < pr.size(); ++p) {
		fs::path plotFile = tmpDataDir / ("plot" + boost::lexical_cast<string>(p));
		serializePlot(pr[p], plotFile);
		plotFiles.push_back(plotFile);
	}

	tmpOut << "set term pdf enhanced font \"Arial\" size 4,3" << "\n";
	tmpOut << "set style data line" << "\n";
	tmpOut << "set xrange [0:1]" << "\n";
	tmpOut << "set yrange [0:1]" << "\n";
	tmpOut << "set xlabel \"Recall\"" << "\n";
	tmpOut << "set ylabel \"Precision\"" << "\n";
	tmpOut << "set grid xtics lw 2.5" << "\n";
	tmpOut << "set grid ytics lw 2.5" << "\n";
	tmpOut << "set title \"" << title << "\";" << "\n";
	tmpOut << "set border lw 1" << "\n";

	const int numColors = 10;
	const char* colors[] = {"red", "orange", "yellow", "green", "cyan", "blue", "violet", "brown", "black", "grey"};

	for (int i = 0; i < numColors; ++i) {
		tmpOut << "set style line " << (i+1) << " lt rgb \"" << colors[i] << "\" lw 3;\n";
	}

	tmpOut << "set output \"" << fs::system_complete(out).file_string() << "\";\n";
	tmpOut << "plot \\\n";

	for (size_t p = 0; p < pr.size(); ++p) {
		tmpOut << "\"" << plotFiles[p].file_string() << "\" ls " << (p%10+1) << " title \"" << pr[p].name << "\"" << (p == pr.size()-1 ? ";\n" : ",\\\n");
	}

	tmpOut.close();

	if (std::system(("gnuplot \"" + tmpCmdPath.file_string() + "\"").c_str())) {
		cout << "Gnuplot returned != 0 while plotting " << out << endl;
	}

	fs::remove(tmpCmdPath);
	fs::remove_all(tmpDataDir);
}
*/

template <class TClassId, class TObjId, class TEmbedding, class TScalar>
template <class TClassIter>
float Evaluation<TClassId,TObjId,TEmbedding,TScalar>::tier(TClassIter begin, TClassIter end, const Metric& metric, bool micro, unsigned int k) {
	int numClasses = std::distance(begin, end);
	float tier = 0.f;

	unsigned int numObjects = 0;
	for (TClassIter cIt = begin; cIt != end; ++cIt) {
		unsigned int n = cIt->getNumObjects();
		numObjects += n;
		unsigned int tierSize = k * n - 1;

		TClassId crtClass = cIt->getId();

		float classTier = 0.f;
		cIt->forEachObj([&](typename Class<TClassId,TObjId,TEmbedding>::Object& o) {
			unsigned int numRele = 0;
			TObjId crtObj = o.id;

			auto ranking = Ranking<TClassId,TObjId,TEmbedding,TScalar>::rankClasses(crtObj, metric, begin, end);

			for (size_t r = 0; r < tierSize; ++r) {
				if (ranking[r].first == crtClass) ++numRele;
			}
			classTier += static_cast<float>(numRele) / (n-1);
		});
		if (micro) classTier /= n;
		tier += classTier;
	}
	return tier / (micro ? numClasses : numObjects);
}

} // IR

#endif
