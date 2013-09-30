#ifndef RANKING_H
#define RANKING_H

#include <functional>

#include <vector>
#include <tuple>
using std::vector;
using std::tuple;
using std::pair;

#include "Classification.h"

namespace IR {

template <class TClassId, class TObjId, class TEmbedding, class TScalar>
class Ranking {
	public:
		typedef vector<pair<TObjId,TScalar>>            RankedObjs;
		typedef vector<pair<TClassId,TScalar>>          RankedClasses;
		typedef vector<tuple<TObjId,TClassId,TScalar>>  RankingList;

		typedef std::function<TScalar (TEmbedding,TEmbedding)> Metric;

	public:
		Ranking();

		template <class TClassIter>
		static RankingList rank(const TObjId& objId, const Metric& metric, TClassIter begin, TClassIter end);

		template <class TClassIter>
		static RankedObjs rankObjs(const TObjId& objId, const Metric& metric, TClassIter begin, TClassIter end);

		template <class TClassIter>
		static RankedClasses rankClasses(const TObjId& objId, const Metric& metric, TClassIter begin, TClassIter end);

};

template <class TClassId, class TObjId, class TEmbedding, class TScalar>
Ranking<TClassId,TObjId,TEmbedding,TScalar>::Ranking() {
}

template <class TClassId, class TObjId, class TEmbedding, class TScalar>
template <class TClassIter>
typename Ranking<TClassId,TObjId,TEmbedding,TScalar>::RankingList Ranking<TClassId,TObjId,TEmbedding,TScalar>::rank(const TObjId& objId, const Metric& metric, TClassIter begin, TClassIter end) {
	vector<tuple<TObjId,TClassId,TScalar>> ranking;

	TEmbedding* objEmbed = NULL;
	for (TClassIter it = begin; it != end; ++it) {
		if (objEmbed) break;
		it->forEachObj([&](typename Class<TClassId,TObjId,TEmbedding>::Object& o) {
			if (objEmbed) return;
			if (o.obj && o.id == objId) objEmbed = o.obj.get();
		});
	};
	if (!objEmbed) {
		Log::error("Could not retrieve object embedding for object id "+lexical_cast<std::string>(1168)+"!");
		return RankingList();
	}
	for (TClassIter it = begin; it != end; ++it) {
		it->forEachObj([&](typename Class<TClassId,TObjId,TEmbedding>::Object& o) {
			// check if obj exists and make sure to omit own embedding
			if (!o.obj || o.id == objId) return;
			ranking.push_back(std::make_tuple(o.id, it->getId(), metric(*objEmbed, *(o.obj))));
		});
	};
	std::sort(ranking.begin(), ranking.end(), [] (tuple<TObjId,TClassId,TScalar> obj0, tuple<TObjId,TClassId,TScalar> obj1) { return std::get<2>(obj0) < std::get<2>(obj1); });
	//RankingList result(ranking.size());
	//std::transform(ranking.begin(), ranking.end(), result.begin(), [&](tuple<TObjId,TClassId,TScalar> o) {return std::make_pair(std::get<0>(o),std::get<1>(o));});
	return ranking;
}

template <class TClassId, class TObjId, class TEmbedding, class TScalar>
template <class TClassIter>
typename Ranking<TClassId,TObjId,TEmbedding,TScalar>::RankedObjs Ranking<TClassId,TObjId,TEmbedding,TScalar>::rankObjs(const TObjId& objId, const Metric& metric, TClassIter begin, TClassIter end) {
	auto ranking = rank(objId, metric, begin, end);
	RankedObjs result(ranking.size());
	std::transform(ranking.begin(), ranking.end(), result.begin(), [](tuple<TObjId,TClassId,TScalar>& o) { return std::make_pair(std::get<0>(o), std::get<2>(o)); });
	return result;
}

template <class TClassId, class TObjId, class TEmbedding, class TScalar>
template <class TClassIter>
typename Ranking<TClassId,TObjId,TEmbedding,TScalar>::RankedClasses Ranking<TClassId,TObjId,TEmbedding,TScalar>::rankClasses(const TObjId& objId, const Metric& metric, TClassIter begin, TClassIter end) {
	auto ranking = rank(objId, metric, begin, end);
	RankedClasses result(ranking.size());
	std::transform(ranking.begin(), ranking.end(), result.begin(), [](tuple<TObjId,TClassId,TScalar>& o) { return std::make_pair(std::get<1>(o), std::get<2>(o)); });
	return result;
}


} // IR

#endif
