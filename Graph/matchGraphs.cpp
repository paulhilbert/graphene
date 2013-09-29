#include <mex.h>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include <fstream>
using std::ofstream;

#include "RCG.h"
#include "Algorithm/Match.h"
using namespace Graph;
using namespace Graph::Algorithm;

#include <StringUtils/StringManip.h>
using namespace StringUtils;

/*
 * matchGraphs.c
 *
 * Given two file paths this function
 * tries to load the graphs and match
 * the first into the second using subgraphIsomorphy.
 *
*/

mxLogical pathsValid(fs::path first, fs::path second) {
	if (!fs::exists(first) || !fs::is_regular_file(first) || first.extension() != ".graphml") {
		mexErrMsgIdAndTxt("GraphmlToolbox:matchGraphs:fstPathInvalid", "First (query) graph path invalid.");
		return false;
	}
	if (!fs::exists(second) || !fs::is_regular_file(second) || second.extension() != ".graphml") {
		mexErrMsgIdAndTxt("GraphmlToolbox:matchGraphs:sndPathInvalid", "Second (target) graph path invalid.");
		return false;
	}

	return true;
}

bool nodeCompare(RCG::NodePtr n0, RCG::NodePtr n1) {
	int t0, t1;
	try {
		t0 = any_cast<int>((*n0)["RoomType"]);
		t1 = any_cast<int>((*n1)["RoomType"]);
	} catch (...) { return false; }

	return t0 == t1;
}

bool edgeCompare(RCG::EdgePtr e0, RCG::EdgePtr e1) {
	try {
		return (any_cast< Vector<int, 3> >((*e1)["MergedPassageTypes"]) - any_cast< Vector<int, 3> >((*e0)["MergedPassageTypes"])).min() >= 0;
	} catch (...) {}

	return false;
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
	if(nrhs!=5) {
		mexErrMsgIdAndTxt("GraphmlToolbox:matchGraphs:nrhs", "Usage: matchGraphs(query, target, maxMatches, timeout, method)");
	}
	if(nlhs!=1) {
		mexErrMsgIdAndTxt("GraphmlToolbox:matchGraphs:nlhs", "One output required.");
	}
	if (!mxIsChar(prhs[0]) || !mxIsChar(prhs[1]) || !mxIsDouble(prhs[2]) || !mxIsDouble(prhs[3]) || !mxIsChar(prhs[4])) {
		mexErrMsgIdAndTxt("GraphmlToolbox:matchGraphs:trhs", "Usage: matchGraphs(query, target, maxMatches, timeout, method)");
	}

	// get file path strings and see if they seem valid
	unsigned int n = mxGetN(prhs[0]);
	unsigned int m = mxGetN(prhs[1]);
	char* cFQuery; char* cFTarget;
	cFQuery = (char*)mxMalloc((n+1)*sizeof(mxChar));
	cFTarget = (char*)mxMalloc((m+1)*sizeof(mxChar));
		
	if (mxGetString(prhs[0], cFQuery, n+1) || mxGetString(prhs[1], cFTarget, m+1)) {
		mexErrMsgIdAndTxt("GraphmlToolbox:matchGraphs:readrhs", "Could not read filenames.");
	}
	int maxMatches = static_cast<int>(mxGetScalar(prhs[2]));
	int timeout = static_cast<int>(mxGetScalar(prhs[3]));
	int method;
	n = mxGetN(prhs[4]);
	char* cMethod = (char*)mxMalloc((n+1)*sizeof(mxChar));
	if (mxGetString(prhs[4], cMethod, n+1)) {
		mexErrMsgIdAndTxt("GraphmlToolbox:matchGraphs:readrhs", "Could not read method parameter.");
	}
	string sMethod(cMethod);
	sMethod = StringManip::upper(sMethod);
	if (sMethod == "SUBISO") method = 0;
	else if (sMethod == "MONO") method = 1;
	else if (sMethod == "ISO") method = 2;
	else {
		mexErrMsgIdAndTxt("GraphmlToolbox:matchGraphs:readrhs", "Unknown method specifier. Should be one of {'ISO','SUBISO','MONO'}");
	}


	fs::path pQuery(cFQuery);
	fs::path pTarget(cFTarget);

	if (!pathsValid(pQuery, pTarget)) {
		mexErrMsgIdAndTxt("GraphmlToolbox:matchGraphs:invalidFilePaths", "Invalid file paths.");
	}

	RCG rcgQuery, rcgTarget;
	rcgQuery.deserializeGraphML(pQuery.file_string(), true);
	rcgTarget.deserializeGraphML(pTarget.file_string(), true);
	Match<RCG>::IsomorphyMap iMap;
	switch (method) {
		case 0: iMap = Match<RCG>::subgraphIsomorphism(rcgQuery, rcgTarget, maxMatches, milliseconds(timeout), &nodeCompare, &edgeCompare); break;
		case 1: iMap = Match<RCG>::monomorphism(rcgQuery, rcgTarget, maxMatches, milliseconds(timeout), &nodeCompare, &edgeCompare); break;
		default: iMap = Match<RCG>::isomorphism(rcgQuery, rcgTarget, maxMatches, milliseconds(timeout), &nodeCompare, &edgeCompare);
	}
	mexPrintf("match Count: %d", iMap.size());
	plhs[0] = mxCreateDoubleScalar(iMap.size());

	mxFree(cFQuery);
	mxFree(cFTarget);
	mxFree(cMethod);
}
