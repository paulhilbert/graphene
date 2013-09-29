#ifndef MATRIXTOOLS_H_
#define MATRIXTOOLS_H_

#include <utility>
using std::pair;

#include <Math/Vector.h>

namespace Math {
namespace Data {

struct MatrixTools {
	typedef  Vector<int, 2>  Pos;

	struct StrictUpperTri {
		static int linearFromPos(Pos p, int n);
		static Pos posFromLinear(int l, int n);
		static pair<Pos,Pos> getBlockBoundaries(int block, int numBlocks, int n);
		static int getNumElements(int n);
	};

};

int MatrixTools::StrictUpperTri::linearFromPos(Pos p, int n) {
	return (2*n*p[0] - p[0]*p[0] - 3*p[0] - 2) / 2 + p[1];
}

MatrixTools::Pos MatrixTools::StrictUpperTri::posFromLinear(int l, int n) {
	int nc = n - 1;

	Pos pos(0, l);
	while ( (pos[1] - nc) >= 0 ) {
		pos[1] -= nc;
		--nc;
		++pos[0];
	}
	pos[1] += pos[0] + 1;
	return pos;
}

pair<MatrixTools::Pos,MatrixTools::Pos> MatrixTools::StrictUpperTri::getBlockBoundaries(int block, int numBlocks, int n) {
	int numElements = getNumElements(n);
	int elementsPerBlock = static_cast<int>(ceil(static_cast<float>(numElements) / numBlocks));
	int lstart = block * elementsPerBlock;
	int lend = lstart + elementsPerBlock - 1;
	if (lend > (numElements - 1)) lend = numElements - 1;

	return pair<Pos,Pos>(posFromLinear(lstart, n), posFromLinear(lend, n));
}

int MatrixTools::StrictUpperTri::getNumElements(int n) {
	return n*(n+1)/2 - n;
}

} // Data
} // Math

#endif /* MATRIXTOOLS_H_ */
