#ifndef EXECUTOR_H_
#define EXECUTOR_H_

#include <vector>
using std::vector;

#include <functional>

#include <boost/any.hpp>
using boost::any;
using boost::any_cast;

#include <boost/mpl/at.hpp>
using boost::mpl::at;
#include <boost/mpl/int.hpp>
using boost::mpl::int_;

namespace Generic {

typedef vector<any> AnyParams;

template<class Arity, class ParamTypes, typename ResultType>
struct Executor;

// specializations for different arities
template<class ParamTypes, typename ResultType>
struct Executor< int_<0>, ParamTypes, ResultType> {
	template<class Fn>
	ResultType operator()(std::function<Fn> fn, const AnyParams& /*params*/) {
		return fn();
	}
};

template<class ParamTypes, typename ResultType>
struct Executor<int_<1>, ParamTypes, ResultType> {
	template<class Fn>
	ResultType operator()(std::function<Fn> fn, const AnyParams& params) {
		return fn( 
			any_cast< typename at<ParamTypes, int_<0> >::type>(params[0]) );
	}
};

template<class ParamTypes, typename ResultType>
struct Executor<int_<2>, ParamTypes, ResultType> {
	template<class Fn>
	ResultType operator()(std::function<Fn> fn, const AnyParams& params) {
		return fn( 
			any_cast< typename at<ParamTypes, int_<0> >::type>(params[0]),
			any_cast< typename at<ParamTypes, int_<1> >::type>(params[1]) );
	}
};

template<class ParamTypes, typename ResultType>
struct Executor< int_<3>, ParamTypes, ResultType > {
	template<class Fn>
	ResultType operator()(std::function<Fn> fn, const AnyParams& params) {
		return fn( 
			any_cast< typename at<ParamTypes, int_<0> >::type>(params[0]),
			any_cast< typename at<ParamTypes, int_<1> >::type>(params[1]),
			any_cast< typename at<ParamTypes, int_<2> >::type>(params[2]) );
	}
};

template<class ParamTypes, typename ResultType>
struct Executor< int_<4>, ParamTypes, ResultType > {
	template<class Fn>
	ResultType operator()(std::function<Fn> fn, const AnyParams& params) {
		return fn( 
			any_cast< typename at<ParamTypes, int_<0> >::type>(params[0]),
			any_cast< typename at<ParamTypes, int_<1> >::type>(params[1]),
			any_cast< typename at<ParamTypes, int_<2> >::type>(params[2]),
			any_cast< typename at<ParamTypes, int_<3> >::type>(params[3]) );
	}
};


} // Generic

#endif /* EXECUTOR_H_ */
