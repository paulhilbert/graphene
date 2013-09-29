#ifndef COMMAND_H_
#define COMMAND_H_

#include <functional>

#include <boost/any.hpp>
using boost::any;

#include <boost/function_types/function_arity.hpp>
using boost::function_types::function_arity;

#include <boost/function_types/parameter_types.hpp>
using boost::function_types::parameter_types;

#include <boost/function_types/result_type.hpp>
using boost::function_types::result_type;

#include <boost/mpl/transform.hpp>

#include <boost/type_traits/is_void.hpp>
using boost::is_void;

#include <boost/type_traits/remove_reference.hpp>
using boost::remove_reference;

#include <boost/mpl/for_each.hpp>
using boost::mpl::for_each;

#include <boost/mpl/int.hpp>
using boost::mpl::int_;

#include "Executor.h"

namespace Generic {

class BaseCommand {
	public:
		BaseCommand() {}

		virtual void addParam(any param) {
			m_params.push_back(param);
		}

		virtual void setParams(const vector<any>& params) {
			m_params = params;
		}

		virtual void operator()() = 0;

	protected:
		vector<any> m_params;
};

template <class Sig>
class Command : public BaseCommand {
	struct remove_ref { template <class T> struct apply { typedef typename remove_reference<T>::type type; }; };

	// callback type
	typedef std::function<Sig> Receiver;
	// params
	typedef typename boost::mpl::transform< parameter_types<Sig>, remove_ref>::type ParamTypes;
	// return type
	typedef typename result_type<Sig>::type ResultType;
	enum { IsVoidResult = is_void<ResultType>::value };
	// executor
	typedef Generic::Executor< int_<function_arity<Sig>::value>, ParamTypes, ResultType > Executor;
	

	public:
		Command(Receiver receiver) : m_receiver(receiver) {}
		virtual ~Command() {}

		virtual void operator()() {
			Executor f;
			f(m_receiver, m_params);
			m_params.clear();
		}

	protected:
		Receiver    m_receiver;
};

} // Generic

#endif /* COMMAND_H_ */
