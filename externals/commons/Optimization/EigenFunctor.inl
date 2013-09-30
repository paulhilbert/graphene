template<typename _Scalar, int NX, int NY>
EigenFunctor<_Scalar,NX,NY>::EigenFunctor(Func func, Deriv deriv) : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime), m_func(func), m_deriv(deriv) {
}

template<typename _Scalar, int NX, int NY>
inline int EigenFunctor<_Scalar,NX,NY>::inputs() const {
	return m_inputs;
}

template<typename _Scalar, int NX, int NY>
inline int EigenFunctor<_Scalar,NX,NY>::values() const {
	return m_values;
}

template<typename _Scalar, int NX, int NY>
inline int EigenFunctor<_Scalar,NX,NY>::operator()(InputType& x, ValueType& y) const {
	m_func(x,y);
	return 0;
}

template<typename _Scalar, int NX, int NY>
inline int EigenFunctor<_Scalar,NX,NY>::df(InputType& x, JacobianType& y) const {
	m_deriv(x,y);
	return 0;
}
