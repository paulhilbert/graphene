template <typename Scalar, int DIM>
Vector<Scalar,DIM>::Vector() {
}

template <typename Scalar, int DIM>
Vector<Scalar,DIM>::Vector(const Scalar& v0, const Scalar& v1) {
	assert(DIM==2);
	m_values[0] = v0; m_values[1] = v1; 
}

template <typename Scalar, int DIM>
Vector<Scalar,DIM>::Vector(const Scalar& v0, const Scalar& v1, const Scalar& v2) {
	assert(DIM==3);
	m_values[0]=v0; m_values[1]=v1; m_values[2]=v2; 
}

template <typename Scalar, int DIM>
Vector<Scalar,DIM>::Vector(const Scalar& v0, const Scalar& v1, const Scalar& v2, const Scalar& v3) {
	assert(DIM==4); 
	m_values[0]=v0; m_values[1]=v1; m_values[2]=v2; m_values[3]=v3; 
}

template <typename Scalar, int DIM>
Vector<Scalar,DIM>::Vector(const Scalar& v) {
	for (int i = 0; i < DIM; ++i) {
		m_values[i] = v;
	}
}

template <typename Scalar, int DIM>
Vector<Scalar,DIM>::Vector(const Scalar values[DIM]) {
	memcpy(m_values, values, DIM*sizeof(Scalar)); 
}

template <typename Scalar, int DIM>
Vector<Scalar,DIM>::Vector(const Vector<Scalar,DIM>& other) { 
	operator=(other); 
}

template <typename Scalar, int DIM>
template <typename OtherScalarType>
Vector<Scalar,DIM>::Vector(const Vector<OtherScalarType,DIM>& other) { 
	operator=(other); 
}

template <typename Scalar, int DIM>
inline Vector<Scalar,DIM>& Vector<Scalar,DIM>::operator=(const Vector<Scalar,DIM>& other) {
	for (int i = 0; i < DIM; ++i) {
		this->m_values[i] = other.m_values[i];
	}
	return *this; 
}

template <typename Scalar, int DIM>
template <typename OtherScalarType>
inline Vector<Scalar,DIM>& Vector<Scalar,DIM>::operator=(const Vector<OtherScalarType,DIM>& other) {
	for (int i = 0; i < DIM; ++i) {
		this->m_values[i] = (Scalar)other.m_values[i];
	}
	return *this; 
}

template <typename Scalar, int DIM>
inline Vector<Scalar,DIM>::operator Scalar*() {
	return m_values;
}

template <typename Scalar, int DIM>
inline Vector<Scalar,DIM>::operator const Scalar*() const {
	return m_values;
}

template <typename Scalar, int DIM>
inline Scalar& Vector<Scalar,DIM>::operator[](int i) {
	assert(i>=0 && i<DIM); return m_values[i]; 
}

template <typename Scalar, int DIM>
inline const Scalar& Vector<Scalar,DIM>::operator[](int i) const {
	assert(i>=0 && i<DIM); return m_values[i]; 
}

template <typename Scalar, int DIM>
inline bool Vector<Scalar,DIM>::operator==(const Vector<Scalar,DIM>& other) const {
	for (int i = 0; i < DIM; ++i) {
		if(m_values[i]!=other.m_values[i]) return false;
	}
	return true; 
}

template <typename Scalar, int DIM>
inline bool Vector<Scalar,DIM>::operator!=(const Vector<Scalar,DIM>& other) const {
	return !(*this == other);
}

template <typename Scalar, int DIM>
inline Vector<Scalar,DIM> Vector<Scalar,DIM>::operator-(void) const {
	Vector<Scalar,DIM> v;
	for (int i = 0; i < DIM; ++i) {
		v.m_values[i] = -m_values[i];
	}
	return v; 
}

template <typename Scalar, int DIM>
inline Vector<Scalar,DIM>& Vector<Scalar,DIM>::operator*=(const Scalar& s) {
	for (int i = 0; i < DIM; ++i) {
		m_values[i] *= s; 
	}
	return *this; 
}

template <typename Scalar, int DIM>
inline Vector<Scalar,DIM>& Vector<Scalar,DIM>::operator/=(const Scalar& s) {
	Scalar inv = Scalar(1)/s;
	for (int i = 0; i < DIM; ++i) {
		m_values[i] *= inv; 
	}
	return *this; 
}

template <typename Scalar, int DIM>
inline Vector<Scalar,DIM> Vector<Scalar,DIM>::operator*(const Scalar& s) const {
	return Vector<Scalar,DIM>(*this) *= s;
}

template <typename Scalar, int DIM>
inline Vector<Scalar,DIM> Vector<Scalar,DIM>::operator/(const Scalar& s) const {
	return Vector<Scalar,DIM>(*this) /= s;
}

template <typename Scalar, int DIM>
inline Vector<Scalar,DIM>& Vector<Scalar,DIM>::operator*=(const Vector<Scalar,DIM>& other) {
	for (int i = 0; i < DIM; ++i) {
		m_values[i] *= other[i]; 
	}
	return *this; 
}

template <typename Scalar, int DIM>
inline Vector<Scalar,DIM>& Vector<Scalar,DIM>::operator/=(const Vector<Scalar,DIM>& other) {
	for (int i = 0; i < DIM; ++i) {
		m_values[i] /= other[i]; 
	}
	return *this; 
}

template <typename Scalar, int DIM>
inline Vector<Scalar,DIM>& Vector<Scalar,DIM>::operator-=(const Vector<Scalar,DIM>& other) {
	for (int i = 0; i < DIM; ++i) {
		m_values[i] -= other[i]; 
	}
	return *this; 
}

template <typename Scalar, int DIM>
inline Vector<Scalar,DIM>& Vector<Scalar,DIM>::operator+=(const Vector<Scalar,DIM>& other) {
	for (int i = 0; i < DIM; ++i) {
		m_values[i] += other[i]; 
	}
	return *this; 
}

template <typename Scalar, int DIM>
inline Vector<Scalar,DIM> Vector<Scalar,DIM>::operator*(const Vector<Scalar,DIM>& _v) const {
	return Vector<Scalar,DIM>(*this) *= _v;
}

template <typename Scalar, int DIM>
inline Vector<Scalar,DIM> Vector<Scalar,DIM>::operator/(const Vector<Scalar,DIM>& _v) const {
	return Vector<Scalar,DIM>(*this) /= _v;
}

template <typename Scalar, int DIM>
inline Vector<Scalar,DIM> Vector<Scalar,DIM>::operator+(const Vector<Scalar,DIM>& _v) const {
	return Vector<Scalar,DIM>(*this) += _v;
}

template <typename Scalar, int DIM>
inline Vector<Scalar,DIM> Vector<Scalar,DIM>::operator-(const Vector<Scalar,DIM>& _v) const {
	return Vector<Scalar,DIM>(*this) -= _v;
}

template <typename Scalar, int DIM>
inline Vector<Scalar,3> Vector<Scalar,DIM>::operator%(const Vector<Scalar,3>& other) const {
	return Vector<Scalar,3>( m_values[1]*other.m_values[2]-m_values[2]*other.m_values[1] ,
	                         m_values[2]*other.m_values[0]-m_values[0]*other.m_values[2] ,
	                         m_values[0]*other.m_values[1]-m_values[1]*other.m_values[0] );
}

template <typename Scalar, int DIM>
inline Vector<Scalar,3> Vector<Scalar,DIM>::operator%(const Vector<Scalar,2>& other) const {
	return Vector<Scalar,3>(0, 0,	m_values[0]*other.m_values[1]-m_values[1]*other.m_values[0]);
}

template <typename Scalar, int DIM>
inline Scalar Vector<Scalar,DIM>::operator|(const Vector<Scalar,DIM>& other) const {
	Scalar p(0);
	for (int i = 0; i < DIM; ++i) {
		p += m_values[i] * other.m_values[i]; 
	}
	return p; 
}

template <typename Scalar, int DIM>
inline Scalar Vector<Scalar,DIM>::norm() const {
	return (Scalar)sqrt(sum_sqr());
}

template <typename Scalar, int DIM>
inline Scalar Vector<Scalar,DIM>::sum_sqr() const {
	Scalar s(0);
	for (int i = 0; i < DIM; ++i) {
		s += m_values[i] * m_values[i]; 
	}
	return s;
}

template <typename Scalar, int DIM>
inline Vector<Scalar,DIM>& Vector<Scalar,DIM>::normalize() {
	operator*=(((Scalar)1.0)/norm());
	return *this;
}

template <typename Scalar, int DIM>
inline Scalar Vector<Scalar,DIM>::max() const {
	Scalar m(m_values[0]);
	for(int i=1; i<DIM; ++i) if(m_values[i]>m) m=m_values[i];
	return m; 
}

template <typename Scalar, int DIM>
inline Scalar Vector<Scalar,DIM>::min() const {
	Scalar m(m_values[0]); 
	for(int i=1; i<DIM; ++i) if(m_values[i]<m) m=m_values[i];
	return m; 
}

template <typename Scalar, int DIM>
inline int Vector<Scalar,DIM>::argMax() const {
	Scalar m(m_values[0]);
	int result = 0;
	for(int i=1; i<DIM; ++i) {
		if(m_values[i]>m) { m=m_values[i]; result=i; }
	}
	return result; 
}

template <typename Scalar, int DIM>
inline int Vector<Scalar,DIM>::argMin() const {
	Scalar m(m_values[0]);
	int result = 0;
	for(int i=1; i<DIM; ++i) if(m_values[i]<m) { m=m_values[i]; result=i; }
	return result; 
}

template <typename Scalar, int DIM>
inline Scalar Vector<Scalar,DIM>::mean() const {
	Scalar m(m_values[0]); 
	for(int i=1; i<DIM; ++i) m+=m_values[i];
	return m/Scalar(DIM); 
}

template <typename Scalar, int DIM>
Vector<Scalar,DIM>& Vector<Scalar,DIM>::abs() {
	for (int i = 0; i < DIM; ++i) {
		m_values[i] = fabs(m_values[i]);
	}
	return *this;	
}

template<typename Scalar, int DIM>
inline Vector<Scalar,DIM> operator*(Scalar s, const Vector<Scalar,DIM>& v) {
	return Vector<Scalar,DIM>(v) *= s;
}

template<typename Scalar, int DIM>
inline Vector<Scalar,DIM> Normalize(const Vector<Scalar,DIM>& v) {
	Vector<Scalar, DIM> vec( v );
	return vec.normalize();
}

template<typename Scalar, int DIM>
inline Vector<Scalar,DIM> Abs(const Vector<Scalar,DIM>& v) {
	Vector<Scalar,DIM> vec(v);
	return vec.abs();
}

template <typename Scalar,int DIM>
inline std::istream& operator>>(std::istream& is, Vector<Scalar,DIM>& vec) {
	for (int i = 0; i < DIM; ++i) {
		is >> std::ws >> vec[i];
	}
	return is;
}

template <typename Scalar,int DIM>
inline std::ostream& operator<<(std::ostream& os, const Vector<Scalar,DIM>& vec) {
	for(int i=0; i<DIM-1; ++i) os << vec[i] << " ";
		os << vec[DIM-1];
	return os;
}
