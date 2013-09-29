template <class Scalar>
inline bool Zero::operator ==(Scalar op) {
	return std::abs(op) <= std::numeric_limits<Scalar>::epsilon();
}

template <class Scalar>
inline bool Zero::operator !=(Scalar op) {
	return !this->operator==(op);
}

template <class Scalar>
inline bool operator==(Scalar op, Zero z) {
	return z == op;
}

template <class Scalar>
inline bool operator!=(Scalar op, Zero z) {
	return z != op;
}
