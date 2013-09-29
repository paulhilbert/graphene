#ifndef BASEEXCEPTION_H_
#define BASEEXCEPTION_H_

namespace Testing {

class BaseException {
	public:
		BaseException() {
		}
		virtual ~BaseException() {
		}

		virtual void print() const noexcept = 0;
};

} // Testing

#endif /* BASEEXCEPTION_H_ */
