template <class Sig>
Notify<Sig>::Notify() : m_onChange(nullptr) {
}

template <class Sig>
Notify<Sig>::~Notify() {
}

template <class Sig>
void Notify<Sig>::setCallback(Callback onChange) {
	m_onChange = onChange;
}

template <class Sig>
void Notify<Sig>::unsetCallback() {
	setCallback(nullptr);
}

template <class Sig>
void Notify<Sig>::notify() const {
	if (m_onChange) m_onChange();
}

template <class Sig>
template <class Arg0>
void Notify<Sig>::notify(Arg0 arg0) const {
	if (m_onChange) m_onChange(arg0);
}

template <class Sig>
template <class Arg0, class Arg1>
void Notify<Sig>::notify(Arg0 arg0, Arg1 arg1) const {
	if (m_onChange) m_onChange(arg0, arg1);
}
