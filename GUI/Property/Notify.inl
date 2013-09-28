template <class... Params>
Notify<Params...>::Notify() : m_onChange(nullptr) {
}

template <class... Params>
Notify<Params...>::~Notify() {
}

template <class... Params>
void Notify<Params...>::setCallback(Callback onChange) {
	m_onChange = onChange;
}

template <class... Params>
void Notify<Params...>::unsetCallback() {
	setCallback(nullptr);
}

template <class... Params>
void Notify<Params...>::notify(Params... params) const {
	if (m_onChange) m_onChange(params...);
}
