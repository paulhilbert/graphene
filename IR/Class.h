#ifndef CLASS_H_
#define CLASS_H_

#include <iostream>
using std::string;
#include <vector>
#include <list>
#include <map>
using std::vector;
using std::list;
using std::pair;
using std::map;

#include <algorithm>
using std::transform;
using std::for_each;

#include <functional>
#include <memory>

#include <boost/optional.hpp>
#include <boost/none.hpp>
using boost::optional;
using boost::none;


namespace IR {

typedef enum { TESTING, TRAINING } ClassType;


template <class TClassId, class TObjId, class TObject>
class Class {
	public:
		struct Object {
			TObjId id;
			std::shared_ptr<TObject> obj;
			ClassType type;
		};
		typedef std::shared_ptr<Object> ObjPtr;
		typedef map<TObjId, ObjPtr> ObjMap;

	public:
		Class(TClassId id);
		Class<TClassId,TObjId,TObject> copy(optional<ClassType> restriction = none);

		TClassId getId() const;
		unsigned int getNumObjects() const;

		ObjMap& getObjects();
		const ObjMap& getObjects() const;
		vector<TObjId> getIdVector() const;
		list<TObjId> getIdList() const;
		Object& operator[](TObjId k);
		const Object& operator[](TObjId k) const;

		void addObject(TObjId id, ClassType type, std::shared_ptr<TObject> obj = std::shared_ptr<TObject>());

		bool contains(const TObjId objId) const;

		void forEachObj(std::function<void (Object&)> func);

		void loadObjects(std::function<std::shared_ptr<TObject> (TObjId)> loadFunc, bool overwrite = true);
		int removeUnloaded();


	protected:
		TClassId  m_id;
		ObjMap    m_objects;
};

template <class TClassId, class TObjId, class TObject>
Class<TClassId,TObjId,TObject>::Class(TClassId id) : m_id(id) {
}

template <class TClassId, class TObjId, class TObject>
inline Class<TClassId,TObjId,TObject> Class<TClassId,TObjId,TObject>::copy(optional<ClassType> restriction) {
	Class<TClassId,TObjId,TObject> c(m_id);
	for_each(m_objects.begin(), m_objects.end(), [&](typename ObjMap::value_type o) {
		if (!restriction || o.second->type == restriction.get())
			c.m_objects[o.first] = o.second;
	});
	return c;
}

template <class TClassId, class TObjId, class TObject>
inline TClassId Class<TClassId,TObjId,TObject>::getId() const {
	return m_id;
}

template <class TClassId, class TObjId, class TObject>
inline unsigned int Class<TClassId,TObjId,TObject>::getNumObjects() const {
	return m_objects.size();
}

template <class TClassId, class TObjId, class TObject>
inline typename Class<TClassId,TObjId,TObject>::ObjMap& Class<TClassId,TObjId,TObject>::getObjects() {
	return m_objects;
}

template <class TClassId, class TObjId, class TObject>
inline const typename Class<TClassId,TObjId,TObject>::ObjMap& Class<TClassId,TObjId,TObject>::getObjects() const {
	return m_objects;
}

template <class TClassId, class TObjId, class TObject>
inline vector<TObjId> Class<TClassId,TObjId,TObject>::getIdVector() const {
	vector<TObjId> result;
	std::for_each(m_objects.begin(), m_objects.end(), [&result](const typename ObjMap::value_type& o) { result.push_back(o.second->id); });
	return result;
}

template <class TClassId, class TObjId, class TObject>
inline list<TObjId> Class<TClassId,TObjId,TObject>::getIdList() const {
	list<TObjId> result;
	std::for_each(m_objects.begin(), m_objects.end(), [&result](const typename ObjMap::value_type& o) { result.push_back(o.second->id); });
	return result;
}

template <class TClassId, class TObjId, class TObject>
inline typename Class<TClassId,TObjId,TObject>::Object& Class<TClassId,TObjId,TObject>::operator[](TObjId k) {
	return *(m_objects[k]);
}

template <class TClassId, class TObjId, class TObject>
inline const typename Class<TClassId,TObjId,TObject>::Object& Class<TClassId,TObjId,TObject>::operator[](TObjId k) const {
	return *(m_objects[k]);
}

template <class TClassId, class TObjId, class TObject>
inline void Class<TClassId,TObjId,TObject>::addObject(TObjId id, ClassType type, std::shared_ptr<TObject> obj) {
	ObjPtr o(new Object());
	o->id   = id;
	o->type = type;
	o->obj  = obj;
	m_objects[id] = o;
}

template <class TClassId, class TObjId, class TObject>
inline bool Class<TClassId,TObjId,TObject>::contains(const TObjId objId) const {
	return m_objects.find(objId) != m_objects.end();
}

template <class TClassId, class TObjId, class TObject>
inline void Class<TClassId,TObjId,TObject>::forEachObj(std::function<void (Object&)> func) {
	for_each(m_objects.begin(), m_objects.end(), [&](typename ObjMap::value_type& o) { func(std::ref(*(o.second))); });
}

template <class TClassId, class TObjId, class TObject>
inline void Class<TClassId,TObjId,TObject>::loadObjects(std::function<std::shared_ptr<TObject> (TObjId)> loadFunc, bool overwrite) {
//	lp.process<ObjMap>(m_objects.begin(), m_objects.end(), [&](typename ObjMap::value_type& o) { if (overwrite || !o.second->obj) o.second->obj = loadFunc(o.second->id); });
	forEachObj([&](Object& obj) { 
		if (overwrite || !obj.obj) {
			obj.obj = loadFunc(obj.id);
		}
	});
}

template <class TClassId, class TObjId, class TObject>
inline int Class<TClassId,TObjId,TObject>::removeUnloaded() {
	int numErased = 0;
	for (typename ObjMap::iterator it = m_objects.begin(); it != m_objects.end();) {
		if (!it->second->obj) {
			m_objects.erase(it++);
			++numErased;
		} else {
			++it;
		}
	}
	return numErased;
}

} // IR


#endif /* CLASS_H_ */
