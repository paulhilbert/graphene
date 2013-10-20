#ifndef TREE_H_
#define TREE_H_

#include <include/common.h>

#include "Base.h"
#include "Labeled.h"
#include "Notify.h"

namespace GUI {
namespace Property {

class Tree : public Base, public Labeled, public Notify<void (std::string, bool)> {
	public:
		typedef std::shared_ptr<Tree> Ptr;
		typedef std::weak_ptr<Tree> WPtr;
		typedef std::vector<std::string> Path;

	public:
		Tree(std::string label);
		virtual ~Tree();

		void add(const std::string id, const std::vector<std::string>& path, bool checked = true);
		void remove(const std::string& id);
		bool has(const std::string id) const;
		virtual void removePath(const std::vector<std::string>& path) = 0;

	protected:
		std::string  getKey(const std::vector<std::string>& path);
		std::string  getId(const std::vector<std::string>& path);
		virtual void add(const std::vector<std::string>& path, bool checked) = 0;

	protected:
		std::map<std::string, std::string>  m_keyToIdMap;
		std::map<std::string, std::string>  m_idToKeyMap;
};

} // Property
} // GUI

#endif /* TREE_H_ */
