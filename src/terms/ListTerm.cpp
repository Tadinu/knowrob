/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */


#include "knowrob/terms/ListTerm.h"
#include "knowrob/terms/String.h"
#include "knowrob/integration/python/utils.h"

using namespace knowrob;

ListTerm::ListTerm(const std::vector<TermPtr> &elements)
		: Function(listFunctor(), elements) {
}

const AtomPtr &ListTerm::listFunctor() {
	static const AtomPtr fun = Atom::Tabled("[]");
	return fun;
}

std::shared_ptr<ListTerm> ListTerm::nil() {
	static std::shared_ptr<ListTerm> x(new ListTerm(
			std::vector<std::shared_ptr<Term>>(0)));
	return x;
}

bool ListTerm::isNIL() const {
	return arguments_.empty();
}

void ListTerm::write(std::ostream &os) const {
	os << '[';
	for (uint32_t i = 0; i < arguments_.size(); i++) {
		os << *arguments_[i];
		if (i + 1 < arguments_.size()) {
			os << ',' << ' ';
		}
	}
	os << ']';
}

namespace knowrob::py {
	template<>
	void createType<ListTerm>() {
	}
}