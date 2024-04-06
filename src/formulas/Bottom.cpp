/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */


#include "knowrob/formulas/Bottom.h"
#include "knowrob/integration/python/utils.h"

using namespace knowrob;

const std::shared_ptr<Bottom> &Bottom::get() {
	static std::shared_ptr<Bottom> singleton(new Bottom);
	return singleton;
}

Bottom::Bottom()
		: Predicate("false", std::vector<TermPtr>()) {
}

void Bottom::write(std::ostream &os) const {
	os << "\u22A5";
}

bool Bottom::isEqual(const Formula &other) const {
	return true; // isEqual is only called if other also has type "Bottom"
}

namespace knowrob::py {
	template<>
	void createType<Bottom>() {
	}
}
