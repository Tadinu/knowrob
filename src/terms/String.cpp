/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/terms/String.h"
#include "knowrob/integration/python/utils.h"

using namespace knowrob;

bool StringBase::isSameString(const StringBase &other) const {
	return stringForm() == other.stringForm();
}

void StringBase::write(std::ostream &os) const {
	os << '"' << stringForm() << '"';
}

namespace knowrob::py {
	template<>
	void createType<String>() {
	}
}
