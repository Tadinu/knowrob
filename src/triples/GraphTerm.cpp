/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/triples/GraphTerm.h"
#include "knowrob/integration/python/utils.h"
#include "knowrob/triples/GraphSequence.h"
#include "knowrob/triples/GraphUnion.h"
#include "knowrob/triples/GraphPattern.h"

using namespace knowrob;

namespace std {
	std::ostream &operator<<(std::ostream &os, const knowrob::GraphTerm &t) {
		t.write(os);
		return os;
	}
}

namespace knowrob::py {
	template<>
	void createType<GraphTerm>() {
	}
}
