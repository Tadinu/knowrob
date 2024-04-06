/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/reasoner/ReasonerEvent.h"
#include "knowrob/integration/python/utils.h"

using namespace knowrob;
using namespace knowrob::reasoner;

template<typename T>
void createTriples(std::vector<FramedTriplePtr> &triples) {
	for (auto &triple: triples) {
		triple.ptr = new T();
		triple.owned = true;
	}
}

TripleEvent::TripleEvent(Type eventType, uint32_t tripleCount, bool copy)
		: Event(eventType), triples_(tripleCount) {
	if (copy) {
		createTriples<FramedTripleCopy>(triples_);
	} else {
		createTriples<FramedTripleView>(triples_);
	}
}

namespace knowrob::py {
	template<>
	void createType<reasoner::Event>() {
	}
}
