/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "sstream"
#include "knowrob/terms/Term.h"
#include "knowrob/terms/Atomic.h"
#include "knowrob/terms/Function.h"
#include "knowrob/terms/Variable.h"
#include "knowrob/integration/python/utils.h"
#include "knowrob/knowrob.h"

using namespace knowrob;

const std::set<std::string_view> Term::noVariables_ = {};

bool Term::isAtom() const {
	return termType() == TermType::ATOMIC && ((Atomic *) this)->atomicType() == AtomicType::ATOM;
}

bool Term::isVariable() const {
	return termType() == TermType::VARIABLE;
}

bool Term::isFunction() const {
	return termType() == TermType::FUNCTION;
}

bool Term::isNumeric() const {
	return termType() == TermType::ATOMIC && ((Atomic *) this)->atomicType() == AtomicType::NUMERIC;
}

bool Term::isString() const {
	return termType() == TermType::ATOMIC && ((Atomic *) this)->atomicType() == AtomicType::STRING;
}

size_t Term::hash() const {
	size_t val = 0;
	hashCombine(val, uint8_t(termType()));
	switch (termType()) {
		case TermType::ATOMIC:
			hashCombine(val, ((Atomic *) this)->hashOfAtomic());
			break;
		case TermType::FUNCTION:
			hashCombine(val, ((Function *) this)->hashOfFunction());
			break;
		case TermType::VARIABLE:
			hashCombine(val, std::hash<std::string_view>{}(((Variable *) this)->name()));
			break;
	}
	return val;
}

bool Term::operator==(const Term &other) const {
	if (this == &other) return true;
	if (termType() != other.termType()) return false;
	switch (termType()) {
		case TermType::ATOMIC:
			return ((Atomic *) this)->isSameAtomic(*((Atomic *) &other));
		case TermType::VARIABLE:
			return ((Variable *) this)->isSameVariable(*((Variable *) &other));
		case TermType::FUNCTION:
			return ((Function *) this)->isSameFunction(*((Function *) &other));
	}
	return false;
}

namespace std {
	ostream &operator<<(ostream &os, const knowrob::Term &t) { //NOLINT
		knowrob::TermWriter(t, os);
		return os;
	}
}

namespace knowrob::py {
	// this struct is needed because Term has pure virtual methods
	struct TermWrap : public Term {
		explicit TermWrap(TermType termType) : Term(termType) {}

		const std::set<std::string_view> &
		variables() const override { }

	private:

	};

	template<>
	void createType<Term>() {
	}
}
