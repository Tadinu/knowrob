/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/formulas/Formula.h>
#include "knowrob/formulas/Top.h"
#include "knowrob/formulas/Bottom.h"
#include "knowrob/integration/python/utils.h"
#include "knowrob/formulas/CompoundFormula.h"
#include "knowrob/formulas/Negation.h"
#include "knowrob/formulas/Implication.h"
#include "knowrob/formulas/Disjunction.h"
#include "knowrob/formulas/ModalFormula.h"
#include "knowrob/formulas/FirstOrderLiteral.h"
#include "knowrob/formulas/PredicateIndicator.h"
#include "knowrob/formulas/Conjunction.h"

using namespace knowrob;

Formula::Formula(const FormulaType &type)
		: type_(type) {}

bool Formula::operator==(const Formula &other) const {
	// note: isEqual can safely perform static cast as type id's do match
	return typeid(*this) == typeid(other) && isEqual(other);
}

bool Formula::isAtomic() const {
	return type() == FormulaType::PREDICATE;
}

bool Formula::isBottom() const {
	return (this == Bottom::get().get());
}

bool Formula::isTop() const {
	return (this == Top::get().get());
}

bool FormulaLabel::operator==(const FormulaLabel &other) {
	return typeid(*this) == typeid(other) && isEqual(other);
}

namespace std {
	std::ostream &operator<<(std::ostream &os, const knowrob::Formula &phi) //NOLINT
	{
		phi.write(os);
		return os;
	}
}

namespace knowrob::py {
	template<>
	void createType<Formula>() {
	}
}
