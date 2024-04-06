/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/formulas/Implication.h"
#include "knowrob/integration/python/utils.h"

using namespace knowrob;

Implication::Implication(const FormulaPtr &antecedent, const FormulaPtr &consequent)
		: CompoundFormula(FormulaType::IMPLICATION, {antecedent, consequent}) {
}

bool Implication::isEqual(const Formula &other) const {
	const auto &x = static_cast<const Implication &>(other); // NOLINT
	return (*antecedent()) == (*x.antecedent()) &&
		   (*consequent()) == (*x.consequent());
}

namespace knowrob::py {
	template<>
	void createType<Implication>() {
	}
}
