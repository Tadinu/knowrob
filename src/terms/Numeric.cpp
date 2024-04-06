/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/terms/Numeric.h"
#include "knowrob/integration/python/utils.h"

using namespace knowrob;

bool Numeric::isFloatingNumber() const {
	return xsdType() == XSDType::FLOAT || xsdType() == XSDType::DOUBLE;
}

std::shared_ptr<Numeric> Numeric::trueAtom() {
	static auto trueAtom = std::make_shared<Boolean>(true);
	return trueAtom;
}

std::shared_ptr<Numeric> Numeric::falseAtom() {
	static auto falseAtom = std::make_shared<Boolean>(false);
	return falseAtom;
}

namespace knowrob::py {
	// this struct is needed because Numeric has pure virtual methods
	struct NumericWrap : public Numeric {
		explicit NumericWrap(XSDType xsdType) : Numeric(xsdType) {}

		double asDouble() const override { return false; }

		float asFloat() const override { return false; }

		int asInteger() const override { return false; }

		long asLong() const override { return false;}

		short asShort() const override { return false; }

		unsigned int asUnsignedInteger() const override { return false;}

		unsigned long asUnsignedLong() const override { return false; }

		unsigned short asUnsignedShort() const override { return false;  }

		bool asBoolean() const override { return false; }

	private:

	};

	template<>
	void createType<Numeric>() {
	}
}
