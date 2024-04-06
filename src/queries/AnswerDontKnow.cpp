/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/queries/AnswerDontKnow.h"
#include "knowrob/integration/python/utils.h"

namespace knowrob {
	const std::shared_ptr<const AnswerDontKnow> &GenericDontKnow() {
		static const auto instance = std::make_shared<const AnswerDontKnow>();
		return instance;
	}
} // namespace knowrob

using namespace knowrob;

AnswerDontKnow::AnswerDontKnow()
		: Answer() {
}

AnswerDontKnow::AnswerDontKnow(const AnswerDontKnow &other)
		: Answer(other) {
}

std::string AnswerDontKnow::stringFormOfDontKnow() const {
	std::stringstream os;
	if (reasonerTerm_) {
		os << "[" << *reasonerTerm_ << "] ";
	}
	os << "don't know\n";
	return os.str();
}

std::string AnswerDontKnow::humanReadableFormOfDontKnow() const {
	static const std::string longMsg = "there was no evidence to conclude either yes or no";
	return longMsg;
}

namespace knowrob::py {
	template<>
	void createType<AnswerDontKnow>() {
	}
}
