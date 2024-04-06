/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/reasoner/GoalDrivenReasoner.h"
#include "knowrob/integration/python/utils.h"

using namespace knowrob;

namespace knowrob::py {
	// this struct is needed because Reasoner has pure virtual methods
	struct GoalDrivenReasonerWrap : public GoalDrivenReasoner {
		explicit GoalDrivenReasonerWrap() : GoalDrivenReasoner() {}

		void setDataBackend(const StoragePtr &backend) override {
		}

		bool initializeReasoner(const PropertyTree &config) override {
		}

		TokenBufferPtr submitQuery(const FramedTriplePatternPtr &literal, const QueryContextPtr &ctx) override {
		}

	private:

	};

	template<>
	void createType<GoalDrivenReasoner>() {
	}
}
