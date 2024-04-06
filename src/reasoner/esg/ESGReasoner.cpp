/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <memory>

#include "knowrob/reasoner/esg/ESGReasoner.h"
#include "knowrob/reasoner/ReasonerManager.h"

using namespace knowrob;

// make reasoner type accessible
KNOWROB_BUILTIN_REASONER("ESG", ESGReasoner)

ESGReasoner::ESGReasoner()
: PrologReasoner()
{
}

bool ESGReasoner::initializeDefaultPackages()
{
	return consult(std::filesystem::path("reasoner") / "esg" / "__init__.pl");
}