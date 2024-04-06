/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */



#include "knowrob/Logger.h"
#include "knowrob/reasoner/ReasonerManager.h"
#include "knowrob/reasoner/mongolog/MongologReasoner.h"
#include "knowrob/queries/QueryError.h"
#include "knowrob/reasoner/ReasonerError.h"
#include "knowrob/terms/Numeric.h"
#include "knowrob/knowrob.h"

using namespace knowrob;

// make reasoner type accessible
KNOWROB_BUILTIN_REASONER("Mongolog", MongologReasoner)

// foreign predicates
foreign_t pl_is_readonly2(term_t, term_t);

foreign_t pl_db_name3(term_t, term_t, term_t);

foreign_t pl_uri3(term_t, term_t, term_t);

foreign_t pl_assert_triple_cpp9(term_t, term_t, term_t, term_t, term_t, term_t, term_t, term_t, term_t);

MongologReasoner::MongologReasoner()
		: PrologReasoner() {}

MongologReasoner::~MongologReasoner()
= default;

bool MongologReasoner::initializeDefaultPackages() {
	static bool initialized = false;

	if (!initialized) {
		initialized = true;
		// load mongolog code once globally into the Prolog engine
		consult(std::filesystem::path("reasoner") / "mongolog" / "__init__.pl",
				"user", false);

		PL_register_foreign("mng_is_readonly_cpp",
							2, (pl_function_t) pl_is_readonly2, 0);
		PL_register_foreign("mng_db_name_cpp",
							3, (pl_function_t) pl_db_name3, 0);
		PL_register_foreign("mng_uri_cpp",
							3, (pl_function_t) pl_uri3, 0);

		PL_register_foreign("mng_assert_triple_cpp",
							9, (pl_function_t) pl_assert_triple_cpp9, 0);
	}

	return true;
}

bool MongologReasoner::initializeReasoner(const PropertyTree &reasonerConfiguration) {
	if (!PrologReasoner::initializeReasoner(reasonerConfiguration)) return false;

	if (!knowledgeGraph_) {
		knowledgeGraph_ = std::make_shared<MongoKnowledgeGraph>();
		knowledgeGraph_->setVocabulary(std::make_shared<Vocabulary>());
		knowledgeGraph_->initializeBackend(
				MongoKnowledgeGraph::DB_URI_DEFAULT,
				MongoKnowledgeGraph::DB_NAME_KNOWROB,
				MongoKnowledgeGraph::COLL_NAME_TRIPLES
		);
		reasonerManager().backendManager()->addPlugin("mongo", knowledgeGraph_);
		KB_WARN("Falling back to default configuration for MongoDB!");
	}

	return true;
}

void MongologReasoner::setDataBackend(const StoragePtr &backend) {
	knowledgeGraph_ = std::dynamic_pointer_cast<MongoKnowledgeGraph>(backend);
	if (!knowledgeGraph_) {
		throw ReasonerError(
				"Unexpected data knowledgeGraph used for Mongolog reasoner. MongoKnowledgeGraph must be used.");
	}
}

std::string_view MongologReasoner::callFunctor() {
	static const auto call_f = "mongolog_call";
	return call_f;
}

static inline std::shared_ptr<MongologReasoner> getMongologReasoner(term_t t_reasonerManager,
																	term_t t_reasonerModule) {
	auto definedReasoner = PrologReasoner::getDefinedReasoner(t_reasonerManager, t_reasonerModule);
	if (!definedReasoner) {
		KB_ERROR("unable to find reasoner with id '{}' (manager id: {}).",
				 *PrologTerm::toKnowRobTerm(t_reasonerModule),
				 *PrologTerm::toKnowRobTerm(t_reasonerManager));
		return {};
	}
	auto reasoner = definedReasoner->value();
	auto mongolog = std::dynamic_pointer_cast<MongologReasoner>(reasoner);
	if (!mongolog) {
		KB_ERROR("reasoner with id '{}' (manager id: {}) is not a mongolog reasoner.",
				 *PrologTerm::toKnowRobTerm(t_reasonerModule),
				 *PrologTerm::toKnowRobTerm(t_reasonerManager));
	}
	return mongolog;
}


foreign_t pl_is_readonly2(term_t t_reasonerManager, term_t t_reasonerModule) {
	auto mongolog = getMongologReasoner(t_reasonerManager, t_reasonerModule);
	char *graph;
	if (mongolog) {
		return mongolog->knowledgeGraph()->isReadOnly();
	}
	return false;
}

foreign_t pl_db_name3(term_t t_reasonerManager, term_t t_reasonerModule, term_t t_dbName) {
	auto mongolog = getMongologReasoner(t_reasonerManager, t_reasonerModule);
	if (mongolog) {
		auto &dbName = mongolog->knowledgeGraph()->dbName();
		return PL_unify_atom_chars(t_dbName, dbName.c_str());
	}
	return false;
}

foreign_t pl_uri3(term_t t_reasonerManager, term_t t_reasonerModule, term_t t_uri) {
	auto mongolog = getMongologReasoner(t_reasonerManager, t_reasonerModule);
	if (mongolog) {
		auto &uri = mongolog->knowledgeGraph()->dbURI();
		return PL_unify_atom_chars(t_uri, uri.c_str());
	}
	return false;
}

foreign_t pl_assert_triple_cpp9(term_t t_reasonerManager,
								term_t t_reasonerModule,
								term_t t_subjectTerm,
								term_t t_propertyTerm,
								term_t t_objectTerm,
								term_t t_graphTerm,
								term_t t_beginTerm,
								term_t t_endTerm,
								term_t t_confidenceTerm) {
	auto mongolog = getMongologReasoner(t_reasonerManager, t_reasonerModule);
	if (mongolog) {
		FramedTripleView tripleData;

		// "s" field
		auto subjectTerm = PrologTerm::toKnowRobTerm(t_subjectTerm);
		if (subjectTerm->termType() != TermType::ATOMIC) throw QueryError("invalid subject term {}", *subjectTerm);
		tripleData.setSubject(((Atomic *) subjectTerm.get())->stringForm());

		// "p" field
		auto propertyTerm = PrologTerm::toKnowRobTerm(t_propertyTerm);
		if (propertyTerm->termType() != TermType::ATOMIC) throw QueryError("invalid property term {}", *propertyTerm);
		tripleData.setPredicate(((Atomic *) propertyTerm.get())->stringForm());

		// "o" field
		auto objectTerm = PrologTerm::toKnowRobTerm(t_objectTerm);
		if(objectTerm->termType() == TermType::ATOMIC) {
			auto atomic = std::static_pointer_cast<Atomic>(objectTerm);
			if (atomic->isNumeric()) {
				tripleData.setXSDValue(atomic->stringForm(),
					std::static_pointer_cast<Numeric>(atomic)->xsdType());
			} else if (atomic->isIRI()) {
				tripleData.setObjectIRI(atomic->stringForm());
			} else if (atomic->isBlank()) {
				tripleData.setObjectBlank(atomic->stringForm());
			} else {
				tripleData.setStringValue(atomic->stringForm());
			}
		} else {
			throw QueryError("object term {} of triple has an invalid type", *objectTerm);
		}

		// "g" field
		TermPtr graphTerm;
		if (!PL_is_variable(t_graphTerm)) {
			graphTerm = PrologTerm::toKnowRobTerm(t_graphTerm);
			if (graphTerm->termType() != TermType::ATOMIC) throw QueryError("invalid property term {}", *graphTerm);
			tripleData.setGraph(((Atomic *) graphTerm.get())->stringForm());
		} else {
			tripleData.setGraph(mongolog->reasonerManager().backendManager()->vocabulary()->importHierarchy()->defaultGraph());
		}

		// "c" field
		TermPtr confidenceTerm;
		if (!PL_is_variable(t_confidenceTerm)) {
			confidenceTerm = PrologTerm::toKnowRobTerm(t_confidenceTerm);
			if (confidenceTerm->isNumeric()) {
				tripleData.setConfidence(std::static_pointer_cast<Numeric>(confidenceTerm)->asDouble());
			} else {
				throw QueryError("invalid confidence term {}", *confidenceTerm);
			}
		}

		// "b" field
		TermPtr beginTerm;
		if (!PL_is_variable(t_beginTerm)) {
			beginTerm = PrologTerm::toKnowRobTerm(t_beginTerm);
			if (beginTerm->isNumeric()) {
				tripleData.setBegin(std::static_pointer_cast<Numeric>(beginTerm)->asDouble());
			} else {
				throw QueryError("invalid begin term {}", *beginTerm);
			}
		}

		// "e" field
		TermPtr endTerm;
		if (!PL_is_variable(t_endTerm)) {
			endTerm = PrologTerm::toKnowRobTerm(t_endTerm);
			if (endTerm->isNumeric()) {
				tripleData.setEnd(std::static_pointer_cast<Numeric>(endTerm)->asDouble());
			} else {
				throw QueryError("invalid end term {}", *endTerm);
			}
		}

		mongolog->reasonerManager().kb()->edb()->mergeInsert(mongolog->knowledgeGraph(), tripleData);
		//mongolog->kb()->insertOne(tripleData);
		return true;
	} else {
		KB_WARN("[mongolog] unable to assert triple: reasoner not found");
		return false;
	}
}
