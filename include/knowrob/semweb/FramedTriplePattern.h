/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_FRAMED_TRIPLE_PATTERN_H
#define KNOWROB_FRAMED_TRIPLE_PATTERN_H

#include "knowrob/formulas/Predicate.h"
#include "knowrob/semweb/FramedTriple.h"
#include "knowrob/semweb/TripleContainer.h"
#include "knowrob/formulas/FirstOrderLiteral.h"
#include "knowrob/queries/QueryContext.h"
#include "knowrob/terms/groundable.h"
#include "knowrob/terms/Numeric.h"
#include "knowrob/terms/IRIAtom.h"
#include "knowrob/terms/Blank.h"

namespace knowrob {
	/**
	 * A triple expression where subject, predicate and object are
	 * represented as a term, and an additional unary operator can be applied to the object.
	 */
	class FramedTriplePattern : public FirstOrderLiteral {
	public:
		/**
		 * Unary operators that can be applied on terms.
		 */
		enum OperatorType {
			EQ, LT, GT, LEQ, GEQ
		};

		/**
		 * Copy char data of StatementData object into Term data structures.
		 * @param tripleData input data, can be deleted afterwards.
		 * @param isNegated a value of true refers to the statement being false.
		 */
		explicit FramedTriplePattern(const FramedTriple &triple, bool isNegated = false);

		/**
		 * @param predicate a predicate with two arguments.
		 * @param isNegated a value of true refers to the statement being false.
		 * @param selector a selector for the graph, agent, begin, end and confidence.
		 */
		explicit FramedTriplePattern(const PredicatePtr &pred, bool isNegated = false);

		/**
		 * @param s the subject term.
		 * @param p the property term.
		 * @param o the object term.
		 * @param isNegated a value of true refers to the statement being false.
		 */
		FramedTriplePattern(const TermPtr &s, const TermPtr &p, const TermPtr &o, bool isNegated = false);

		/**
		 * @param frame the graph, agent, begin, end and confidence.
		 */
		void setTripleFrame(const GraphSelector &frame);

		/**
		 * @return the subject term of this expression.
		 */
		auto subjectTerm() const { return subjectTerm_; }

		/**
		 * @return the property term of this expression.
		 */
		auto propertyTerm() const { return propertyTerm_; }

		/**
		 * @return the object term of this expression.
		 */
		auto objectTerm() const { return objectTerm_; }

		/**
		 * @return the graph term of this expression.
		 */
		auto graphTerm() const { return graphTerm_; }

		/**
		 * Set the graph term of this expression.
		 * @param graphTerm the graph term.
		 */
		void setGraphTerm(const groundable<Atom> &graphTerm) { graphTerm_ = graphTerm; }

		/**
		 * Set the graph term of this expression.
		 * @param graphName the name of the graph.
		 */
		void setGraphName(const std::string_view &graphName) { graphTerm_ = getGraphTerm(graphName); }

		/**
		 * @return the agent term of this expression.
		 */
		auto agentTerm() const { return agentTerm_; }

		/**
		 * Set the agent term of this expression.
		 * @param agentTerm the agent term.
		 */
		void setAgentTerm(const groundable<Atom> &agentTerm) { agentTerm_ = agentTerm; }

		/**
		 * @return the begin term of this expression.
		 */
		auto beginTerm() const { return beginTerm_; }

		/**
		 * Set the begin term of this expression.
		 * @param beginTerm the begin term.
		 */
		void setBeginTerm(const groundable<Double> &beginTerm) { beginTerm_ = beginTerm; }

		/**
		 * @return the end term of this expression.
		 */
		auto endTerm() const { return endTerm_; }

		/**
		 * Set the end term of this expression.
		 * @param endTerm the end term.
		 */
		void setEndTerm(const groundable<Double> &endTerm) { endTerm_ = endTerm; }

		/**
		 * @return the confidence term of this expression.
		 */
		auto confidenceTerm() const { return confidenceTerm_; }

		/**
		 * Set the confidence term of this expression.
		 * @param confidenceTerm the confidence term.
		 */
		void setConfidenceTerm(const groundable<Double> &confidenceTerm) { confidenceTerm_ = confidenceTerm; }

		/**
		 * @return the operator for the object of the triple.
		 */
		auto objectOperator() const { return objectOperator_; }

		/**
		 * Set the operator for the object of the triple.
		 * @param objectOperator the operator.
		 */
		void setObjectOperator(OperatorType objectOperator) { objectOperator_ = objectOperator; }

		/**
		 * @return the temporal operator of this expression.
		 */
		auto temporalOperator() const { return temporalOperator_; };

		/**
		 * Set the temporal operator of this expression.
		 * @param temporalOperator the temporal operator.
		 */
		void setTemporalOperator(TemporalOperator temporalOperator) { temporalOperator_ = temporalOperator; }

		/**
		 * @return the epistemic operator of this expression.
		 */
		auto epistemicOperator() const { return epistemicOperator_; };

		/**
		 * Set the epistemic operator of this expression.
		 * @param epistemicOperator the epistemic operator.
		 */
		void setEpistemicOperator(EpistemicOperator epistemicOperator) { epistemicOperator_ = epistemicOperator; }

		/**
		 * @return the number of variables in this expression.
		 */
		uint32_t numVariables() const override;

		/**
		 * Map the instantiation of this expression into a triple.
		 * @param triple the triple to be instantiated.
		 * @param bindings the substitution to be applied.
		 * @return true if the instantiation was successful.
		 */
		bool instantiateInto(FramedTriple &triple, const std::shared_ptr<const Substitution> &bindings = Substitution::emptySubstitution()) const;

	protected:
		TermPtr subjectTerm_;
		TermPtr propertyTerm_;
		TermPtr objectTerm_;
		OperatorType objectOperator_;

		// below are treated as optional
		groundable<Atom> graphTerm_;
		groundable<Atom> agentTerm_;
		groundable<Double> beginTerm_;
		groundable<Double> endTerm_;
		groundable<Double> confidenceTerm_;
		std::optional<TemporalOperator> temporalOperator_;
		std::optional<EpistemicOperator> epistemicOperator_;

		static std::shared_ptr<Atom> getGraphTerm(const std::string_view &graphName);

		static std::shared_ptr<Predicate> getRDFPredicate(const TermPtr &s, const TermPtr &p, const TermPtr &o);

		static std::shared_ptr<Predicate> getRDFPredicate(const FramedTriple &data);

		static std::shared_ptr<Predicate> getRDFPredicate(const PredicatePtr &predicate);
	};

	/**
	 * A shared pointer to a framed triple pattern.
	 */
	using FramedTriplePatternPtr = std::shared_ptr<FramedTriplePattern>;

	/**
	 * Apply a substitution to a framed triple pattern.
	 * @param pat the framed triple pattern.
	 * @param bindings the substitution.
	 * @return the framed triple pattern with the substitution applied.
	 */
	FramedTriplePatternPtr applyBindings(const FramedTriplePatternPtr &pat, const Substitution &bindings);

	/**
	 * A container that maps a vector of framed triple patterns into a vector of framed triples.
	 */
	class TriplePatternContainer : public semweb::MutableTripleContainer {
	public:
		/**
		 * @param triple a triple query.
		 */
		void push_back(const FramedTriplePatternPtr &q);

		// Override TripleContainer
		ConstGenerator cgenerator() const override;

		// Override MutableTripleContainer
		MutableGenerator generator() override;

	protected:
		std::vector<FramedTriplePtr> data_;
		std::vector<FramedTriplePatternPtr> statements_;
	};
} // knowrob

#endif //KNOWROB_FRAMED_TRIPLE_PATTERN_H
