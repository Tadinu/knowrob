//
// Created by daniel on 01.04.23.
//

#ifndef KNOWROB_MONGO_KNOWLEDGE_GRAPH_H
#define KNOWROB_MONGO_KNOWLEDGE_GRAPH_H

#include <optional>
#include <list>
#include "boost/property_tree/ptree.hpp"
#include "knowrob/backend/KnowledgeGraph.h"
#include "knowrob/semweb/Vocabulary.h"
#include "Collection.h"
#include "Cursor.h"
#include "knowrob/queries/AnswerBuffer.h"
#include "knowrob/formulas/Literal.h"
#include "knowrob/mongodb/TripleLoader.h"
#include "knowrob/mongodb/AnswerCursor.h"
#include "knowrob/semweb/ImportHierarchy.h"

namespace knowrob {
    /**
     * A knowledge graph implemented with MongoDB.
     */
    class MongoKnowledgeGraph : public knowrob::KnowledgeGraph {
    public:
        MongoKnowledgeGraph();

        explicit MongoKnowledgeGraph(
                const char* db_uri,
                const char* db_name="knowrob",
                const char* collectionName="triples");

        const auto& importHierarchy() const { return importHierarchy_; }

        /**
         * (re)create search indices.
         */
        void createSearchIndices();

        /**
         * Delete all triples of a named graph in this knowledge graph.
         * @param graphName a graph name
         */
        void dropGraph(const std::string_view &graphName);

        /**
         * Delete all triples in the database.
         * Note: ths will also delete all indices which need to be re-created afterwards.
         */
        void drop();

        /**
         * @param graphName the name of a graph
         * @return the version string associated to the named graph if any
         */
        std::optional<std::string> getCurrentGraphVersion(const std::string &graphName);

        // Override KnowledgeGraph
        bool loadConfiguration(const boost::property_tree::ptree &config) override;

        // Override KnowledgeGraph
        bool insert(const TripleData &tripleData) override;

        // Override KnowledgeGraph
        bool insert(const std::vector<TripleData> &tripleData) override;

        // Override KnowledgeGraph
        void removeAll(const semweb::TripleExpression &tripleExpression) override;

        // Override KnowledgeGraph
        void removeOne(const semweb::TripleExpression &tripleExpression) override;

        // Override KnowledgeGraph
        bool loadTriples(
                const std::string_view &uriString,
                TripleFormat format,
                const std::optional<ModalIteration> &modality) override;

        // Override KnowledgeGraph
        void evaluateQuery(const GraphQueryPtr &query, AnswerBufferPtr &resultStream) override;

        // Override KnowledgeGraph
        AnswerBufferPtr watchQuery(const GraphQueryPtr &literal) override;

        /**
         * Lookup up all matching triples.
         * @param tripleExpression a triple expression
         * @return a cursor over matching triples
         */
        mongo::AnswerCursorPtr lookupTriples(const semweb::TripleExpression &tripleExpression);

        mongo::AnswerCursorPtr lookupTriples(const TripleData &tripleData)
        { return lookupTriples(semweb::TripleExpression(tripleData)); }

        /**
         * Lookup up a path of matching triples.
         * The lookup pipeline includes a step for each expression in the vector
         * in the same order as the expressions are ordered in the vector.
         * @param tripleExpressions a vector of triple expressions
         * @return a cursor over matching triples
         */
        mongo::AnswerCursorPtr lookupTriplePaths(const std::list<semweb::TripleExpression> &tripleExpressions);

    protected:
        std::shared_ptr<mongo::Collection> tripleCollection_;
        std::shared_ptr<mongo::Collection> oneCollection_;
        std::shared_ptr<semweb::ImportHierarchy> importHierarchy_;

        void initialize();

        void setCurrentGraphVersion(const std::string &graphName,
                                    const std::string &graphURI,
                                    const std::string &graphVersion);

        static std::shared_ptr<mongo::Collection> connect(const boost::property_tree::ptree &config);

        static const char* getDBName(const boost::property_tree::ptree &config);

        static const char* getCollectionName(const boost::property_tree::ptree &config);

        static std::string getURI(const boost::property_tree::ptree &config);

        void updateHierarchy(mongo::TripleLoader &tripleLoader);

        void updateTimeInterval(const TripleData &tripleLoader);

        static bson_t* getTripleSelector(const semweb::TripleExpression &tripleExpression,
                                  bool isTaxonomicProperty);

        bool isTaxonomicProperty(const TermPtr &propertyTerm);
    };

} // knowrob::mongo

#endif //KNOWROB_MONGO_KNOWLEDGE_GRAPH_H
