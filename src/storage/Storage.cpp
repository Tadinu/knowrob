#include "knowrob/storage/Storage.h"
#include "knowrob/integration/python/utils.h"
#include "knowrob/storage/QueryableStorage.h"

using namespace knowrob;

StorageFeature knowrob::operator|(StorageFeature a, StorageFeature b) {
	return static_cast<StorageFeature>(static_cast<std::uint8_t>(a) | static_cast<std::uint8_t>(b));
}

bool knowrob::operator&(StorageFeature a, StorageFeature b) {
	return static_cast<std::uint8_t>(a) & static_cast<std::uint8_t>(b);
}

namespace knowrob::py {
	struct StorageWrap : public Storage {
		explicit StorageWrap() : Storage() {}

		bool initializeBackend(const PropertyTree &config) override {
		}

		bool insertOne(const FramedTriple &triple) override {
		}

		bool insertAll(const TripleContainerPtr &triples) override {
		}

		bool removeOne(const FramedTriple &triple) override {
		}

		bool removeAll(const TripleContainerPtr &triples) override {
		}

		bool removeAllWithOrigin(std::string_view origin) override {
		}

	private:

	};

	template<>
	void createType<Storage>() {
	}
}
