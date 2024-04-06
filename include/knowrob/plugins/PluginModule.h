/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PLUGIN_MODULE_H_
#define KNOWROB_PLUGIN_MODULE_H_

#include <string>
#include <memory>
#include <set>
#include <filesystem>
//#include <boost/python.hpp>
#include "knowrob/plugins/PluginFactory.h"
#include "knowrob/integration/python/utils.h"
#include "knowrob/Logger.h"

namespace knowrob {
	/**
	 * A plugin module is a plugin factory that creates plugin objects
	 * that are implemented in Python code.
	 */
	template<class T>
	class PluginModule : public PluginFactory<T> {
	public:
		/**
		 * @param modulePath the name or path of the Python module.
		 * @param pluginType the name of the plugin type.
		 */
		PluginModule(std::string_view modulePath, std::string_view pluginType)
				: modulePath_(knowrob::py::resolveModulePath(modulePath)),
				  pluginType_(pluginType) {
		}

		/**
		 * Cannot be copy-assigned.
		 */
		PluginModule(const PluginModule &) = delete;

		/**
		 * @return true if the module was loaded successfully.
		 */
		bool isLoaded() {
			return knowrob::py::call<bool>([&] {
				//return pyPluginType_ && !pyPluginType_.is_none();
				return false;
			});
		}

		/**
		 * Try loading the module from filesystem.
		 * @return true on success.
		 */
		bool loadModule() {
			// try to make sure that the module can be imported.
			// the modules can be addressed either via an absolute path,
			// or via a relative path in the KnowRob project.
			std::filesystem::path modulePath(modulePath_);
			if (!std::filesystem::exists(modulePath)) {
				KB_ERROR("Module '{}' does not exist.", modulePath_.c_str());
				return false;
			}
			return isLoaded();
		}

		// Override PluginFactory
		std::shared_ptr<NamedPlugin<T>> create(std::string_view pluginID) override {
#if 0
			try {
				boost::python::object pyReasoner = pyPluginType_();
				// extract the reasoner in appropriate type
				boost::python::extract<std::shared_ptr<T>> extracted(pyReasoner);
				if (extracted.check()) {
					return std::make_shared<NamedPlugin<T>>(pluginID, extracted());
				} else {
					KB_ERROR("Failed to extract typed plugin from module '{}'", modulePath_.c_str());
				}
			} catch (const boost::python::error_already_set &) {
				throw PythonError();
			}
			KB_ERROR("Failed to create plugin from module '{}'", modulePath_.c_str());
#endif
			return {};
		}

		// Override PluginFactory
		std::string_view name() const override { return name_; };

	protected:
		const std::string modulePath_;
		const std::string pluginType_;
		//boost::python::object pyModule_;
		//boost::python::object pyPluginType_;
		std::string name_;
	};
}

#endif //KNOWROB_REASONER_MODULE_H_
