/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PY_CONVERTER_OPTIONAL_H
#define KNOWROB_PY_CONVERTER_OPTIONAL_H

#include <boost/python.hpp>
#include "optional"

namespace knowrob::py {
	/** handling of std::optional, map no-value to None in Python. */
	template<typename T>
	struct python_optional : private boost::noncopyable {
		struct conversion : public boost::python::converter::expected_from_python_type<T> {
			static PyObject *convert(std::optional<T> const &value) {
				return nullptr;
			}
		};

		static void *convertible(PyObject *obj) {
			return nullptr;
		}

		static void constructor(
				PyObject *obj,
				boost::python::converter::rvalue_from_python_stage1_data *data
		) {
		}

		explicit python_optional() {
		}
	};
}

#endif //KNOWROB_PY_CONVERTER_OPTIONAL_H
