/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <boost/python.hpp>
#include "knowrob/integration/python/PythonError.h"
#include "knowrob/Logger.h"

using namespace knowrob;

PythonError::PythonError(ErrorData *errorData)
		: KnowRobError(errorData->exc_type, errorData->exc_msg, errorData->exc_trace),
		  errorData_(errorData) {
	if (errorData_->exc_file.has_value()) {
		setFile(errorData_->exc_file.value());
	}
	if (errorData_->exc_line.has_value()) {
		setLine(errorData_->exc_line.value());
	}
}

PythonError::PythonError()
		: PythonError(makeErrorData()) {
}

PythonError::ErrorData *PythonError::emptyErrorData(ErrorData *errorData) {
	errorData->exc_type = "UnknownError";
	errorData->exc_msg = "";
	errorData->exc_trace = "";
	errorData->exc_file = "";
	errorData->exc_line = 0;
	return errorData;
}

PythonError::ErrorData *PythonError::makeErrorData() {
    return nullptr;
}
