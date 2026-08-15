#ifndef ARM_BEHAVIOR__OPERATION_RESULT_HPP_
#define ARM_BEHAVIOR__OPERATION_RESULT_HPP_

#include <string>

namespace arm
{

struct OperationResult
{
  bool success;
  std::string message;
};

}  // namespace arm

#endif  // ARM_BEHAVIOR__OPERATION_RESULT_HPP_
