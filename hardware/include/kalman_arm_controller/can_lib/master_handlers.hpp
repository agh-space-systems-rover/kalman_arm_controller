#include <unordered_map>
#include <cstdint>
#include "kalman_arm_controller/can_lib/can_types.hpp"

namespace CAN_handlers
{
// Define the command handler array
extern std::unordered_map<uint8_t, canCmdHandler_t> MASTER_HANDLES;

}  // namespace CAN_handlers