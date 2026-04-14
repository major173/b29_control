// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <stdexcept>
#include <string_view>

#include <b29_smc_auto_controller/command_dispatcher.h>

namespace b29_smc_auto_controller
{
inline CommandDispatcher::OutputMode parseOutputMode(std::string_view value)
{
  if (value == "normal")
  {
    return CommandDispatcher::OutputMode::kNormal;
  }

  if (value == "safe_hold")
  {
    return CommandDispatcher::OutputMode::kSafeHold;
  }

  throw std::invalid_argument("unsupported output_mode");
}

inline const char* toString(CommandDispatcher::OutputMode mode)
{
  switch (mode)
  {
    case CommandDispatcher::OutputMode::kSafeHold:
      return "safe_hold";
    case CommandDispatcher::OutputMode::kNormal:
    default:
      return "normal";
  }
}
}  // namespace b29_smc_auto_controller
