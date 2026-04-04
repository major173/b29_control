// SPDX-License-Identifier: BSD-3-Clause
#pragma once

// Generated SMC headers include <statemap.h>. During local builds we want to
// pick up the next runtime header on the include path (typically the repo
// bundled SMC runtime). For downstream consumers that only get ROS's smclib
// export, fall back to <smclib/statemap.h>.
#if defined(__has_include_next)
#  if __has_include_next(<statemap.h>)
#    include_next <statemap.h>
#  elif defined(__has_include) && __has_include(<smclib/statemap.h>)
#    include <smclib/statemap.h>
#  else
#    error "statemap.h not found. Install smclib or provide the SMC runtime include path."
#  endif
#elif defined(__has_include) && __has_include(<smclib/statemap.h>)
#  include <smclib/statemap.h>
#else
#  error "statemap.h not found. Install smclib or provide the SMC runtime include path."
#endif
