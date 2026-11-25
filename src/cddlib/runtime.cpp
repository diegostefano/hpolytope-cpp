// Copyright (C) 2026 Diego Stéfano Fonseca Ferreira
// SPDX-License-Identifier: GPL-3.0-or-later

#include "runtime.hpp"

// clang-format off
#include <cddlib/setoper.h>
#include <cddlib/cdd.h>
// clang-format on

namespace hpolytope::cddlib {

namespace {

class Runtime {
public:
  Runtime() { dd_set_global_constants(); }

  ~Runtime() { dd_free_global_constants(); }
};

} // namespace

void ensureCddlibInitialized() { static Runtime runtime; }

} // namespace hpolytope::cddlib
