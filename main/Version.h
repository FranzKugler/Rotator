#pragma once

// Fallback used when the source tree has no reachable v* release tag, git
// itself is unavailable, or an IDE's code model parses this file directly
// (which does not run the build). scripts/version.py overrides
// ROTATOR_VERSION at build time - on every build, not just once - with the
// output of `git describe --tags --dirty --match "v[0-9]*"`, via a custom
// target in main/CMakeLists.txt; it also reads the value below back out via
// regex for its own fallback, so keep the #define on its own line exactly
// as shown. Not kept in sync automatically otherwise - bump it by hand
// occasionally so it stays a plausible fallback, but it never has to be
// exact for a tagged build to report correctly.
#ifndef ROTATOR_VERSION
#define ROTATOR_VERSION "0.12.0"
#endif
