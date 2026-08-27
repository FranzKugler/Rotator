#pragma once

// Fallback used when the source tree has no reachable v* release tag.
// The build system overrides this with `git describe` when possible.
#ifndef ROTATOR_VERSION
#define ROTATOR_VERSION "0.8.0"
#endif
