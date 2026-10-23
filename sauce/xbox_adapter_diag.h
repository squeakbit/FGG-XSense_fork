#ifndef FGG_XBOX_ADAPTER_DIAG_H
#define FGG_XBOX_ADAPTER_DIAG_H

#include <stddef.h>

int xbox_adapter_run_diagnostics(const char *node, char *why, size_t whylen);

#endif
