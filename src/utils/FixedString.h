#pragma once
#include <string.h>
#include <stdio.h>

template <size_t N>
static inline void copyString(char (&dst)[N], const char* src) {
  if (!src) src = "";
  snprintf(dst, N, "%s", src);
}
