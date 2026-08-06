# Wrapper Makefile — forwards every target to Makefile2 with -j set to the
# host's CPU count, so builds are multithreaded by default on Windows, macOS,
# and Linux without the caller having to pass -j themselves.

MAKEFLAGS += --no-print-directory

ifeq ($(OS),Windows_NT)
NPROCS := $(NUMBER_OF_PROCESSORS)
else
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
NPROCS := $(shell sysctl -n hw.ncpu 2>/dev/null)
else
NPROCS := $(shell nproc 2>/dev/null || grep -c ^processor /proc/cpuinfo 2>/dev/null)
endif
endif

ifeq ($(strip $(NPROCS)),)
NPROCS := 1
endif

.DEFAULT_GOAL := quick

# Prevent make from trying to remake the Makefiles themselves using the % rule
Makefile Makefile2: ;

# The clean pattern match rule without FORCE dependency
%:
	@$(MAKE) -f Makefile2 -j$(NPROCS) $@
