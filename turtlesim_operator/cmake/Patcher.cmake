#
# Copyright (c) 2017, 2021 ADLINK Technology Inc.
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
# which is available at https://www.apache.org/licenses/LICENSE-2.0.
#
# SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
#
# Contributors:
#   ADLINK zenoh team, <zenoh@adlink-labs.tech>
#

message(STATUS "Patching ${HEADER}")

# Read all the contents of `HEADER` in `header_unpatched`.
file(READ ${HEADER} header_unpatched)

# Replace the faulty line with an empty string.
# NOTE: `header_unpatched` must be put between quotes as otherwise CMake will
# interpret the ";" as a list separator.
string(REPLACE "    using State = ::zenoh::flow::State;" ""
  header_patched "${header_unpatched}")

# Write over the contents of `HEADER` with the patched content.
# NOTE: `header_patched` must also be put between quotes as otherwise CMake
# will interpret the ";" as list separators.
file(WRITE ${HEADER} "${header_patched}")

message(STATUS "Patching ${HEADER} - done")
