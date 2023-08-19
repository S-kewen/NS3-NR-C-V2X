# Copyright (c) 2022 Universidade de Brasília
#
# This program is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License version 2 as published by the Free
# Software Foundation;
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc., 59 Temple
# Place, Suite 330, Boston, MA  02111-1307 USA
#
# Author: Gabriel Ferreira <gabrielcarvfer@gmail.com>

include(CheckCXXSourceCompiles)

# Clang 6, 7 and 8 shipped with incomplete C++17 features and do not handle
# ostream& operator<<(ostream& os, nullptr_t ptr)
# https://gitlab.com/nsnam/ns-3-dev/-/issues/730
check_cxx_source_compiles(
  "
    #include <iostream>
    #include <cstddef>
    inline std::ostream& operator << (std::ostream& os, std::nullptr_t ptr)
    {
      return os << \"nullptr\"; //whatever you want nullptr to show up as in the console
    }
    int main()
    {
        std::ostream os(NULL);
        os << std::nullptr_t();
        return 0;
    }
    "
  MISSING_OSTREAM_NULLPTR_OPERATOR
)

if(${MISSING_OSTREAM_NULLPTR_OPERATOR})
  message(
    ${HIGHLIGHTED_STATUS}
    "Using compiler workaround: compiling in \"ostream& operator<<(ostream&, nullptr_t)\""
  )
  add_definitions(
    -include
    ${CMAKE_CURRENT_SOURCE_DIR}/build-support/compiler-workarounds/ostream-operator-nullptr.h
  )
endif()
