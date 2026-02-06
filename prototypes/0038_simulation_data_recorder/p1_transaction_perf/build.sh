#!/bin/bash
# Build script for prototype P1

set -e

# Find Conan installation
CONAN_HOME="${HOME}/.conan2"
CPP_SQLITE_ROOT=$(find ${CONAN_HOME}/p/b -name "cpp_sfa*" -type d | head -1)

if [ -z "$CPP_SQLITE_ROOT" ]; then
    echo "Error: Could not find cpp_sqlite in Conan cache"
    exit 1
fi

CPP_SQLITE_INCLUDE="${CPP_SQLITE_ROOT}/p/include"
CPP_SQLITE_LIB="${CPP_SQLITE_ROOT}/p/lib"

# Find other dependencies
SQLITE3_ROOT=$(find ${CONAN_HOME}/p/b -name "sqlite*" -type d | grep -v cpp_sqlite | head -1)
SPDLOG_ROOT=$(find ${CONAN_HOME}/p/b -name "spdlog*" -type d | head -1)
BOOST_ROOT=$(find ${CONAN_HOME}/p/b -name "boost*" -type d | head -1)

SQLITE3_INCLUDE="${SQLITE3_ROOT}/p/include"
SQLITE3_LIB="${SQLITE3_ROOT}/p/lib"
SPDLOG_INCLUDE="${SPDLOG_ROOT}/p/include"
BOOST_INCLUDE="${BOOST_ROOT}/p/include"

echo "Building P1 Transaction Performance..."
echo "cpp_sqlite: ${CPP_SQLITE_ROOT}"
echo "sqlite3: ${SQLITE3_ROOT}"
echo "spdlog: ${SPDLOG_ROOT}"
echo "boost: ${BOOST_ROOT}"

# Compile
c++ -std=c++20 -O2 \
    -I${CPP_SQLITE_INCLUDE} \
    -I${SQLITE3_INCLUDE} \
    -I${SPDLOG_INCLUDE} \
    -I${BOOST_INCLUDE} \
    -I.. \
    main.cpp \
    -L${CPP_SQLITE_LIB} -lcpp_sqlite \
    -L${SQLITE3_LIB} -lsqlite3 \
    -o p1_transaction_perf

echo "Build complete! Run with: ./p1_transaction_perf"
