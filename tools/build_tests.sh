#!/bin/bash
# This script is used by CI to build all test functions

SCRIPT_DIR=$(dirname "$0")
TEST_FUNCTIONS=$(perl -nE 'say $1 if /void (test_\w+)\s*\(/' src/test/test.c)
for test_function in $TEST_FUNCTIONS
do
    make HW=ROBOTIC_ARM TEST=$test_function
    make HW=ARM_SLEEVE TEST=$test_function
done