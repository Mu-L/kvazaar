#!/bin/sh

# Test GOP, with and without OWF.

set -eu
. "${0%/*}/util.sh"

common_args='--preset=veryslow -p0 --threads=2 --wpp'

# Skip valgrind tests if KVZ_TEST_VALGRIND is not set to 1
if [ "${KVZ_TEST_CHROMASUBSAMPLING:-0}" != '0' ]; then


  valgrind_test_444 264x130 10 $common_args --gop=8 -p0 --owf=1
  valgrind_test_444 264x130 10 $common_args --gop=16 -p0 --owf=1 --cross-comp-pred

  valgrind_test_422 264x130 10 $common_args --gop=8 -p0 --owf=1
  valgrind_test_422 264x130 10 $common_args --gop=16 -p0 --owf=1 --cross-comp-pred

  # Do more extensive tests in a private gitlab CI runner
  if [ ! -z ${GITLAB_CI+x} ];then valgrind_test_444 264x130 40 $common_args --gop=8 -p32 --owf=4 --no-open-gop; fi
  if [ ! -z ${GITLAB_CI+x} ];then valgrind_test_422 264x130 40 $common_args --gop=8 -p32 --owf=4 --no-open-gop; fi

fi