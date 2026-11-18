#!/bin/sh
# 19_Final_Project/host - the swarm on the PC: build and test.  See test.bat.
set -e
cd "$(dirname "$0")"
mkdir -p out
SIM="../fleet.c ../pilot.c ../craft.c ../trig.c"
case "$(uname -s)" in
  MINGW*|MSYS*|CYGWIN*) lib=out/swarm.dll; extra=-static-libgcc; exe=out/test_show.exe ;;
  Darwin*)              lib=out/libswarm.dylib; extra=; exe=out/test_show ;;
  *)                    lib=out/libswarm.so; extra=-fPIC; exe=out/test_show ;;
esac
gcc -std=c11 -O2 -Wall -Wextra -Wno-format-truncation -I.. -I../../_lib -o "$exe" test_show.c     ../plan.c ../director.c ../show.c ../fmt.c ../../_lib/proto.c $SIM -lm
gcc -std=c11 -O2 -Wall -Wextra -Werror -shared $extra -DFLEET_MAX=64 -I.. -I../../_lib -o "$lib" swarm.c $SIM
"./$exe"
uv run test_swarm.py
