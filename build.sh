#!/bin/sh
# Builds the converter from this tree.
#   ./build.sh mac       -> build/unimog-convert (host, macOS or Linux)
#   ./build.sh android   -> build/libunimogconv.so (arm64 executable;
#                           needs ANDROID_NDK pointing at an NDK r27+)
set -e
cd "$(dirname "$0")"
T=${1:-mac}
B=$PWD/build; mkdir -p "$B"
# 1. Upstream sources + our patch
if [ ! -d "$B/libredwg-0.14" ]; then
  tar -xJf third_party/libredwg-0.14.tar.xz -C "$B"
  patch -d "$B" -p0 < patches/libredwg-0.14-unimog-streaming.patch
fi
[ -d "$B/zstd-1.5.7" ] || tar -xzf third_party/zstd-1.5.7.tar.gz -C "$B"
# 2. Toolchain
case $T in
  mac) CC=${CC:-cc}; AR=${AR:-ar}; HOST=; OUTBIN=$B/unimog-convert; CFL="-O2" ;;
  android)
    [ -n "$ANDROID_NDK" ] || { echo "set ANDROID_NDK" >&2; exit 1; }
    TC=$(echo "$ANDROID_NDK"/toolchains/llvm/prebuilt/*)
    CC=$TC/bin/aarch64-linux-android29-clang; AR=$TC/bin/llvm-ar
    HOST=--host=aarch64-linux-android; OUTBIN=$B/libunimogconv.so
    CFL="-O2 -fPIC" ;;
  *) echo "usage: $0 mac|android" >&2; exit 2 ;;
esac
# 3. LibreDWG (static, library only)
O=$B/obj-$T/libredwg; mkdir -p "$O"
if [ ! -f "$O/src/.libs/libredwg.a" ]; then
  (cd "$O" && "$B/libredwg-0.14/configure" $HOST --disable-shared --enable-static \
     --disable-bindings --disable-python --disable-docs --disable-werror \
     CC="$CC" AR="$AR" CFLAGS="$CFL" >/dev/null && make -C src -j4 >/dev/null)
fi
# 4. zstd (static, compress + decompress)
Z=$B/zstd-1.5.7/lib; ZO=$B/obj-$T/zstd; mkdir -p "$ZO"
if [ ! -f "$ZO/libzstd.a" ]; then
  for f in "$Z"/common/*.c "$Z"/compress/*.c "$Z"/decompress/*.c; do
    "$CC" $CFL -DZSTD_DISABLE_ASM -c "$f" -o "$ZO/$(basename "$f" .c).o"
  done
  "$AR" rcs "$ZO/libzstd.a" "$ZO"/*.o
fi
# 5. Converter
INC="-Iconverter -Icore -I$B/libredwg-0.14/include -I$O/src -I$Z"
if [ "$T" = android ]; then PIE="-fPIE -pie"; else PIE=; fi
"$CC" -O2 $PIE -Wno-unused-function -o "$OUTBIN" converter/unimog_convert.c $INC \
  "$O/src/.libs/libredwg.a" "$ZO/libzstd.a" -lm
[ "$T" = android ] && "$TC/bin/llvm-strip" "$OUTBIN"
ls -l "$OUTBIN"
