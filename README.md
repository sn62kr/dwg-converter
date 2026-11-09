# DWG converter (GPL source)

이 저장소는 Android 앱 `app.sn62kr.plan`(버전 0.1.25)에 들어 있는 **DWG 변환 프로그램**
(`libunimogconv.so`)의 전체 소스입니다. GNU GPL v3 이상(COPYING)에 따라 공개합니다.

This repository is the complete corresponding source of the DWG converter
(`libunimogconv.so`) shipped in the Android app `app.sn62kr.plan` (0.1.25), released
under the GNU GPL version 3 or later (see COPYING).

## 구성 / Contents

| 경로 | 내용 | 라이선스 |
|---|---|---|
| converter/, core/tess.h | 변환기: DWG → 앱 캐시(UMG8) | GPL-3.0-or-later |
| patches/libredwg-0.14-unimog-streaming.patch | LibreDWG 수정분: 엔티티를 읽는 즉시 변환·해제(R2004+·R2007), 확장 데이터(EED) 건너뛰기 — 메모리 절감; R2007 페이지 압축 크기가 잘못 적힌 파일 읽기 | GPL-3.0-or-later |
| third_party/libredwg-0.14.tar.xz (+ .sig) | LibreDWG 0.14 원본 (https://www.gnu.org/software/libredwg/) | GPL-3.0-or-later |
| third_party/zstd-1.5.7.tar.gz | Zstandard 1.5.7 원본 (https://github.com/facebook/zstd) | BSD (licenses/zstd-BSD.txt) |
| converter/earcut.h | mapbox earcut의 C 이식본 (https://github.com/mapbox/earcut) | ISC (licenses/earcut-ISC.txt) |

앱의 나머지(화면, 측정, 스냅 등)는 이 변환기와 별도 프로그램이며 표준 입출력과 파일로만 통신합니다.
The rest of the app is a separate program that runs this converter as a
separate process and talks to it only through files and stdout.

## 빌드 / Build

    ./build.sh mac                                   # build/unimog-convert
    ANDROID_NDK=/path/to/ndk ./build.sh android      # build/libunimogconv.so (arm64)

필요: C 컴파일러, make, xz, patch. 사용법: `unimog-convert in.dwg out.umg`

## 앱에 설치된 변환기 교체 / Installing a modified converter

변환기는 APK의 `lib/arm64-v8a/libunimogconv.so`입니다. 수정한 변환기로 바꾸려면
APK의 그 파일을 교체한 뒤 다시 서명해서 설치하면 됩니다(서명이 달라지므로 기존 앱 삭제 후 설치).
