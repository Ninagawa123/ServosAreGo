#!/usr/bin/env python3
"""ブラウザ版 ServosAreGo（1ファイルのHTML）を組み立てる。

web/src/index.html に、次のものを埋め込んで ServosAreGo.html（リポジトリの第一階層）を出力する。
  - ファームウェア（PlatformIO のビルド結果: bootloader / partitions / boot_app0 / firmware）
  - 書き込み用ライブラリ esptool-js（web/vendor/esptool-js.bundle.js）
  - ロゴ（web/assets/logo.png）

使い方（ServosAreGo/web/ で）:
  python3 build_html.py           # pio run でビルドしてから組み立てる
  python3 build_html.py --no-build # 今あるビルド結果で組み立てる
"""

import base64
import json
import os
import re
import subprocess
import sys

WEB_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.join(WEB_DIR, "..", "ServosAreGo")
BUILD_DIR = os.path.join(PROJECT_DIR, ".pio", "build", "esp32dev")
OUTPUT = os.path.normpath(os.path.join(WEB_DIR, "..", "ServosAreGo.html"))

# 書き込みアドレス（ESP32 Arduino の標準配置）
FLASH_PARTS = [
    (0x1000, os.path.join(BUILD_DIR, "bootloader.bin")),
    (0x8000, os.path.join(BUILD_DIR, "partitions.bin")),
    (0xE000, None),  # boot_app0.bin（Arduino フレームワークの中にある）
    (0x10000, os.path.join(BUILD_DIR, "firmware.bin")),
]


def find_boot_app0():
    core_dir = os.environ.get("PLATFORMIO_CORE_DIR", os.path.join(os.path.expanduser("~"), ".platformio"))
    path = os.path.join(core_dir, "packages", "framework-arduinoespressif32", "tools", "partitions", "boot_app0.bin")
    if not os.path.exists(path):
        sys.exit("boot_app0.bin not found: " + path)
    return path


def read_fw_version():
    with open(os.path.join(PROJECT_DIR, "src", "main.cpp"), encoding="utf-8") as f:
        match = re.search(r'#define FW_VERSION "([^"]+)"', f.read())
    if not match:
        sys.exit("FW_VERSION not found in src/main.cpp")
    return match.group(1)


def read_kind_names():
    """SERVO_KINDS の種類名（接続前の一覧表示用）"""
    with open(os.path.join(PROJECT_DIR, "src", "main.cpp"), encoding="utf-8") as f:
        source = f.read()
    table = source[source.index("const ServoKind SERVO_KINDS[]"):]
    table = table[: table.index("};")]
    return re.findall(r'^\s*\{"([^"]+)"', table, re.M)


def b64_file(path):
    with open(path, "rb") as f:
        return base64.b64encode(f.read()).decode("ascii")


def main():
    if "--no-build" not in sys.argv:
        subprocess.run(["pio", "run"], cwd=PROJECT_DIR, check=True)

    version = read_fw_version()
    parts = []
    for address, path in FLASH_PARTS:
        path = path or find_boot_app0()
        if not os.path.exists(path):
            sys.exit("not found (run pio run first): " + path)
        parts.append({"address": address, "data": b64_file(path)})
    firmware_json = json.dumps({"version": version, "parts": parts}, separators=(",", ":"))

    with open(os.path.join(WEB_DIR, "src", "index.html"), encoding="utf-8") as f:
        html = f.read()
    replacements = {
        "/*__FW_VERSION__*/": version,
        "/*__KINDS_JSON__*/": json.dumps(read_kind_names()),
        "/*__LOGO_B64__*/": b64_file(os.path.join(WEB_DIR, "assets", "logo.png")),
        "/*__ESPTOOL_B64__*/": b64_file(os.path.join(WEB_DIR, "vendor", "esptool-js.bundle.js")),
        "/*__FIRMWARE_JSON__*/": firmware_json,
    }
    for key, value in replacements.items():
        if key not in html:
            sys.exit("placeholder not found: " + key)
        html = html.replace(key, value)

    with open(OUTPUT, "w", encoding="utf-8") as f:
        f.write(html)
    print("wrote %s (%d KB, firmware %s)" % (os.path.relpath(OUTPUT), len(html) // 1024, version))


if __name__ == "__main__":
    main()
