# PlatformIO pre script: keep the builder's home folder out of the firmware.
# Header asserts embed __FILE__ (e.g. ~/.platformio/.../gpio_ll.h); map the home path to "~".
import os

Import("env")

home = os.path.expanduser("~")
env.Append(CCFLAGS=["-fmacro-prefix-map=%s=~" % home, "-fdebug-prefix-map=%s=~" % home])
