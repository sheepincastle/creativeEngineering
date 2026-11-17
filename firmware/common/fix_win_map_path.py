"""Convert absolute build-dir .map paths in LINKFLAGS to project-relative paths.

Arduino-ESP32 v3.x passes an absolute `$BUILD_DIR/${PROGNAME}.map` path to GNU ld.
When the project directory contains non-ASCII characters (e.g. Korean) on Windows,
SCons writes long link commands to a UTF-8 response file that GCC/ld reads using
the active ANSI code page, causing `cannot open map file` errors.
"""

Import("env")
import os

project_dir = os.path.normpath(env.subst("$PROJECT_DIR"))
build_dir_norm = os.path.normcase(os.path.normpath(env.subst("$BUILD_DIR")))
linkflags = list(env.get("LINKFLAGS", []))

for idx, flag in enumerate(linkflags):
    if isinstance(flag, str) and flag.endswith(".map"):
        norm_flag = os.path.normcase(os.path.normpath(flag))
        if norm_flag.startswith(build_dir_norm + os.sep):
            linkflags[idx] = os.path.relpath(os.path.normpath(flag), project_dir).replace("\\", "/")

env.Replace(LINKFLAGS=linkflags)
