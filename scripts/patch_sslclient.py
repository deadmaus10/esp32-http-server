# scripts/patch_sslclient.py
from SCons.Script import Import
Import("env")

import os, io

def find_sslclient_cpp():
    # PlatformIO provides these
    libdeps_root = env["PROJECT_LIBDEPS_DIR"]          # .../libdeps
    envname      = env["PIOENV"]                       # e.g. "esp32"
    base = os.path.join(libdeps_root, envname)
    for root, dirs, files in os.walk(base):
        if os.path.basename(root) == "SSLClient":
            candidate = os.path.join(root, "src", "SSLClient.cpp")
            if os.path.isfile(candidate):
                return candidate
    return None

def patch_sslclient(target, source, env):  # <-- IMPORTANT: accept these args
    fpath = find_sslclient_cpp()
    if not fpath:
        print("[patch_sslclient] SSLClient.cpp not found (will try again on next build).")
        return

    with io.open(fpath, "r", encoding="utf-8") as f:
        s = f.read()

    if "esp_random()" in s:
        print("[patch_sslclient] Already patched:", fpath)
        return

    # Ensure esp_random() header on ESP32
    needle_inc = "#include <Arduino.h>"
    add_inc = (
        "#include <Arduino.h>\n"
        "#if defined(ARDUINO_ARCH_ESP32)\n"
        "#include <esp_system.h>\n"
        "#endif"
    )
    if needle_inc in s and add_inc not in s:
        s = s.replace(needle_inc, add_inc, 1)

    # Replace the bad analogRead seeding
    before = "rng_seeds[i] = static_cast<uint8_t>(analogRead(m_analog_pin));"
    after  = (
        "#if defined(ARDUINO_ARCH_ESP32)\n"
        "  rng_seeds[i] = static_cast<uint8_t>(esp_random() & 0xFF);\n"
        "#else\n"
        "  rng_seeds[i] = static_cast<uint8_t>(analogRead(m_analog_pin));\n"
        "#endif"
    )
    if before in s:
        s = s.replace(before, after, 1)
    else:
        print("[patch_sslclient] WARNING: expected seeding line not found; file layout changed?")
        return

    with io.open(fpath, "w", encoding="utf-8") as f:
        f.write(s)
    print("[patch_sslclient] Patched:", fpath)

# Run BEFORE compiling so the change is in effect
env.AddPreAction("buildprog", patch_sslclient)
