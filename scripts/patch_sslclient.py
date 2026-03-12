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

    changed = False

    # ---- Patch 1: ESP32 RNG seeding for SSL entropy ----
    if "esp_random()" not in s:
        needle_inc = "#include <Arduino.h>"
        add_inc = (
            "#include <Arduino.h>\n"
            "#if defined(ARDUINO_ARCH_ESP32)\n"
            "#include <esp_system.h>\n"
            "#endif"
        )
        if needle_inc in s and add_inc not in s:
            s = s.replace(needle_inc, add_inc, 1)
            changed = True

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
            changed = True
        else:
            print("[patch_sslclient] WARNING: expected RNG seeding line not found; file layout changed?")

    # ---- Patch 2: host connect fallback (DNS+IP) while keeping hostname verification ----
    host_fallback_marker = "SSLCLIENT_HOST_DNS_FALLBACK_PATCH"
    if host_fallback_marker not in s:
        if '#include "Dns.h"' not in s:
            s = s.replace('#include "SSLClient.h"', '#include "SSLClient.h"\n#include "Dns.h"', 1)
            changed = True

        old_connect_host = """int SSLClient::connect(const char *host, uint16_t port) {
    const char* func_name = __func__;
    // connection check
    if (get_arduino_client().connected())
        m_warn("Arduino client is already connected? Continuing anyway...", func_name);
    // reset indexs for saftey
    m_write_idx = 0;
    // first we need our hidden client member to negotiate the socket for us,
    // since most times socket functionality is implemented in hardeware.
    if (!get_arduino_client().connect(host, port)) {
        m_error("Failed to connect using m_client. Are you connected to the internet?", func_name);
        setWriteError(SSL_CLIENT_CONNECT_FAIL);
        return 0;
    }
    m_info("Base client connected!", func_name);
    // start ssl!
    return m_start_ssl(host, getSession(host));
}
"""

        new_connect_host = """int SSLClient::connect(const char *host, uint16_t port) {
    const char* func_name = __func__;
    // connection check
    if (get_arduino_client().connected())
        m_warn("Arduino client is already connected? Continuing anyway...", func_name);
    // reset indexs for saftey
    m_write_idx = 0;

    // [SSLCLIENT_HOST_DNS_FALLBACK_PATCH]
    // Try native host connect first.
    if (get_arduino_client().connect(host, port)) {
        m_info("Base client connected!", func_name);
        return m_start_ssl(host, getSession(host));
    }

    // Fallback: resolve explicitly via DNSClient and connect by IP.
    // We still pass host into m_start_ssl so SNI + hostname verification remain active.
    IPAddress resolved_ip;
    DNSClient dns;
    dns.begin(Ethernet.dnsServerIP());
    int dns_rc = dns.getHostByName(host, resolved_ip);
    if (dns_rc == 1 && get_arduino_client().connect(resolved_ip, port)) {
        m_warn("Base connect(host) failed; fallback connect(IP) succeeded", func_name);
        m_info("Base client connected!", func_name);
        return m_start_ssl(host, getSession(host));
    }

    m_error("Failed to connect using m_client. Are you connected to the internet?", func_name);
    setWriteError(SSL_CLIENT_CONNECT_FAIL);
    return 0;
}
"""
        if old_connect_host in s:
            s = s.replace(old_connect_host, new_connect_host, 1)
            changed = True
        else:
            print("[patch_sslclient] WARNING: expected connect(host) block not found; file layout changed?")

    if not changed:
        print("[patch_sslclient] Already patched:", fpath)
        return

    with io.open(fpath, "w", encoding="utf-8") as f:
        f.write(s)
    print("[patch_sslclient] Patched:", fpath)

# Run BEFORE compiling so the change is in effect
env.AddPreAction("buildprog", patch_sslclient)
