# Sourced by build.sh and flash.sh. Not executable.
# Gentoo installs ESP-IDF as dev-embedded/esp-idf:
#   framework: /usr/share/esp-idf
#   command:   /usr/bin/idf
# A leftover IDF_PATH that does not contain tools/idf.py is ignored.

if [[ -z "${IDF_PATH:-}" || ! -f "${IDF_PATH}/tools/idf.py" ]]; then
    if [[ -n "${IDF_PATH:-}" ]]; then
        echo "IDF_PATH=${IDF_PATH} has no tools/idf.py; using /usr/share/esp-idf" >&2
    fi
    if [[ -f /usr/share/esp-idf/tools/idf.py ]]; then
        export IDF_PATH=/usr/share/esp-idf
    else
        echo "ESP-IDF was not found. Install dev-embedded/esp-idf, or set IDF_PATH to a tree that contains tools/idf.py." >&2
        return 1
    fi
fi

if [[ -x /usr/bin/idf ]]; then
    idf() { /usr/bin/idf "$@"; }
elif command -v idf.py >/dev/null 2>&1; then
    idf() { idf.py "$@"; }
else
    idf() { python "${IDF_PATH}/tools/idf.py" "$@"; }
fi
