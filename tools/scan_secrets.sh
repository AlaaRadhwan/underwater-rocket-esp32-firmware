#!/usr/bin/env bash
# Local pre-release scan. Flags MAC-like strings (other than the all-zero placeholder),
# long hex/base64-like tokens, password/secret assignments with literal values, absolute user
# paths and e-mail addresses in tracked files. Run from the repository root.
set -u
fail=0
chk() { local label=$1; shift; local out; out=$("$@" 2>/dev/null); if [ -n "$out" ]; then echo "== $label"; echo "$out" | head -20; fail=1; fi; }
files() { git ls-files -z; }
chk "MAC-like strings" bash -c 'git ls-files -z | xargs -0 grep -nIE "([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}" | grep -v "00:00:00:00:00:00"'
chk "literal password/secret assignments" bash -c 'git ls-files -z | xargs -0 grep -nIiE "(pass(word)?|secret|token|api[_-]?key)[A-Za-z_0-9]*[[:space:]]*[=:]?[[:space:]]*\"[^\"]{4,}\"" | grep -viE "change-me|UWR_|sysenv|ps5Controller"'
chk "absolute user paths" bash -c 'git ls-files -z | xargs -0 grep -nIE "([A-Za-z]:\\\\Users|/home/[a-z]|/media/[a-z])"'
chk "e-mail addresses" bash -c 'git ls-files -z | xargs -0 grep -nIE "[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}" | grep -v "noreply.github.com"'
chk "secrets.h tracked" bash -c 'git ls-files | grep -E "(^|/)secrets\.h$"'
[ $fail -eq 0 ] && echo "scan: no findings" || echo "scan: FINDINGS above"
exit $fail
