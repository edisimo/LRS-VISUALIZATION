#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
python3 -m unittest discover -s tests -p 'test_*.py' -v
if command -v node >/dev/null; then
  node --test tests/*.test.js
  for file in app.js common/*.js mapping/*.js planning/*.js trajectory/*.js avoidance/*.js; do node --check "$file"; done
fi
