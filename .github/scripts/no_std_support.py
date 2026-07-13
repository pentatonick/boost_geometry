#!/usr/bin/env python3
"""Probe every workspace crate for no_std support and splice a table into
README.md between `<!-- no-std-table:start/end -->` markers.

A crate "supports no_std" if `cargo build -p <crate> --no-default-features`
succeeds, trying `--features libm` first for crates that need a libm-backed
Float impl (geometry-coords and its dependents gate `sqrt`/`abs` behind
`std`/`libm`).

Usage: python3 .github/scripts/no_std_support.py
Run from the workspace root.
"""

import re
import subprocess
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
README = ROOT / "README.md"
START = "<!-- no-std-table:start -->"
END = "<!-- no-std-table:end -->"


def workspace_crates():
    manifest = tomllib.loads((ROOT / "Cargo.toml").read_text())
    names = []
    for member in manifest["workspace"]["members"]:
        crate_toml = ROOT / member / "Cargo.toml"
        names.append(tomllib.loads(crate_toml.read_text())["package"]["name"])
    return sorted(names)


def build_ok(name, extra_args):
    result = subprocess.run(
        ["cargo", "build", "-p", name, "--no-default-features", *extra_args],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    return result.returncode == 0


def supports_no_std(name):
    if build_ok(name, ["--features", "libm"]):
        return True
    return build_ok(name, [])


def render_table(rows):
    lines = ["| Crate | `no_std` |", "|---|---|"]
    for name, supported in rows:
        lines.append(f"| `{name}` | {'✅' if supported else '❌'} |")
    return "\n".join(lines)


def main():
    rows = [(name, supports_no_std(name)) for name in workspace_crates()]
    table = render_table(rows)
    block = f"{START}\n{table}\n{END}"

    text = README.read_text()
    pattern = re.compile(re.escape(START) + r".*?" + re.escape(END), re.DOTALL)
    if not pattern.search(text):
        raise SystemExit(f"README.md is missing {START} / {END} markers")
    README.write_text(pattern.sub(block, text))


if __name__ == "__main__":
    main()
