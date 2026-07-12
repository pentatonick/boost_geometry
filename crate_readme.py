#!/usr/bin/env python3
"""Regenerate each sub-crate's README.md from its lib.rs `//!` doc comment.

Usage: python3 crate_readme.py
Run from the workspace root. The facade crate (crates/geometry) is skipped —
it packages the workspace-root README.md instead.
"""

import re
from pathlib import Path

FACADE_DIR = "geometry"

HEADER = (
    "# {name}\n\n"
    "Part of the [boost_geometry](https://crates.io/crates/boost_geometry)"
    " workspace — a Rust port of"
    " [Boost.Geometry](https://www.boost.org/doc/libs/release/libs/geometry/)."
    " Most users should depend on the facade crate, which re-exports this one;"
    " depend on this crate directly only for a slimmer build.\n\n"
)

FOOTER = (
    "\n## License\n\nBSL-1.0 — see"
    " [LICENSE](https://github.com/pentatonick/boost_geometry/blob/main/LICENSE).\n"
)

FENCE_RE = re.compile(r"^```(\S*)$")
# rustdoc fence infostrings that mean "this is rust code"
RUST_INFOSTRINGS = {"", "rust", "ignore", "no_run", "should_panic", "compile_fail", "edition2024"}


def doc_comment(lib_rs: Path) -> str:
    lines = []
    for line in lib_rs.read_text().splitlines():
        stripped = line.strip()
        if stripped.startswith("//!"):
            lines.append(stripped[4:] if stripped.startswith("//! ") else stripped[3:])
        elif stripped.startswith("#!") or not stripped:
            continue
        else:
            break  # doc comment ends at the first item
    return "\n".join(lines)


def to_markdown(doc: str) -> str:
    out, in_code, keep = [], False, True
    for line in doc.splitlines():
        m = FENCE_RE.match(line.strip())
        if m and not in_code:
            in_code = True
            info = m.group(1).split(",")[0]
            keep = info in RUST_INFOSTRINGS or info.startswith("rust")
            out.append("```rust" if keep else line)
            continue
        if m and in_code:
            in_code = False
            out.append("```")
            continue
        if in_code and (line.startswith("# ") or line.strip() == "#"):
            continue  # hidden doctest line
        out.append(line)
    # demote doc headings so the crate name stays the only h1
    return "\n".join(
        ("#" + l if l.startswith("#") and not l.startswith("#!") else l) for l in out
    )


def main() -> None:
    for crate in sorted(Path("crates").iterdir()):
        if not crate.is_dir() or crate.name == FACADE_DIR:
            continue
        lib_rs = crate / "src" / "lib.rs"
        doc = doc_comment(lib_rs)
        if not doc:
            print(f"SKIP {crate.name}: no //! doc comment")
            continue
        name_m = re.search(r'^name\s*=\s*"(.*)"', (crate / "Cargo.toml").read_text(), re.M)
        readme = (
            HEADER.format(name=name_m.group(1))
            + to_markdown(doc)
            + "\n"
            + FOOTER
        )
        (crate / "README.md").write_text(readme)
        print(f"wrote {crate.name}/README.md ({len(readme)} bytes)")


if __name__ == "__main__":
    main()
