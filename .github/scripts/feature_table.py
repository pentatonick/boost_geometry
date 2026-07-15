#!/usr/bin/env python3
"""Regenerate the feature table in the workspace-root README.md between
`<!-- feature-table:start/end -->` markers from `feature-*:` tags on the
`pub use` lines of each crate's lib.rs.

The table's one non-derivable fact — which capability group a function belongs
to — is written as a comment tag next to the export it describes, so it cannot
drift from the code:

    // feature-group: Measures
    // feature-desc: Scalar quantities of a geometry
    pub use area::{area, area_with, box_area};

Grammar (comment lines immediately above a `pub use`):
  // feature-group: <Name>   required — this pub use becomes one row under <Name>
  // feature-desc: <text>     optional — group prose; continue with `//   <more>`
  // feature-keep: <Ident>    optional — surface a PascalCase type that would
                              otherwise be dropped (types are fn-table noise by default)

Group DISPLAY ORDER is not a tag — it is presentation and lives in the
`GROUP_ORDER` map in this script. An unranked group sorts last, alphabetically.

Everything else is derived:
  * function names        — parsed from the tagged `pub use path::{a, b, c}` block
  * no_std status         — read from the already-rendered no-std table in README
                            (keyed by the crate the export lives in) — no rebuild
  * docs.rs link          — facade path if the crate is re-exported by the
                            `boost_geometry` facade, else the leaf crate's page

A `pub use` that exports a snake_case free function but carries no
`feature-group` tag, in a crate that participates in the table, is a hard
error: it forces the author to classify every new algorithm.

Usage: python3 .github/scripts/feature_table.py
Run from the workspace root. CI runs this then `git diff --exit-code`.
"""

import re
from pathlib import Path

# Cargo.toml is parsed with regex (not tomllib) so this runs on the pre-commit
# hook's Python, which may predate 3.11 — matching crate_readme.py's approach.

ROOT = Path(__file__).resolve().parent.parent.parent
README = ROOT / "README.md"
START = "<!-- feature-table:start -->"
END = "<!-- feature-table:end -->"
DOCS = "https://docs.rs"
FACADE = "geometry"  # crate dir of the boost_geometry facade

# Group display order — presentation, so it lives here, not scattered across
# lib.rs files. A group named by a `feature-group` tag but absent from this
# map sorts last, alphabetically (rank it here when you care where it lands).
# The group's NAME and DESCRIPTION stay in lib.rs, next to the code they
# describe; only the ordering is editorial.
GROUP_ORDER = {
    "Measures": 1,
    "Spatial predicates": 2,
    "Boolean operations": 3,
    "Construction & transformation": 4,
    "Inspection": 5,
    "Mutation & assembly": 6,
    "Spatial index": 7,
    "I/O — Well-Known Text": 8,
    "I/O — Well-Known Binary": 9,
    "I/O — GeoJSON": 10,
    "I/O — SVG": 11,
    "Reprojection": 12,
}

# A crate "participates" in the table iff its lib.rs contains at least one
# feature-group tag — so scope is auto-discovered, not configured here.


def workspace_crate_dirs():
    text = (ROOT / "Cargo.toml").read_text()
    members = re.search(r"members\s*=\s*\[(.*?)\]", text, re.S)
    if not members:
        raise SystemExit("Cargo.toml: no workspace members array")
    return [ROOT / m for m in re.findall(r'"([^"]+)"', members.group(1))]


def crate_name(crate_dir: Path) -> str:
    m = re.search(r'^name\s*=\s*"(.*)"', (crate_dir / "Cargo.toml").read_text(), re.M)
    return m.group(1)


def facade_reexports() -> dict[str, str]:
    """Map crate-name -> facade module (e.g. geometry-algorithm -> "algorithm")
    for every crate the facade re-exports via `pub use geometry_x::*` inside a
    `pub mod <module>`. Used to decide whether a fn gets a docs.rs facade path."""
    text = (ROOT / "crates" / FACADE / "src" / "lib.rs").read_text()
    out = {}
    # pub mod algorithm {  pub use geometry_algorithm::*;
    for mod_name, dep in re.findall(
        r"pub mod (\w+)\s*\{[^}]*?pub use (geometry_\w+)::\*", text, re.S
    ):
        out[dep.replace("_", "-")] = mod_name
    return out


def no_std_map() -> dict[str, bool]:
    """crate-name -> no_std, read from the rendered no-std table in README."""
    text = README.read_text()
    out = {}
    for name, mark in re.findall(r"\|\s*`([\w-]+)`\s*\|\s*([^\s|]+)\s*\|", text):
        if mark in ("✅", "❌"):
            out[name] = mark == "✅"
    return out


TAG_RE = re.compile(r"//\s*feature-(group|desc|keep):\s*(.*)")
DESC_CONT_RE = re.compile(r"//\s{2,}(\S.*)")
PUB_USE_RE = re.compile(r"pub use (?:\w+::)?\{?([^;{}]*)\}?;?\s*$")


def parse_crate(lib_rs: Path):
    """Yield (group, desc, [idents]) for each tagged pub use, plus the untagged
    fn-exporting pub use lines (for the classify-everything error check).
    Group ORDER is not read here — it is presentation, held in GROUP_ORDER."""
    lines = lib_rs.read_text().splitlines()
    rows, untagged_fns = [], []
    i = 0
    while i < len(lines):
        line = lines[i]
        m = TAG_RE.search(line)
        if not m:
            # untagged pub use that exports a snake_case fn?
            if line.strip().startswith("pub use ") and _has_fn(_join_use(lines, i)[0]):
                untagged_fns.append((i + 1, line.strip()))
            i += 1
            continue
        # collect a tag block
        group = None
        desc_parts = []
        keep = set()
        while i < len(lines):
            tm = TAG_RE.search(lines[i])
            if tm:
                kind, val = tm.group(1), tm.group(2).strip()
                if kind == "group":
                    group = val
                elif kind == "desc":
                    desc_parts.append(val)
                elif kind == "keep":
                    keep.add(val)
                i += 1
                continue
            cont = DESC_CONT_RE.search(lines[i])
            if cont and desc_parts:  # wrapped feature-desc continuation
                desc_parts.append(cont.group(1).strip())
                i += 1
                continue
            break
        # the next non-blank line must be the pub use this block annotates
        while i < len(lines) and not lines[i].strip():
            i += 1
        if i >= len(lines) or not lines[i].strip().startswith("pub use "):
            raise SystemExit(
                f"{lib_rs}:{i}: feature-group tag not followed by a `pub use`"
            )
        block, i = _join_use(lines, i)
        idents = _idents(block, keep)
        if idents:
            rows.append(
                {
                    "group": group,
                    "desc": " ".join(desc_parts),
                    "idents": idents,
                }
            )
    return rows, untagged_fns


def _join_use(lines, i):
    """Join a possibly multi-line `pub use ... { ... };` into one string;
    return (joined, index_after)."""
    buf = lines[i]
    j = i
    while ";" not in buf and j + 1 < len(lines):
        j += 1
        buf += " " + lines[j].strip()
    return buf, j + 1


def _names(block: str):
    m = PUB_USE_RE.search(block.strip())
    if not m:
        return []
    out = []
    for raw in m.group(1).split(","):
        n = raw.strip()
        if not n:
            continue
        # `path as alias` — the alias is the public name
        if " as " in n:
            n = n.split(" as ")[-1].strip()
        out.append(n.removeprefix("r#"))
    return out


def _has_fn(block: str) -> bool:
    return any(n[:1].islower() for n in _names(block))


def _idents(block: str, keep: set[str]):
    """snake_case fns always; PascalCase only if explicitly kept."""
    out = []
    for n in _names(block):
        if n[:1].islower() or n in keep:
            out.append(n)
    return out


def docs_link(crate: str, first_ident: str, facade: dict[str, str]) -> str:
    if crate in facade:
        mod = facade[crate]
        kind = "fn" if first_ident[:1].islower() else "struct"
        return f"{DOCS}/boost_geometry/latest/boost_geometry/{mod}/{kind}.{first_ident}.html"
    return f"{DOCS}/{crate}"


def render(groups) -> str:
    out = ["| Function | `no_std` | Docs |", "|---|:---:|---|"]
    for g in groups:
        header = f"**{g['name']}**"
        if g["desc"]:
            header += f" — {g['desc']}"
        out.append(f"| {header} |||")
        for row in g["rows"]:
            fns = " / ".join(f"`{n}`" for n in row["idents"])
            mark = "✅" if row["no_std"] else "❌"
            out.append(f"| {fns} | {mark} | [→]({row['link']}) |")
    return "\n".join(out)


def main():
    facade = facade_reexports()
    nostd = no_std_map()
    group_desc = {}  # name -> description (first non-empty wins)
    group_rows = {}  # name -> list of rows
    errors = []

    for crate_dir in workspace_crate_dirs():
        lib_rs = crate_dir / "src" / "lib.rs"
        if not lib_rs.exists():
            continue
        name = crate_name(crate_dir)
        rows, untagged = parse_crate(lib_rs)
        if not rows:
            continue  # crate does not participate
        for line_no, src in untagged:
            errors.append(f"{lib_rs}:{line_no}: untagged fn export `{src}` "
                          f"(add `// feature-group: <Name>`)")
        for r in rows:
            g = r["group"]
            if r["desc"] and not group_desc.get(g):
                group_desc[g] = r["desc"]
            group_rows.setdefault(g, []).append({
                "idents": r["idents"],
                "no_std": nostd.get(name, False),
                "link": docs_link(name, r["idents"][0], facade),
            })

    if errors:
        raise SystemExit("feature-table: unclassified exports:\n  " + "\n  ".join(errors))

    # Group sequence is editorial (GROUP_ORDER); unranked groups sort last,
    # alphabetically. Rows within a group are always alphabetical by first ident.
    ordered = sorted(
        group_rows,
        key=lambda g: (GROUP_ORDER.get(g, len(GROUP_ORDER) + 1), g),
    )
    groups = [
        {
            "name": g,
            "desc": group_desc.get(g, ""),
            "rows": sorted(group_rows[g], key=lambda r: r["idents"][0].lower()),
        }
        for g in ordered
    ]
    block = f"{START}\n{render(groups)}\n{END}"

    text = README.read_text()
    pattern = re.compile(re.escape(START) + r".*?" + re.escape(END), re.DOTALL)
    if not pattern.search(text):
        raise SystemExit(f"README.md is missing {START} / {END} markers")
    README.write_text(pattern.sub(lambda _: block, text))
    print(f"wrote feature table: {len(groups)} groups, "
          f"{sum(len(g['rows']) for g in groups)} rows")


if __name__ == "__main__":
    main()
