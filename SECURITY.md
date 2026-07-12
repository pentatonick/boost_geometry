# Security Policy

## Reporting a vulnerability

Please report suspected vulnerabilities privately via
[GitHub Security Advisories](https://github.com/pentatonick/boost_geometry/security/advisories/new)
rather than opening a public issue.

You should receive a response within a week. Once the report is
confirmed, a fix will be developed privately and released with an
advisory crediting the reporter (unless you prefer otherwise).

## Scope

All crates published from this repository (`boost_geometry` and the
`geometry-*` crates). The workspace forbids `unsafe` code entirely, so
memory-safety issues are expected to originate in dependencies —
reports for those should go upstream, but pointers here are still
appreciated so we can bump the affected dependency.
