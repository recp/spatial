# spatial spec

Normative specification for the spatial shared kernel. Implementations
must conform to these documents to claim spatial compatibility.

## Documents

- [pose.md](pose.md) — canonical pose and transform types, memory layout, invariants
- [handle.md](handle.md) — generational node handle, validation, lifetime
- [2d.md](2d.md) — 2D variant for Box2D-style engines
- [authority.md](authority.md) — who writes and reads what, flag semantics
- [update.md](update.md) — `spatial_update()` algorithm, dirty propagation, versioning
- [hotpath.md](hotpath.md) — pointer caching and zero-copy accessors for physics / render hot loops
- [parallel.md](parallel.md) — thread pool dispatch model, MT writer contract, threshold semantics

Related specs maintained elsewhere:

- `assetio/docs/assetkit-spatial-bridge.md` — AssetKit → spatial import bridge

## Status

Draft. The spec tracks [DESIGN.md](../DESIGN.md). Changes to either must
stay in sync.

## Conformance terms

- **MUST** — required for conformance
- **SHOULD** — recommended; deviation requires a documented reason
- **MAY** — optional, implementation choice
