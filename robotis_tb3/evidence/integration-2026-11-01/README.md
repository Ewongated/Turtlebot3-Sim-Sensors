# Integrated validation evidence, 1 November 2026

See [integration write-up](../../INTEGRATION_STATUS.md).

- `before/`: eight trials on Ethan_PartD, Q x10/R x1, LQR + path compensation, entry 0.4 rad.
- `final/`: eight trials via the repaired controller batch CLI on merged code, same settings except entry 0.8 rad.
- `threshold-06-targeted-result.json`: a single diagnostic run, not a full batch.
- `threshold-comparison.png`: mean completion time and estimation RMSE tradeoff.
- `provenance.json`: scope and reproduction command; per-run source snapshots and manifests identify exact tested source.

Both batches have 8/8 successes and no collisions/timeouts. Keep these separate
from the original baseline, Part C replay study, and Part D's 80-trial comparison.
Traces are preserved byte-for-byte in gzip, with uncompressed SHA-256 hashes.
Output paths in result/summary metadata were made repository-relative. Cache
signatures exclude output paths, so this does not alter the signed runtime settings.
Expand traces using the repository's unpack_evidence.py before offline analysis.
