# Contributing

## Workflow
1. Fork repository
2. Create feature branch: `git checkout -b feature/<short_topic>`
3. Make changes (code + docs)
4. Ensure build succeeds: `./scripts/build_nodes.sh`
5. Run minimal simulation or example
6. Commit with conventional prefix (feat, fix, docs, chore, refactor)
7. Push & open Pull Request

## Commit Message Examples
```
feat: add offline deployment packaging scripts
fix: normalize python script line endings
chore: update README with documentation index
```

## Code Style
* C++: auto-format with repository `.clang-format`
* Python: Keep scripts executable; prefer f-strings; avoid print spam
* Shell: `set -euo pipefail` for new scripts; POSIX sh where possible

## Adding Messages
1. Place file under `interface_protocol/msg`
2. Rebuild workspace
3. Update protocol README table if user-facing

## Review Checklist
| Item | Consideration |
| --- | --- |
| Build | `colcon build` clean |
| Runtime | Example runs without error |
| Docs | Updated if behavior / usage changes |
| Safety | No regressions to FSM or control modes |
| Portability | Works with offline deployment if relevant |

## Large Changes
Open an issue first describing motivation, design sketch, impact on existing nodes.

## Releases
Tagging strategy TBD; propose semantic-like tags after initial stabilization.

## License
By contributing you agree your work is released under the repository license.
