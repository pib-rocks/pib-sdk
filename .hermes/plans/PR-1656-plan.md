# PR-1656: pib-sdk release-drafter release notes + release tag as version source (Weg 2: tag->build via CI)

Ticket: https://pib-rocks.atlassian.net/browse/PR-1656
Repo: pib-rocks/pib-sdk — local at /home/pib/opencode/pib-sdk, branch `PR-1656` (from origin/main `bf63cad`).
Scope: implement Weg 2 — release-drafter (release notes) + a CI publish workflow that injects the
git-tag version into the build, so `pip install pib-sdk` reports a version matching the GitHub release tag.
Do NOT merge to develop (pib-sdk has no develop — trunk IS main). Do NOT push.

## Goal
1. Automated, categorized release notes from PR labels (release-drafter), built from `main` history.
2. A single version source = the GitHub release tag. On release/tag, CI resolves the version from the
   tag, injects it into `pyproject.toml` (and `src/pib_sdk/__init__.py` `__version__`) before building,
   and publishes the sdist+wheel to PyPI. Static version stays (NO setuptools-scm / dynamic version).

## Verified facts
- pib-sdk trunk is `main` (git branches are `main`, `origin/main`); there is NO `develop`.
- `pyproject.toml` line 7: `version = "0.4"`. `src/pib_sdk/__init__.py` line 57: `__version__ = "0.4"`
  (exported in `__all__`). Build: `python -m build` (setuptools static version). Tests: `PYTHONPATH=src python3 -m pytest tests/ -q`.
- `.github` does not exist yet in pib-sdk.
- cerebra/pib-backend release-drafter pattern (adapt to main, not develop):
  - `.github/workflows/release-drafter.yml`: `on: push: branches: [DEVELOP→main for pib-sdk]` + `pull_request_target: types: [opened, reopened, labeled, unlabeled]`, permissions `contents: read`/`pull-requests: write`, job uses `release-drafter/release-drafter@v6` with `commitish: refs/heads/main` and `config-name: release-drafter.yml`, `env: GITHUB_TOKEN`.
  - `.github/release-drafter.yml`: `name-template / tag-template 'v$RESOLVED_VERSION'`, categories
    (Features feature/enhancement, Testing testing, Bug Fixes bug/fix, Documentation documentation,
    Refactoring refactor, Maintenance maintenance/dependencies), `exclude-labels` (duplicate/wontfix/question),
    `version-resolver` (major: major; minor: feature/enhancement; patch: bug/fix/testing/documentation/refactor/maintenance/dependencies; default patch), template `## What's Changed\n\n$CHANGES`.
- cerebra/pib-backend publish/pairing uses `on: release: types: [published]`.

## Work items (single coherent change; can be one cursor-agent run, but split into 2 small commits you make on the branch)

### A. Release-drafter (release notes automation)
Create:
- `.github/workflows/release-drafter.yml` (adapted: `push: branches: [main]`, `commitish: refs/heads/main`).
- `.github/release-drafter.yml` (category/label config) — copy the cerebra/pib-backend structure verbatim (same labels).

### B. Version single-source + PyPI publish workflow (Weg 2)
1. Create a publish workflow, e.g. `.github/workflows/publish.yml`, triggered on `release: types: [published]`
   (and optionally `push: tags: ['v*']` as an alternative trigger). It must:
   - Check out the tag.
   - Resolve the version from the tag: strip leading `v` (`VERSION=${GITHUB_REF_NAME#v}`; for release event use
     `${{ github.event.release.tag_name }}`), e.g. tag `v0.5.0` -> version `0.5.0`.
   - Inject it into `pyproject.toml` (`version = "..."`) AND into `src/pib_sdk/__init__.py` (`__version__ = "..."`)
     BEFORE building (sed is fine; commit is NOT needed — publish builds from the working tree).
   - Build sdist + wheel (`python -m build`).
   - Publish to PyPI via `pypa/gh-action-pypi-publish@release/v1` using a token:
     `password: ${{ secrets.PYPI_TOKEN }}` (and optionally `repository-url: https://test.pypi.org/legacy/` with
     `secrets.TEST_PYPI_TOKEN` for a dry-run stage job).
   - Set `on: release: types: [published]` matching how the other repos publish.

## Hard constraints
1. Stay on branch `PR-1656`. Do NOT merge to develop (no develop). Do NOT push.
2. Do NOT use setuptools-scm / dynamic version. Keep `pyproject.toml` static; version injection happens in CI before build.
3. Do NOT modify existing functionality, tests, `src/pib_sdk/**` logic, or README unless strictly needed.
   Only touch: create `.github/workflows/*.yml`, `.github/release-drafter.yml`. The version-injection target
   (`pyproject.toml` + `__init__.py __version__`) is changed at publish time by the workflow, not by this branch's code.
4. `config-name: release-drafter.yml` in the workflow must match the config file name.
5. Do NOT commit `.hermes/plans/**`.

## Verify
- `ls .github/workflows/` shows release-drafter.yml (and publish.yml), `ls .github/release-drafter.yml` exists.
- YAML is valid (no obvious syntax error — a quick `python3 -c "import yaml,sys; [yaml.safe_load(open(f)) for f in ['...']]"` if pyyaml present).
- `PYTHONPATH=src python3 -m pytest tests/ -q` still green (no code changes).
- `python3 -c "import sys; sys.path.insert(0,'src'); import pib_sdk; print(pib_sdk.__version__)"` still `0.4` (this branch does not change the current version; CI changes it at publish only).
- No merge/push (origin/main sha unchanged). Branch = PR-1656.