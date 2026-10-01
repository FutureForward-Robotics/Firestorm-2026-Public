# Public Mirror

A nightly workflow copies this private repository to
[Firestorm-2026-Public](https://github.com/FutureForward-Robotics/Firestorm-2026-Public)
without the authors, so other teams can read the code.

## What gets removed

Git history, which is where the names are. The workflow builds a directory from
the current commit, makes a new repository out of it, commits once as
`Firestorm-bot`, and force pushes over the mirror's `main`. The mirror holds one
commit, matching `master`.

Also removed:

- The paths in [`.github/mirror-exclude.txt`](../.github/mirror-exclude.txt).
  Currently the mirror workflow and script. Add a path there to keep something
  private.
- Untracked and gitignored files. The snapshot comes from `git archive`, so
  `local.properties`, build output, and pulled run logs cannot leak.

Before pushing, the workflow greps the snapshot for email addresses, javadoc
author tags, and absolute home directory paths. A match fails the run without
publishing. The patterns are `DENY_PATTERNS` in `scripts/mirror-public.sh`; on a
false positive, exclude the file or edit the pattern.

For specific words, such as a nickname in a comment, put an extended regular
expression in the `MIRROR_DENY_REGEX` secret. Keep it in the secret, since a
file listing names would be mirrored too.

## Setup

Done for 2026. Next season, with the [GitHub CLI](https://cli.github.com) and
admin on the organization:

```
gh repo create FutureForward-Robotics/<Team>-<Year>-Public --public \
  -d "Public mirror of <Team>'s <Year> robot code"

ssh-keygen -t ed25519 -N "" -C "<team>-<year>-mirror" -f mirror-key
gh repo deploy-key add mirror-key.pub -R FutureForward-Robotics/<Team>-<Year>-Public \
  -t "mirror-public (write)" --allow-write
gh secret set MIRROR_DEPLOY_KEY -R FutureForward-Robotics/<Team>-<Year> < mirror-key
rm mirror-key mirror-key.pub
```

The deploy key writes to that one repository and does not expire. `gh` cannot
create a personal access token, because GitHub has no API for issuing one.

Then point the `TARGET_*` values in
[`.github/workflows/mirror-public.yml`](../.github/workflows/mirror-public.yml)
at the new repository.

To stop publishing, delete the deploy key:

```
gh repo deploy-key list -R FutureForward-Robotics/<Team>-<Year>-Public
gh repo deploy-key delete <id> -R FutureForward-Robotics/<Team>-<Year>-Public
```

## Running it by hand

*Actions, mirror-public, Run workflow.* The job checks out `master` whichever
branch you dispatch from, and exits without pushing when the snapshot matches
what is already public.

To build and check a snapshot without publishing:

```
TARGET_URL=https://github.com/FutureForward-Robotics/Firestorm-2026-Public.git \
  DRY_RUN=1 scripts/mirror-public.sh
```

The snapshot comes from `HEAD`, so uncommitted work is not included.
