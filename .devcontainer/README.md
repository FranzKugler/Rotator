# Rotator development container

This environment pins ESP-IDF 5.4.1 by immutable image digest and Claude Code
2.1.247. Run it through `scripts/dev-container.sh` from anywhere inside the
repository.

State is separated as follows:

- repository: bind-mounted at `/workspace`;
- compiler cache: named `idf-cache` volume;
- npm cache: named `npm-cache` volume;
- Claude Code account and sessions: named `claude-state` volume.

The service runs as a non-root user matching Franz's host UID/GID. It has all
Linux capabilities dropped, `no-new-privileges`, no Docker socket, no host-home
mount, no published ports and no hardware devices.

Authenticate Claude independently inside the container with
`scripts/dev-container.sh claude`. Credentials stay in the private named volume
and are neither baked into the image nor committed to Git.

USB flashing is intentionally separate. Add only a stable
`/dev/serial/by-id/...` device in a local Compose override after the target is
attached; never enable privileged mode or mount all of `/dev`.
