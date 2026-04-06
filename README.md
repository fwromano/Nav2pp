# Nav2++

Nav2++ is a repo-local bootstrap layer for bringing up Nav2 on a fresh machine without spraying config across the host.

If you are handing this repo to another person, start them with [HANDOFF.md](docs/HANDOFF.md).
If something goes wrong on the happy path, use [TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md).

The intended flow is:

1. Put this repo somewhere sensible.
2. Run `./nav2++ setup`.
3. Run `./nav2++ start`.

That is the intended two-command path to a usable Nav2 session on a fresh machine.

## Where the repo should live

The repo can live almost anywhere, but location still matters:

- Best: a local SSD path under your home directory, for example `~/dev/Nav2++`
- Acceptable: another local disk
- Works for evaluation: a removable flash drive

Builds and VM host mounts are materially smoother from a local SSD. On macOS, the recommended Lima workflow is easiest when the repo lives under `$HOME`.
If the host-mounted repo is read-only inside the Lima guest, Nav2++ falls back to `$HOME/nav2pp` inside the guest for its writable state and workspace.

## Commands

Inspect the current host and selected strategy:

```bash
./nav2++ doctor
```

Generate local state and installer scripts without running system package installs:

```bash
./nav2++ setup --report-only
```

Run the full setup flow non-interactively:

```bash
./nav2++ setup --yes
```

On macOS, if you want a browser-visible Linux desktop instead of the Lima guest path:

```bash
./nav2++ setup --mode docker --yes
```

Start the default Nav2 session:

```bash
./nav2++ start
```

With the Docker mode on macOS, `start` prints and opens a local browser URL for the container desktop:

`http://127.0.0.1:6080/vnc.html?autoconnect=1&resize=scale`

`./nav2++ start` also rebuilds the local Docker image so container-side launch fixes are picked up without rerunning setup.

If a live ROS graph already exists and you want to reuse it instead of launching the TB3 demo:

```bash
./nav2++ start --mode live
```

Infer common Nav2 topics from a live ROS graph:

```bash
./nav2++ topics
```

Or from a saved topic list:

```bash
./nav2++ topics --from-file topics.txt --json
```

Verify that a live ROS graph is usable for a vehicle/Nav2 bring-up:

```bash
./nav2++ validate --profile vehicle
```

Or verify a saved topic snapshot without needing a live `ros2` environment on the current shell:

```bash
./nav2++ validate --profile nav2 --from-file topics.txt
```

If you need exact robot-specific checks, create `profiles/<name>.json` or point to a file directly:

```bash
./nav2++ validate --profile jeep
./nav2++ validate --profile-file profiles/jeep.json
```

An example custom profile lives at `profiles/jeep.example.json`.
That example assumes the real deployed target is Linux with an NVIDIA GPU and checks `nvidia-smi` when run live on the machine.

To generate a first draft from the current ROS graph on the target Linux machine:

```bash
./nav2++ profile scaffold jeep
./nav2++ validate --profile jeep
```

## What setup creates

`./nav2++ setup` creates:

- `.nav2pp/env/nav2pp.env` for repo-local environment activation
- `.nav2pp/state/host.json` with machine diagnostics
- `.nav2pp/state/plan.json` with the selected setup strategy
- `.nav2pp/generated/*.sh` installer scripts
- `.nav2pp/state/start-plan.json` after `./nav2++ start`
- `workspace/src` for imported Nav2 sources and overlays
- `manifests/nav2.repos` for fetching upstream Nav2 with `vcs`

The activation script keeps the local Python venv, logs, and colcon state inside the repo:

```bash
source .nav2pp/env/nav2pp.env
```

## Strategy selection

`nav2++` chooses a setup strategy from the detected machine:

- Linux `ubuntu` or `debian`: native ROS install via `apt`
- Real deployment target: Linux on the vehicle computer, with optional robot-specific validation profiles such as `profiles/jeep.json`
- macOS: Lima-managed Ubuntu guest by default
- macOS `--mode docker`: Colima-backed Docker runtime with a browser-served Linux desktop
- macOS `--mode native`: diagnostic-only today, because native Nav2 automation is still experimental
- Raspberry Pi class ARM Linux: native install with a lighter profile

`nav2++ start` chooses a run strategy too:

- Default: launch the documented Nav2 TB3 simulation with Gazebo and RViz
- If a usable live graph is already visible: reuse it, stub missing core topics, and launch RViz
- On macOS: run the selected start path inside the Lima guest
- On macOS without guest GUI forwarding: fall back to headless TB3 sim and seed AMCL with the default initial pose automatically
- On macOS with Docker mode: run the TB3 demo inside a container and expose Gazebo/RViz through noVNC in a browser

## Current limits

- The default `start` path is the official Nav2 demo path. The live-overlay path is intentionally lighter weight and currently focuses on RViz plus stubbed core topics rather than a full robot-specific bringup.
- Topic discovery uses common heuristics and should be treated as a starting point, not final truth.
- `nav2++ validate` is currently focused on core bring-up readiness: topic presence, message types, basic frame IDs, required TF edges, and the `navigate_to_pose` action for the `nav2` profile.
- `nav2++ validate` can also load an exact JSON profile for a specific robot stack, including host checks such as `nvidia-smi`, but you still need to define the real topic names, frame expectations, and command path for that robot.
- `nav2++ profile scaffold <name>` can generate a first draft from the live graph, but you should still review the resulting profile before treating it as final.
- macOS support is intentionally VM-first because that is the practical path for Nav2.
- Headless macOS startup will run the sim and Nav2 stack, but it still does not provide a local GUI unless you add guest display forwarding such as XQuartz/X11.
- The Docker mode is currently aimed at the packaged demo path on macOS. Live host-graph overlay is still Lima-first because DDS across host/container boundaries is a separate problem.
- The browser desktop launch trims the Nav2 stack to the essential demo nodes on macOS Docker so route, docking, and following extras do not block the basic sim path.
