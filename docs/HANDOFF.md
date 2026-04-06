# Nav2++ Handoff Guide

This is the document to hand to the next person.

If they read only one file in this repo, it should be this one.

The goal is simple:

1. Get Nav2++ installed correctly on the target machine.
2. Confirm the machine is usable for Nav2.
3. Launch either the packaged demo or a live ROS graph without guessing.

This guide assumes the person using it is not the original author.

## What Nav2++ Is

Nav2++ is a repo-local bootstrap and verification wrapper around Nav2.

It helps with:

- installing or preparing the environment
- generating repeatable setup and start scripts
- checking whether a ROS graph is usable for Nav2
- launching the packaged TB3 demo
- launching a lighter live-graph overlay path

It does not magically replace the rest of a robot stack.

A real robot still needs:

- localization
- a correct TF tree
- sensors
- a command path to the vehicle controller
- robot-specific parameters and safety logic

## Before You Start

You need:

- this repo checked out locally
- `python3`
- a terminal

Go to the repo root first:

```bash
cd /path/to/Nav2++
```

Everything in this guide assumes commands are run from the repo root.

## Choose Your Machine Type

Start here.

Pick the section that matches the machine you are standing at.

### Linux vehicle computer or Linux robot computer with NVIDIA GPU

This is the most important case.

If this machine is the real runtime for the robot, start here first.

Use this path for:

- real robot bring-up
- vehicle bring-up
- DBW integration
- deployed Linux + NVIDIA target validation
- validating whether the ROS graph is actually usable

Run:

```bash
./nav2++ setup --yes
./nav2++ profile scaffold jeep
./nav2++ validate --profile vehicle
./nav2++ validate --profile nav2
./nav2++ validate --profile jeep
```

After validation passes, choose one:

- packaged demo:

```bash
./nav2++ start --yes
```

- live graph overlay:

```bash
./nav2++ start --mode live --yes
```

This is the machine that matters for real verification.

If the robot has exact non-generic topic names, do not stop at the built-in profile.

Create a robot-specific validation profile from the live graph first:

```bash
./nav2++ profile scaffold jeep
```

Then validate with it:

```bash
./nav2++ validate --profile jeep
```

Use `profiles/jeep.example.json` only as a fallback starting template when you cannot scaffold from a live graph.

The scaffolded profile and the example profile both include an NVIDIA host check via `nvidia-smi`.

### Linux workstation or Linux dev box

Use this if you are on Linux but not necessarily on the final robot computer.

Use this path for:

- development
- trying the packaged Nav2 demo
- checking a live ROS graph from a simulator or bench setup

Run:

```bash
./nav2++ setup --yes
```

Then either:

```bash
./nav2++ start --yes
```

or:

```bash
./nav2++ validate --profile vehicle
./nav2++ start --mode live --yes
```

### macOS laptop or desktop for demo and evaluation

Use this if you want a visible Nav2 session with Gazebo and RViz on a Mac.

Run:

```bash
./nav2++ setup --mode docker --yes
./nav2++ start --yes
```

What to expect:

- a Docker image will build
- a browser URL will print
- Gazebo and RViz will run inside a browser-served Linux desktop

Use this path for:

- evaluation
- demos
- learning the Nav2 UI
- quick packaged simulation

Do not use this path as your mental model for a production Linux + NVIDIA vehicle computer.

### macOS laptop or desktop while the real robot runs somewhere else

Do not treat macOS as the final runtime for the robot.

Use macOS for:

- running the demo path above
- reading docs
- viewing logs
- preparing the repo

Use the actual Linux robot computer for verification and runtime.

## The Three Commands To Know

These are the only commands most people need.

### `./nav2++ doctor`

Shows what Nav2++ thinks about the machine and which setup strategy it will use.

Use it when:

- you are unsure what path Nav2++ will choose
- you want to confirm whether you are on Linux-native, macOS-Docker, or macOS-Lima

### `./nav2++ setup`

Creates repo-local state and generates installer scripts.

The normal non-interactive form is:

```bash
./nav2++ setup --yes
```

On macOS, to force the browser-based path:

```bash
./nav2++ setup --mode docker --yes
```

### `./nav2++ validate`

Checks if the current ROS graph is good enough for vehicle or Nav2 use.

Recommended forms:

```bash
./nav2++ validate --profile vehicle
./nav2++ validate --profile nav2
```

### `./nav2++ start`

Starts either the packaged demo or a live overlay path.

Recommended forms:

```bash
./nav2++ start --yes
./nav2++ start --mode live --yes
```

## Recommended Workflows

## Workflow 1: “I just want to see Nav2 working”

### macOS demo machine

Run:

```bash
./nav2++ setup --mode docker --yes
./nav2++ start --yes
```

Success looks like:

- the terminal prints a browser URL like `http://127.0.0.1:6080/...`
- a browser window opens
- a Linux desktop appears in the browser
- Gazebo opens
- RViz opens

In RViz:

1. If the robot pose looks wrong, click `2D Pose Estimate` once and place the robot.
2. Click `Nav2 Goal` or `2D Nav Goal`.
3. Click a reachable place on the map and drag to set heading.

Do not use `Navigate Through Poses` unless you intentionally want multiple waypoints.

If you click that tool with no waypoints, planning will fail and it is not a setup problem.

### Linux machine

Run:

```bash
./nav2++ setup --yes
./nav2++ start --yes
```

Success looks like:

- Gazebo starts
- RViz starts
- the TurtleBot demo world appears

## Workflow 2: “I need to know if this ROS graph is usable for a real robot”

Run this on the Linux machine that can actually see the robot topics:

```bash
./nav2++ validate --profile vehicle
```

Then, if you want Nav2-specific readiness too:

```bash
./nav2++ validate --profile nav2
```

Interpretation:

- `Result: pass` means the required checks passed
- `WARN` means useful but not fatal information
- `FAIL` on a required check means the machine is not ready for that profile yet

The `vehicle` profile checks for:

- map
- odom
- tf
- tf_static
- cmd_vel
- at least one sensor input such as `/scan` or point cloud

The `nav2` profile adds live Nav2 expectations such as:

- frame checks
- TF edges
- `navigate_to_pose`

If this command fails, fix the graph first.

Do not go straight to “bring up Nav2 anyway” and hope for the best.

## Workflow 3: “I have a live ROS graph and want a quick overlay”

Run:

```bash
./nav2++ start --mode live --yes
```

Use this only when a robot or existing simulator is already publishing useful topics.

What this mode does:

- inspects the ROS graph
- decides whether enough of the graph exists
- stubs some missing basics if needed
- launches the lighter live-overlay path

What this mode is not:

- a full production robot bring-up
- a replacement for your robot-specific launch files

## What Success Looks Like

Use this checklist instead of guessing.

### Setup success

You should see:

- `.nav2pp/` created in the repo
- `.nav2pp/generated/` populated with scripts
- `.nav2pp/state/plan.json` written

### Validation success

You should see:

- `Result: pass`
- no required `FAIL` lines

### Demo start success

You should see:

- Gazebo
- RViz
- a map
- a robot
- successful navigation after placing a goal

### Live graph success

You should see:

- validation passes or only optional warnings remain
- RViz can see the map and robot frames
- goals can be sent without “action server is not available”

## The Most Important Rules

These rules prevent most wasted time.

### 1. Use the real Linux machine for real robot verification

If the final runtime is a vehicle or Linux box, do validation there.

Do not trust a Mac host as proof that the real robot graph is good.

### 2. Topic presence is not enough

A topic existing does not mean it is correct.

A graph can still fail Nav2 because of:

- wrong message type
- wrong frame IDs
- stale timestamps
- missing TF edges
- no working command path

That is why `./nav2++ validate` exists.

### 3. Start with the demo if you are unsure

If someone is new to this repo, the least risky path is:

```bash
./nav2++ setup --yes
./nav2++ start --yes
```

or on macOS:

```bash
./nav2++ setup --mode docker --yes
./nav2++ start --yes
```

Only move to live graph mode after the demo path is understood.

### 4. Use `Nav2 Goal`, not `Navigate Through Poses`, unless you mean it

`Navigate Through Poses` is for multiple waypoints.

For a basic sanity check, use one normal navigation goal.

## Copy-Paste Commands

This section is intentionally redundant.

### Fastest macOS demo path

```bash
cd /path/to/Nav2++
./nav2++ setup --mode docker --yes
./nav2++ start --yes
```

### Fastest Linux demo path

```bash
cd /path/to/Nav2++
./nav2++ setup --yes
./nav2++ start --yes
```

### Linux vehicle verification path

```bash
cd /path/to/Nav2++
./nav2++ setup --yes
./nav2++ validate --profile vehicle
./nav2++ validate --profile nav2
```

### Live overlay path

```bash
cd /path/to/Nav2++
./nav2++ validate --profile vehicle
./nav2++ start --mode live --yes
```

## Reading Validation Output

Example:

```text
Profile: vehicle
Result: pass (6/6 required checks passed)
Checks:
  - PASS map [required] /map [nav_msgs/msg/OccupancyGrid]
  - PASS odom [required] /odom [nav_msgs/msg/Odometry]
  - PASS tf [required] /tf [tf2_msgs/msg/TFMessage]
  - PASS tf_static [required] /tf_static [tf2_msgs/msg/TFMessage]
  - PASS cmd_vel [required] /cmd_vel [geometry_msgs/msg/Twist]
  - PASS sensor_input [required] using /scan [sensor_msgs/msg/LaserScan]
```

How to read it:

- `PASS` means good
- `WARN` means usable but incomplete
- `FAIL` on a required line means stop and fix that issue first

## Troubleshooting

## “ros2 is not installed or not on PATH”

Meaning:

- the current shell cannot query a live ROS graph directly

What to do:

- on macOS, this is expected on the host when using Docker or Lima
- run the packaged demo path anyway if that is your goal
- for real graph validation, run `validate` on the Linux machine that actually has ROS

## “No guest GUI display detected. Launching headless Nav2 sim.”

Meaning:

- Lima could not show Linux GUI apps on macOS

What to do:

- if you need a visible GUI on macOS, use Docker mode instead:

```bash
./nav2++ setup --mode docker --yes
./nav2++ start --yes
```

## Browser desktop comes up but you do not know what you are looking at

Focus only on:

- Gazebo or `gz sim`
- RViz

Ignore the desktop itself.

Gazebo is the simulator.

RViz is the navigation UI.

## RViz says `navigate_to_pose action server is not available`

Usually this means one of these:

- Nav2 did not finish coming up
- the nav container crashed
- the initial pose has not been set
- you restarted part of the stack and RViz is talking to stale state

What to do:

1. Restart the session cleanly.
2. If using the demo, wait until the stack settles.
3. Set `2D Pose Estimate` once if needed.
4. Use `Nav2 Goal`, not `Navigate Through Poses`.

## Validation passes but the real vehicle still does not move

That usually means the gap is below Nav2.

Common causes:

- `cmd_vel` is not bridged into the DBW interface
- safety interlocks are blocking motion
- localization exists but is not stable enough
- robot-specific Nav2 parameters are wrong

Nav2++ can prove the graph is plausible. It cannot replace your robot integration layer.

## When To Ask For Help

Ask for help only after collecting these exact things:

1. The machine type and OS.
2. The command you ran.
3. The full output from the first error onward.
4. The output of:

```bash
./nav2++ doctor
./nav2++ validate --profile vehicle
```

If this is a GUI issue on macOS, also say whether you used:

- `setup --mode docker`
- or the default Lima path

## Recommended Handoff Script For The Next Person

If you are handing this repo to someone else, tell them this:

1. Start with [README.md](../README.md).
2. Then follow this file exactly.
3. On macOS, use Docker mode unless you specifically want Lima.
4. On a real robot or vehicle computer, use Linux and run `validate` before trying to drive anything.
5. If a required validation check fails, do not proceed until it is fixed.

## Final Summary

For a visible demo:

- macOS: Docker mode
- Linux: native mode

For a real vehicle:

- run on Linux
- use `./nav2++ validate --profile vehicle`
- use `./nav2++ validate --profile nav2`
- only then move to live Nav2 work

If the next person follows this document literally, they should not need repo archaeology to get started.
