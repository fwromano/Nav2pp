# Nav2++ Troubleshooting

Use [HANDOFF.md](HANDOFF.md) first.

Use this file only when the happy path did not work.

## Quick Triage

Run these first:

```bash
./nav2++ doctor
./nav2++ validate --profile vehicle
```

If you are on a real Nav2 graph, also run:

```bash
./nav2++ validate --profile nav2
```

## Common Problems

## Setup fails on macOS Lima with write errors

Symptom:

- guest says the mounted repo is read-only

Meaning:

- the Lima guest cannot write back into the host-mounted repo path

Expected behavior now:

- Nav2++ should fall back to `$HOME/nav2pp` inside the guest automatically

If that fallback message appears, that part is working.

## Gazebo or RViz does not appear on macOS Lima

Meaning:

- guest GUI forwarding is unavailable

Use Docker mode instead:

```bash
./nav2++ setup --mode docker --yes
./nav2++ start --yes
```

## Browser desktop appears but navigation is dead

Check:

- did RViz fully load
- did Gazebo fully load
- did you set an initial pose if needed
- did you use `Nav2 Goal` instead of `Navigate Through Poses`

## `Result: fail` from validation

Do not continue to start live navigation until the required failures are gone.

Fix the failing items in this order:

1. `map`
2. `odom`
3. `tf`
4. `tf_static`
5. `sensor_input`
6. `cmd_vel`
7. TF edge checks
8. action checks

## `nvidia_gpu` fails in a robot-specific profile

Meaning:

- the deployed machine does not expose `nvidia-smi`
- the NVIDIA driver/runtime is not installed correctly
- or you are validating on the wrong machine

What to do:

- run the validation on the actual Linux vehicle computer
- confirm `nvidia-smi` works in a normal shell
- if your deployed stack does not actually require GPU access, remove that host check from the robot profile

## `profile scaffold` generates a weak or incomplete profile

Meaning:

- the live graph did not expose enough exact robot topics yet
- or the DBW topic names were too custom to infer safely

What to do:

- rerun it on the real Linux vehicle computer while the full stack is publishing
- inspect `profiles/jeep.json`
- compare it against `profiles/jeep.example.json`
- fix the final ambiguous fields manually once, then keep that file in the repo

## What To Send With A Bug Report

Send:

```bash
./nav2++ doctor
./nav2++ validate --profile vehicle
./nav2++ validate --profile nav2
```

Plus:

- machine type
- OS
- whether you used Linux, Docker, or Lima
- the first failing command
- the log from the first error onward
