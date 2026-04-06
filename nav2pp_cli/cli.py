from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from . import __version__
from .bootstrap import run_setup
from .diagnostics import collect_host_info
from .planner import build_setup_plan
from .profile_scaffold import scaffold_profile
from .starter import run_start
from .topics import discover_topics, render_topic_report
from .validator import render_validation_report, validate_profile


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="nav2++", description="Bootstrap and diagnose Nav2++ hosts.")
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")

    subparsers = parser.add_subparsers(dest="command", required=True)

    doctor = subparsers.add_parser("doctor", help="Inspect the current machine and print the selected setup plan.")
    doctor.add_argument("--mode", default="auto", choices=["auto", "native", "lima", "docker"])
    doctor.add_argument("--json", action="store_true", help="Emit machine info and the setup plan as JSON.")
    doctor.set_defaults(func=cmd_doctor)

    setup = subparsers.add_parser("setup", help="Prepare repo-local state and generate installer scripts.")
    setup.add_argument("--mode", default="auto", choices=["auto", "native", "lima", "docker"])
    setup.add_argument("--report-only", action="store_true", help="Generate state and scripts but do not run installers.")
    setup.add_argument("--yes", action="store_true", help="Run the generated installer without prompting.")
    setup.set_defaults(func=cmd_setup)

    topics = subparsers.add_parser("topics", help="Infer common Nav2 topic inputs from a live ROS graph or a file.")
    topics.add_argument("--json", action="store_true", help="Emit matches as JSON.")
    topics.add_argument("--from-file", type=Path, help="Read topic names from a file instead of running ros2 topic list.")
    topics.add_argument("--topic", action="append", dest="topics", help="Provide topic names directly.")
    topics.set_defaults(func=cmd_topics)

    validate = subparsers.add_parser("validate", help="Verify whether the current ROS graph is usable for Nav2 or a vehicle bring-up.")
    validate.add_argument(
        "--profile",
        default="vehicle",
        help="Built-in profile name or local profile name under profiles/<name>.json. Defaults to vehicle.",
    )
    validate.add_argument(
        "--profile-file",
        type=Path,
        help="Load an exact validation profile from a JSON file.",
    )
    validate.add_argument("--json", action="store_true", help="Emit the validation report as JSON.")
    validate.add_argument("--from-file", type=Path, help="Read a topic snapshot from a file instead of querying ros2.")
    validate.add_argument("--topic", action="append", dest="topics", help="Provide topic entries directly, optionally as /name:type.")
    validate.set_defaults(func=cmd_validate)

    profile = subparsers.add_parser("profile", help="Create or manage robot-specific validation profiles.")
    profile_subparsers = profile.add_subparsers(dest="profile_command", required=True)

    scaffold = profile_subparsers.add_parser("scaffold", help="Generate a draft robot validation profile from a live graph or topic snapshot.")
    scaffold.add_argument("name", help="Profile name, for example jeep.")
    scaffold.add_argument("--from-file", type=Path, help="Read typed topic entries from a file instead of querying ros2.")
    scaffold.add_argument("--topic", action="append", dest="topics", help="Provide typed topic entries directly, optionally as /name:type.")
    scaffold.add_argument("--output", type=Path, help="Write the generated profile to a specific path.")
    scaffold.add_argument("--skip-nvidia", action="store_true", help="Do not require nvidia-smi in the generated profile.")
    scaffold.add_argument("--force", action="store_true", help="Overwrite the destination file if it already exists.")
    scaffold.set_defaults(func=cmd_profile_scaffold)

    start = subparsers.add_parser("start", help="Start a Nav2 demo or a live RViz overlay from the detected graph.")
    start.add_argument("--mode", default="auto", choices=["auto", "sim", "live"])
    start.add_argument("--dry-run", action="store_true", help="Generate the start plan and scripts without executing them.")
    start.add_argument("--yes", action="store_true", help="Run the generated start script without prompting.")
    start.add_argument("--from-file", type=Path, help="Read topic names from a file instead of running ros2 topic list.")
    start.add_argument("--topic", action="append", dest="topics", help="Provide topic names directly.")
    start.set_defaults(func=cmd_start)

    return parser


def cmd_doctor(args: argparse.Namespace) -> int:
    repo_root = Path.cwd()
    host = collect_host_info(repo_root)
    plan = build_setup_plan(host, mode=args.mode)
    if args.json:
        payload = {
            "host": host.to_dict(),
            "plan": plan.to_dict(),
        }
        print(json.dumps(payload, indent=2, sort_keys=True))
        return 0

    print(f"Platform: {plan.platform_label}")
    print(f"Strategy: {plan.strategy}")
    print(f"Summary: {plan.summary}")
    if plan.ros_distro:
        print(f"ROS distro: {plan.ros_distro}")
    if plan.rationale:
        print("Rationale:")
        for line in plan.rationale:
            print(f"  - {line}")
    if plan.warnings:
        print("Warnings:")
        for line in plan.warnings:
            print(f"  - {line}")
    return 0


def cmd_setup(args: argparse.Namespace) -> int:
    repo_root = Path.cwd()
    _, plan = run_setup(repo_root, mode=args.mode, report_only=args.report_only, yes=args.yes)
    print(f"Prepared Nav2++ state under {repo_root / '.nav2pp'}")
    print(f"Selected strategy: {plan.strategy}")
    if args.report_only:
        print("Installer execution skipped because --report-only was set.")
    elif not args.yes and not sys.stdin.isatty():
        print("Installer execution skipped because the session is non-interactive and --yes was not provided.")
    generated_dir = repo_root / ".nav2pp" / "generated"
    print(f"Generated scripts live in {generated_dir}")
    if plan.warnings:
        print("Warnings:")
        for line in plan.warnings:
            print(f"  - {line}")
    return 0


def cmd_topics(args: argparse.Namespace) -> int:
    try:
        matches = discover_topics(topics=args.topics, topic_file=args.from_file)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    print(render_topic_report(matches, as_json=args.json))
    return 0


def cmd_validate(args: argparse.Namespace) -> int:
    try:
        report = validate_profile(
            args.profile,
            topics=args.topics,
            topic_file=args.from_file,
            profile_file=args.profile_file,
            repo_root=Path.cwd(),
        )
    except (RuntimeError, ValueError, json.JSONDecodeError) as exc:
        print(str(exc), file=sys.stderr)
        return 1
    print(render_validation_report(report, as_json=args.json))
    return 0 if report.passed else 1


def cmd_profile_scaffold(args: argparse.Namespace) -> int:
    try:
        output_path, notes = scaffold_profile(
            args.name,
            repo_root=Path.cwd(),
            topics=args.topics,
            topic_file=args.from_file,
            output_path=args.output,
            require_nvidia=not args.skip_nvidia,
            force=args.force,
        )
    except (RuntimeError, ValueError, json.JSONDecodeError) as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print(f"Wrote profile scaffold to {output_path}")
    print(f"Validate it with: ./nav2++ validate --profile-file {output_path}")
    if notes:
        print("Review:")
        for note in notes:
            print(f"  - {note}")
    return 0


def cmd_start(args: argparse.Namespace) -> int:
    repo_root = Path.cwd()
    _, plan = run_start(
        repo_root,
        mode=args.mode,
        topics=args.topics,
        topic_file=args.from_file,
        dry_run=args.dry_run,
        yes=args.yes,
    )
    print(f"Prepared start script {repo_root / '.nav2pp' / 'generated' / plan.runner_script}")
    print(f"Selected strategy: {plan.strategy}")
    print(f"Summary: {plan.summary}")
    if plan.stubbed_topics:
        print("Stubbed topics:")
        for topic_name in plan.stubbed_topics:
            print(f"  - {topic_name}")
    if args.dry_run:
        print("Start execution skipped because --dry-run was set.")
    elif not args.yes and not sys.stdin.isatty():
        print("Start execution skipped because the session is non-interactive and --yes was not provided.")
    if plan.warnings:
        print("Warnings:")
        for line in plan.warnings:
            print(f"  - {line}")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
