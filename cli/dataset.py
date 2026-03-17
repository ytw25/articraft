from __future__ import annotations

import argparse
from datetime import datetime, timezone

from storage.datasets import DatasetStore
from storage.queries import StorageQueries
from storage.repo import StorageRepo

from cli.common import add_data_root_argument


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="articraft-dataset")
    add_data_root_argument(parser)
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("init-storage", help="Create the canonical data/ directory layout.")
    subparsers.add_parser("status", help="Show dataset storage status.")
    promote = subparsers.add_parser("promote", help="Promote a record into the dataset with a record-local entry.")
    promote.add_argument("--record-id", required=True, help="Record identifier to promote.")
    promote.add_argument("--dataset-id", required=True, help="Stable dataset identifier to assign.")
    promote.add_argument("--category-slug", help="Optional category slug override when the record lacks one.")
    promote.add_argument(
        "--promoted-at",
        help="UTC timestamp for promotion. Defaults to the current time.",
    )
    subparsers.add_parser("validate", help="Validate record-local dataset entries.")
    subparsers.add_parser("build-manifest", help="Build the derived dataset manifest under data/cache/manifests/.")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    repo = StorageRepo(args.repo_root)
    datasets = DatasetStore(repo)
    queries = StorageQueries(repo)

    if args.command == "init-storage":
        repo.ensure_layout()
        print(f"Initialized dataset storage at {repo.layout.data_root}")
        return 0

    if args.command == "status":
        record_count = len(queries.list_record_ids())
        category_count = len(queries.list_category_slugs())
        dataset_entries = datasets.list_entries()
        print(f"records={record_count} categories={category_count} dataset_entries={len(dataset_entries)}")
        return 0

    if args.command == "promote":
        repo.ensure_layout()
        try:
            entry = datasets.promote_record(
                record_id=args.record_id,
                dataset_id=args.dataset_id,
                category_slug=args.category_slug,
                promoted_at=args.promoted_at or _utc_now(),
            )
        except ValueError as exc:
            print(str(exc))
            return 1
        print(
            "Promoted "
            f"record_id={entry['record_id']} dataset_id={entry['dataset_id']} "
            f"at {repo.layout.record_dataset_entry_path(entry['record_id'])}"
        )
        return 0

    if args.command == "validate":
        errors = datasets.validate()
        if errors:
            for error in errors:
                print(error)
            return 1
        print(f"Dataset valid: entries={len(datasets.list_entries())}")
        return 0

    if args.command == "build-manifest":
        repo.ensure_layout()
        manifest = datasets.write_dataset_manifest()
        print(f"Wrote dataset manifest to {repo.layout.dataset_manifest_path()} entries={len(manifest['generated'])}")
        return 0

    parser.error(f"Unhandled command: {args.command}")
    return 2
