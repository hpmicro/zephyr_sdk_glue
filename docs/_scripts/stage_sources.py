# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: Apache-2.0

"""Prepare isolated English and Chinese Sphinx source trees."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path


LANGUAGE_SUFFIX = {"en": "en", "zh": "zh"}


def _copy_file(source: Path, destination: Path) -> None:
	destination.parent.mkdir(parents=True, exist_ok=True)
	shutil.copy2(source, destination)


def _validate_destination(docs_dir: Path, destination: Path) -> None:
	build_dir = (docs_dir / "build").resolve()
	destination = destination.resolve()
	if destination != build_dir and build_dir not in destination.parents:
		raise ValueError(f"destination must be inside {build_dir}: {destination}")


def stage_language(docs_dir: Path, language: str) -> None:
	repository = docs_dir.parent
	destination = docs_dir / "build" / "source" / language
	_validate_destination(docs_dir, destination)

	if destination.exists():
		shutil.rmtree(destination)

	static_source = docs_dir / "source" / language
	shutil.copytree(
		static_source,
		destination,
		ignore=shutil.ignore_patterns("samples", "snippets", "tests", "CHANGELOG.md"),
	)

	_copy_file(repository / "CHANGELOG.md", destination / "CHANGELOG.md")
	verified_samples = static_source / "snippets" / f"verified_samples_{language}.rst"
	_copy_file(
		verified_samples,
		destination / "snippets" / verified_samples.name,
	)

	suffix = LANGUAGE_SUFFIX[language]
	for source_root in (repository / "samples", repository / "snippets"):
		for source in source_root.rglob(f"*_{suffix}.rst"):
			_copy_file(source, destination / source.relative_to(repository))

	clock_tree = repository / "tests" / "clock_tree"
	clock_source = clock_tree / ("README.rst" if language == "en" else "README_zh.rst")
	_copy_file(clock_source, destination / clock_source.relative_to(repository))


def clean(docs_dir: Path) -> None:
	build_dir = docs_dir / "build"
	_validate_destination(docs_dir, build_dir)
	if build_dir.exists():
		shutil.rmtree(build_dir)


def main() -> None:
	parser = argparse.ArgumentParser()
	parser.add_argument("--language", choices=sorted(LANGUAGE_SUFFIX))
	parser.add_argument("--clean", action="store_true")
	args = parser.parse_args()

	docs_dir = Path(__file__).resolve().parents[1]
	if args.clean:
		clean(docs_dir)
		return
	if args.language is None:
		parser.error("--language is required unless --clean is used")
	stage_language(docs_dir, args.language)


if __name__ == "__main__":
	main()
