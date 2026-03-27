#!/usr/bin/env python3
"""
Doxygen Documentation Generator for Conan Dependencies

Discovers Conan dependency include paths, generates minimal Doxyfiles (XML-only),
runs Doxygen on each dependency, and optionally ingests the results into the
codebase SQLite and/or Neo4j graph databases.

Usage:
    # Generate XML only
    python doxygen_deps.py --output-dir build/Debug/docs/deps

    # Generate XML and ingest into SQLite
    python doxygen_deps.py --output-dir build/Debug/docs/deps \
        --sqlite build/Debug/docs/codebase.db

    # Generate XML and ingest into Neo4j
    python doxygen_deps.py --output-dir build/Debug/docs/deps --neo4j

    # Only process specific dependencies
    python doxygen_deps.py --output-dir build/Debug/docs/deps --only eigen,sdl

    # Skip Doxygen generation, just ingest existing XML
    python doxygen_deps.py --output-dir build/Debug/docs/deps \
        --sqlite build/Debug/docs/codebase.db --skip-doxygen
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


# ---------------------------------------------------------------------------
# Dependency configuration
# ---------------------------------------------------------------------------

# Each entry configures how to run Doxygen on a specific Conan dependency.
# - file_patterns: which header extensions to index
# - recursive: whether to recurse into subdirectories
# - subdir: optional subdirectory under include/ to index (avoids noise)
# - predefined: optional preprocessor definitions for Doxygen
DEPS_CONFIG = {
    "eigen": {
        "file_patterns": "*.h *.hpp",
        "recursive": True,
        "subdir": "eigen3/Eigen",
        "predefined": "EIGEN_PARSED_BY_DOXYGEN",
    },
    "boost": {
        "file_patterns": "*.hpp",
        "recursive": True,
        "subdir": "boost/describe",
    },
    "sdl": {
        "file_patterns": "*.h",
        "recursive": False,
        "subdir": "SDL3",
    },
    "sdl_image": {
        "file_patterns": "*.h",
        "recursive": False,
        "subdir": "SDL3_image",
    },
    "sdl_mixer": {
        "file_patterns": "*.h",
        "recursive": False,
        "subdir": "SDL3_mixer",
    },
    "sdl_ttf": {
        "file_patterns": "*.h",
        "recursive": False,
        "subdir": "SDL3_ttf",
    },
    "spdlog": {
        "file_patterns": "*.h",
        "recursive": True,
        "subdir": "spdlog",
    },
    "nlopt": {
        "file_patterns": "*.h *.hpp",
        "recursive": True,
    },
    "qhull": {
        "file_patterns": "*.h",
        "recursive": True,
        "subdir": "libqhull_r",
    },
    "cpp_sqlite": {
        "file_patterns": "*.h *.hpp",
        "recursive": True,
    },
}


# ---------------------------------------------------------------------------
# Conan package discovery
# ---------------------------------------------------------------------------

def discover_conan_packages(project_dir: Path, build_type: str = "Debug") -> dict[str, Path]:
    """Discover Conan dependency include paths using conan graph info + cache path."""
    print(f"Discovering Conan dependency paths (build_type={build_type})...")
    try:
        result = subprocess.run(
            ["conan", "graph", "info", ".", "--format=json",
             "-s", f"build_type={build_type}"],
            capture_output=True, text=True, cwd=project_dir, check=True,
        )
    except subprocess.CalledProcessError as e:
        print(f"Error running 'conan graph info': {e.stderr}", file=sys.stderr)
        sys.exit(1)
    except FileNotFoundError:
        print("Error: 'conan' not found. Is Conan installed?", file=sys.stderr)
        sys.exit(1)

    raw = json.loads(result.stdout)
    # Conan 2.x nests nodes under "graph"
    graph_nodes = raw.get("graph", raw).get("nodes", {})
    paths = {}

    for node in graph_nodes.values():
        name = node.get("name", "")
        if name not in DEPS_CONFIG:
            continue

        # Try package_folder first (set when package is in local build context)
        pkg_folder = node.get("package_folder") or node.get("immutable_package_folder")

        # If not available, resolve via 'conan cache path <ref>:<package_id>'
        if not pkg_folder:
            ref = node.get("ref", "")
            package_id = node.get("package_id", "")
            if ref and package_id:
                try:
                    cache_result = subprocess.run(
                        ["conan", "cache", "path", f"{ref}:{package_id}"],
                        capture_output=True, text=True, check=True,
                    )
                    pkg_folder = cache_result.stdout.strip()
                except subprocess.CalledProcessError:
                    pass

        if pkg_folder:
            include_dir = Path(pkg_folder) / "include"
            if include_dir.exists():
                paths[name] = include_dir
                print(f"  Found {name}: {include_dir}")
            else:
                print(f"  Warning: {name} include dir not found: {include_dir}")
        else:
            binary_status = node.get("binary", "unknown")
            print(f"  Warning: {name} package not installed (binary: {binary_status})")

    return paths


# ---------------------------------------------------------------------------
# Doxyfile generation and Doxygen execution
# ---------------------------------------------------------------------------

def generate_doxyfile(dep_name: str, include_path: Path, xml_output_dir: Path) -> str:
    """Generate a minimal Doxyfile for XML-only output."""
    config = DEPS_CONFIG[dep_name]

    input_path = include_path
    if "subdir" in config:
        subdir_path = include_path / config["subdir"]
        if subdir_path.exists():
            input_path = subdir_path
        else:
            print(f"  Warning: subdir '{config['subdir']}' not found, using {include_path}")

    exclude_patterns = config.get("exclude_patterns", "")
    predefined = config.get("predefined", "")

    return f"""\
# Auto-generated Doxyfile for {dep_name} dependency
PROJECT_NAME           = "{dep_name}"
INPUT                  = {input_path}
RECURSIVE              = {"YES" if config.get("recursive") else "NO"}
FILE_PATTERNS          = {config["file_patterns"]}
EXCLUDE_PATTERNS       = {exclude_patterns}

GENERATE_HTML          = NO
GENERATE_LATEX         = NO
GENERATE_XML           = YES
XML_OUTPUT             = {xml_output_dir}

EXTRACT_ALL            = YES
EXTRACT_PRIVATE        = NO
EXTRACT_STATIC         = YES
EXTRACT_LOCAL_CLASSES  = YES

QUIET                  = YES
WARNINGS               = NO
WARN_IF_UNDOCUMENTED   = NO

JAVADOC_AUTOBRIEF      = YES
BUILTIN_STL_SUPPORT    = YES
ENABLE_PREPROCESSING   = YES
MACRO_EXPANSION        = YES
EXPAND_ONLY_PREDEF     = NO
PREDEFINED             = {predefined}

REFERENCED_BY_RELATION = YES
REFERENCES_RELATION    = YES

HAVE_DOT               = NO
"""


def run_doxygen(dep_name: str, include_path: Path, output_base: Path) -> Path | None:
    """Run Doxygen on a single dependency. Returns XML output dir or None on failure."""
    xml_dir = output_base / dep_name / "xml"
    xml_dir.mkdir(parents=True, exist_ok=True)

    doxyfile_content = generate_doxyfile(dep_name, include_path, xml_dir)

    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".doxyfile", delete=False, prefix=f"doxy_{dep_name}_"
    ) as f:
        f.write(doxyfile_content)
        doxyfile_path = f.name

    try:
        print(f"  Running Doxygen for {dep_name}...")
        subprocess.run(
            ["doxygen", doxyfile_path],
            check=True, capture_output=True, text=True,
        )

        index_xml = xml_dir / "index.xml"
        if not index_xml.exists():
            print(f"  Warning: Doxygen produced no index.xml for {dep_name}")
            return None

        # Count generated files
        xml_count = len(list(xml_dir.glob("*.xml")))
        print(f"  {dep_name}: {xml_count} XML files generated")
        return xml_dir

    except subprocess.CalledProcessError as e:
        print(f"  Error running Doxygen for {dep_name}: {e.stderr[:200]}")
        return None
    except FileNotFoundError:
        print("Error: 'doxygen' not found. Is Doxygen installed?", file=sys.stderr)
        sys.exit(1)
    finally:
        os.unlink(doxyfile_path)


# ---------------------------------------------------------------------------
# Ingestion into databases
# ---------------------------------------------------------------------------

def ingest_sqlite(xml_dir: Path, dep_name: str, db_path: Path, project_root: Path) -> None:
    """Ingest dependency XML into the SQLite codebase database."""
    script = project_root / "scripts" / "doxygen_to_sqlite.py"
    print(f"  Ingesting {dep_name} into SQLite ({db_path.name})...")
    subprocess.run(
        [
            sys.executable, str(script),
            str(xml_dir), str(db_path),
            "--source", dep_name,
            "--append",
        ],
        check=True,
    )


def ingest_neo4j(xml_dir: Path, dep_name: str, project_root: Path,
                 uri: str, user: str, password: str) -> None:
    """Ingest dependency XML into the Neo4j graph database."""
    script = project_root / "scripts" / "doxygen_to_neo4j.py"
    print(f"  Ingesting {dep_name} into Neo4j...")
    subprocess.run(
        [
            sys.executable, str(script),
            str(xml_dir),
            "--source", dep_name,
            "--no-clear",
            "--uri", uri,
            "--user", user,
            "--password", password,
        ],
        check=True,
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Generate Doxygen documentation for Conan dependencies"
    )
    parser.add_argument("--output-dir", default=None,
                        help="Base output directory for dependency XML (e.g., build/Debug/docs/deps)")
    parser.add_argument("--project-dir", default=".",
                        help="Project root directory (default: current directory)")
    parser.add_argument("--only", default=None,
                        help="Comma-separated list of deps to process (default: all configured)")
    parser.add_argument("--build-type", default="Debug",
                        help="Conan build type to match installed packages (default: Debug)")
    parser.add_argument("--skip-doxygen", action="store_true",
                        help="Skip Doxygen generation, only run ingestion on existing XML")
    parser.add_argument("--list-deps", action="store_true",
                        help="List configured dependencies and exit")

    # SQLite ingestion
    parser.add_argument("--sqlite", default=None,
                        help="Path to SQLite codebase.db to append dependency data")

    # Neo4j ingestion
    parser.add_argument("--neo4j", action="store_true",
                        help="Ingest into Neo4j graph database")
    parser.add_argument("--neo4j-uri", default=os.environ.get("NEO4J_URI", "bolt://localhost:7687"),
                        help="Neo4j Bolt URI")
    parser.add_argument("--neo4j-user", default=os.environ.get("NEO4J_USER", "neo4j"),
                        help="Neo4j username")
    parser.add_argument("--neo4j-password", default=os.environ.get("NEO4J_PASSWORD", "msd-local-dev"),
                        help="Neo4j password")

    args = parser.parse_args()

    if args.list_deps:
        print("Configured dependencies:")
        for name, config in sorted(DEPS_CONFIG.items()):
            subdir = config.get("subdir", "(root)")
            print(f"  {name:15s}  patterns={config['file_patterns']}  subdir={subdir}")
        sys.exit(0)

    if not args.output_dir:
        print("Error: --output-dir is required (unless using --list-deps)", file=sys.stderr)
        sys.exit(1)

    project_dir = Path(args.project_dir).resolve()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Filter dependencies if --only specified
    requested_deps = set(DEPS_CONFIG.keys())
    if args.only:
        requested_deps = set(args.only.split(","))
        unknown = requested_deps - set(DEPS_CONFIG.keys())
        if unknown:
            print(f"Warning: Unknown dependencies (skipping): {', '.join(unknown)}")
            requested_deps -= unknown

    # Discover Conan package paths
    package_paths = discover_conan_packages(project_dir, args.build_type)
    available_deps = requested_deps & set(package_paths.keys())
    missing_deps = requested_deps - set(package_paths.keys())

    if missing_deps:
        print(f"\nDependencies not found in Conan cache (skipping): {', '.join(sorted(missing_deps))}")
        print("  Hint: Run 'conan install . --build=missing' first")

    if not available_deps:
        print("No dependencies to process.", file=sys.stderr)
        sys.exit(1)

    print(f"\nProcessing {len(available_deps)} dependencies: {', '.join(sorted(available_deps))}")

    # Phase 1: Generate Doxygen XML
    xml_dirs: dict[str, Path] = {}
    if not args.skip_doxygen:
        print("\n--- Phase 1: Generating Doxygen XML ---")
        for dep_name in sorted(available_deps):
            xml_dir = run_doxygen(dep_name, package_paths[dep_name], output_dir)
            if xml_dir:
                xml_dirs[dep_name] = xml_dir
    else:
        print("\n--- Skipping Doxygen generation (--skip-doxygen) ---")
        for dep_name in sorted(available_deps):
            xml_dir = output_dir / dep_name / "xml"
            if (xml_dir / "index.xml").exists():
                xml_dirs[dep_name] = xml_dir
                print(f"  Found existing XML for {dep_name}")
            else:
                print(f"  No existing XML for {dep_name}, skipping")

    if not xml_dirs:
        print("No XML output produced. Nothing to ingest.", file=sys.stderr)
        sys.exit(1)

    # Phase 2: Ingest into databases
    if args.sqlite or args.neo4j:
        print("\n--- Phase 2: Ingesting into databases ---")

        for dep_name, xml_dir in sorted(xml_dirs.items()):
            if args.sqlite:
                try:
                    ingest_sqlite(xml_dir, dep_name, Path(args.sqlite), project_dir)
                except subprocess.CalledProcessError as e:
                    print(f"  Error ingesting {dep_name} into SQLite: {e}")

            if args.neo4j:
                try:
                    ingest_neo4j(
                        xml_dir, dep_name, project_dir,
                        args.neo4j_uri, args.neo4j_user, args.neo4j_password,
                    )
                except subprocess.CalledProcessError as e:
                    print(f"  Error ingesting {dep_name} into Neo4j: {e}")

    # Summary
    print("\n--- Summary ---")
    print(f"Dependencies processed: {len(xml_dirs)}")
    for dep_name, xml_dir in sorted(xml_dirs.items()):
        xml_count = len(list(xml_dir.glob("*.xml")))
        print(f"  {dep_name}: {xml_count} XML files")
    if args.sqlite:
        print(f"SQLite database: {args.sqlite}")
    if args.neo4j:
        print(f"Neo4j: {args.neo4j_uri}")
    print(f"XML output: {output_dir}")


if __name__ == "__main__":
    main()
