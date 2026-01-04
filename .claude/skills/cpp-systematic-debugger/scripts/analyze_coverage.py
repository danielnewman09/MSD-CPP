#!/usr/bin/env python3
"""
Analyze test coverage for C++ files to help with debugging.

Usage:
    python analyze_coverage.py path/to/suspect_file.cpp
    python analyze_coverage.py path/to/file.cpp --find-tests
    python analyze_coverage.py path/to/file.cpp --run-coverage
"""

import argparse
import subprocess
import sys
import re
from pathlib import Path


def find_related_tests(source_file: Path, project_root: Path) -> list[Path]:
    """Find test files that might test the given source file."""
    tests = []
    
    # Common test directory patterns
    test_dirs = ["tests", "test", "unittest", "unit_tests", "spec"]
    
    # Get the base name without extension
    stem = source_file.stem
    
    # Search patterns for C++ test files
    patterns = [
        f"test_{stem}.cpp",
        f"{stem}_test.cpp",
        f"{stem}_tests.cpp",
        f"test_{stem}*.cpp",
        f"*{stem}*test*.cpp",
    ]
    
    for test_dir in test_dirs:
        test_path = project_root / test_dir
        if test_path.exists():
            for pattern in patterns:
                tests.extend(test_path.rglob(pattern))
    
    # Also check for tests in the same directory
    source_dir = source_file.parent
    for pattern in patterns:
        tests.extend(source_dir.glob(pattern))
    
    # Deduplicate
    return list(set(tests))


def detect_test_framework(project_root: Path) -> str:
    """Detect which test framework is being used."""
    cmake_files = list(project_root.rglob("CMakeLists.txt"))
    
    for cmake_file in cmake_files:
        content = cmake_file.read_text().lower()
        if "gtest" in content or "googletest" in content:
            return "gtest"
        elif "catch" in content:
            return "catch2"
        elif "doctest" in content:
            return "doctest"
        elif "boost" in content and "test" in content:
            return "boost"
    
    return "unknown"


def analyze_test_file(test_file: Path) -> dict:
    """Analyze a C++ test file to extract test names."""
    content = test_file.read_text()
    tests = []
    
    # GTest patterns
    gtest_pattern = r'TEST(?:_F|_P)?\s*\(\s*(\w+)\s*,\s*(\w+)\s*\)'
    for match in re.finditer(gtest_pattern, content):
        suite, name = match.groups()
        line = content[:match.start()].count('\n') + 1
        tests.append({
            "name": f"{suite}.{name}",
            "line": line,
            "framework": "gtest"
        })
    
    # Catch2 patterns
    catch_pattern = r'TEST_CASE\s*\(\s*"([^"]+)"'
    for match in re.finditer(catch_pattern, content):
        name = match.group(1)
        line = content[:match.start()].count('\n') + 1
        tests.append({
            "name": name,
            "line": line,
            "framework": "catch2"
        })
    
    # Doctest patterns
    doctest_pattern = r'TEST_CASE\s*\(\s*"([^"]+)"'
    # Similar to Catch2, already covered above
    
    return {"tests": tests}


def find_header_file(source_file: Path, project_root: Path) -> Path | None:
    """Find the corresponding header file."""
    stem = source_file.stem
    
    # Check common header locations
    header_patterns = [
        source_file.parent / f"{stem}.h",
        source_file.parent / f"{stem}.hpp",
        project_root / "include" / f"{stem}.h",
        project_root / "include" / f"{stem}.hpp",
    ]
    
    for header in header_patterns:
        if header.exists():
            return header
    
    # Search more broadly
    for ext in [".h", ".hpp"]:
        results = list(project_root.rglob(f"{stem}{ext}"))
        if results:
            return results[0]
    
    return None


def run_coverage(source_file: Path, project_root: Path) -> str:
    """Run coverage analysis for the source file."""
    build_dir = project_root / "build"
    
    if not build_dir.exists():
        return "Build directory not found. Run cmake and build first."
    
    # Try to find coverage info
    coverage_files = list(build_dir.rglob("*.gcda")) + list(build_dir.rglob("*.gcno"))
    
    if not coverage_files:
        return """No coverage data found. To enable coverage:

1. Configure with coverage flags:
   cmake -DCMAKE_BUILD_TYPE=Debug \\
         -DCMAKE_CXX_FLAGS="--coverage -fprofile-arcs -ftest-coverage" ..

2. Build and run tests:
   cmake --build .
   ctest

3. Generate report:
   lcov --capture --directory . --output-file coverage.info
   lcov --list coverage.info | grep "{}"
""".format(source_file.name)
    
    # Try to run lcov
    cmd = f"lcov --list {build_dir}/coverage.info 2>/dev/null | grep -i {source_file.stem}"
    
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=30
        )
        if result.stdout:
            return f"Coverage for {source_file.name}:\n{result.stdout}"
        else:
            return f"No coverage data found for {source_file.name}"
    except Exception as e:
        return f"Error running coverage: {e}"


def main():
    parser = argparse.ArgumentParser(
        description="Analyze C++ test coverage for debugging"
    )
    parser.add_argument(
        "source_file",
        help="Path to the C++ source file to analyze"
    )
    parser.add_argument(
        "--find-tests",
        action="store_true",
        help="Find and list related test files"
    )
    parser.add_argument(
        "--run-coverage",
        action="store_true",
        help="Run coverage analysis"
    )
    parser.add_argument(
        "--project-root",
        default=".",
        help="Project root directory"
    )
    
    args = parser.parse_args()
    
    source_file = Path(args.source_file).resolve()
    project_root = Path(args.project_root).resolve()
    
    if not source_file.exists():
        print(f"❌ Source file not found: {source_file}")
        sys.exit(1)
    
    print(f"📁 Analyzing: {source_file}")
    print(f"📂 Project root: {project_root}")
    print()
    
    # Detect test framework
    framework = detect_test_framework(project_root)
    print(f"🧪 Detected test framework: {framework}")
    print()
    
    # Find corresponding header
    header = find_header_file(source_file, project_root)
    if header:
        print(f"📄 Header file: {header.relative_to(project_root)}")
    else:
        print("📄 Header file: not found")
    print()
    
    # Find related tests
    test_files = find_related_tests(source_file, project_root)
    
    if args.find_tests or not args.run_coverage:
        print("🔍 Related Test Files:")
        if test_files:
            for tf in test_files:
                rel_path = tf.relative_to(project_root) if tf.is_relative_to(project_root) else tf
                print(f"   - {rel_path}")
                analysis = analyze_test_file(tf)
                if analysis.get("tests"):
                    for test in analysis["tests"][:10]:
                        print(f"       • {test['name']} (line {test['line']}, {test['framework']})")
        else:
            print("   ⚠️  No related test files found")
            print()
            print("   Suggestions:")
            print(f"   - Create tests/test_{source_file.stem}.cpp")
            print(f"   - Or create {source_file.parent}/test_{source_file.stem}.cpp")
    
    print()
    
    if args.run_coverage:
        print("📊 Coverage Analysis...")
        print("-" * 50)
        output = run_coverage(source_file, project_root)
        print(output)


if __name__ == "__main__":
    main()