#!/usr/bin/env python3
"""
Validate tutorial directory structure and required files.

Usage:
    python validate_tutorial.py <tutorial_path>
    
Example:
    python validate_tutorial.py docs/tutorials/matrix-multiplication
"""

import sys
import os
from pathlib import Path


REQUIRED_FILES = [
    "README.md",
    "example.cpp", 
    "CMakeLists.txt",
    "presentation.html"
]

OPTIONAL_FILES = [
    "algorithm.md",
    "Makefile"
]


def validate_tutorial(tutorial_path: str) -> tuple[bool, list[str]]:
    """
    Validate a tutorial directory has all required files.
    
    Args:
        tutorial_path: Path to the tutorial directory
        
    Returns:
        Tuple of (is_valid, list of error messages)
    """
    errors = []
    path = Path(tutorial_path)
    
    if not path.exists():
        return False, [f"Directory does not exist: {tutorial_path}"]
    
    if not path.is_dir():
        return False, [f"Path is not a directory: {tutorial_path}"]
    
    # Check required files
    for filename in REQUIRED_FILES:
        filepath = path / filename
        if not filepath.exists():
            errors.append(f"Missing required file: {filename}")
        elif filepath.stat().st_size == 0:
            errors.append(f"File is empty: {filename}")
    
    # Check README.md has expected sections
    readme_path = path / "README.md"
    if readme_path.exists():
        content = readme_path.read_text()
        expected_sections = ["## Overview", "## Prerequisites", "## References"]
        for section in expected_sections:
            if section not in content:
                errors.append(f"README.md missing section: {section}")
    
    # Check example.cpp compiles (basic syntax check)
    cpp_path = path / "example.cpp"
    if cpp_path.exists():
        content = cpp_path.read_text()
        if "int main(" not in content and "int main (" not in content:
            errors.append("example.cpp missing main() function")
        if "#include" not in content:
            errors.append("example.cpp has no #include statements")
    
    # Check CMakeLists.txt has project definition
    cmake_path = path / "CMakeLists.txt"
    if cmake_path.exists():
        content = cmake_path.read_text()
        if "project(" not in content:
            errors.append("CMakeLists.txt missing project() definition")
        if "add_executable" not in content:
            errors.append("CMakeLists.txt missing add_executable()")
    
    # Check presentation.html has Reveal.js
    html_path = path / "presentation.html"
    if html_path.exists():
        content = html_path.read_text()
        if "reveal.js" not in content.lower():
            errors.append("presentation.html missing Reveal.js reference")
        if "<section>" not in content:
            errors.append("presentation.html missing slide sections")
    
    return len(errors) == 0, errors


def main():
    if len(sys.argv) < 2:
        print("Usage: python validate_tutorial.py <tutorial_path>")
        print("Example: python validate_tutorial.py docs/tutorials/matrix-ops")
        sys.exit(1)
    
    tutorial_path = sys.argv[1]
    print(f"Validating tutorial: {tutorial_path}")
    print("-" * 50)
    
    is_valid, errors = validate_tutorial(tutorial_path)
    
    if is_valid:
        print("✅ Tutorial structure is valid!")
        print(f"\nFiles present:")
        for f in REQUIRED_FILES:
            print(f"  ✓ {f}")
        
        # Check for optional files
        path = Path(tutorial_path)
        for f in OPTIONAL_FILES:
            if (path / f).exists():
                print(f"  ✓ {f} (optional)")
    else:
        print("❌ Validation failed!\n")
        print("Errors:")
        for error in errors:
            print(f"  • {error}")
        sys.exit(1)
    
    print("\n" + "-" * 50)
    print("Next steps:")
    print(f"  1. Build: cd {tutorial_path} && mkdir -p build && cd build && cmake .. && make")
    print(f"  2. Run:   ./build/example")
    print(f"  3. View:  Open {tutorial_path}/presentation.html in browser")


if __name__ == "__main__":
    main()