#!/usr/bin/env python3

import os
import subprocess
import shutil
import multiprocessing
import json
from datetime import datetime


def extend_mem_file(input_file, target_lines, output_file, padding_value="0000000000000000"):
    """
    Extends a memory file to the specified number of lines
    """
    with open(input_file, 'r') as f:
        existing_lines = [line.strip() for line in f.readlines() if line.strip()]

    current_lines = len(existing_lines)

    if current_lines > target_lines:
        raise ValueError(f"File already has {current_lines} lines, which exceeds target {target_lines}")

    # Calculate padding needed
    padding_needed = target_lines - current_lines
    padding = [padding_value + '\n' for _ in range(padding_needed)]

    # Write output
    with open(output_file, 'w') as f:
        f.writelines([line + '\n' for line in existing_lines])
        f.writelines(padding)

    print(f"  > Extended {input_file} from {current_lines} to {target_lines} lines")


def read_apps_path_file(apps_path_file: str, zephyr_base: str) -> list:
    """
    Reads a file containing application paths to compile.
    Supports both relative paths (from ZEPHYR_BASE) and absolute paths.
    """
    apps = []

    if not os.path.exists(apps_path_file):
        print(f"Warning: apps.path file '{apps_path_file}' not found!")
        print("Creating a sample apps.path file with common applications...")

        # Create a sample apps.path file
        sample_apps = [
            "samples/basic/blinky",
            "samples/basic/button", 
            "samples/hello_world",
            "tests/kernel/common",
            "# Add more application paths here",
            "# Use relative paths from ZEPHYR_BASE or absolute paths"
        ]

        with open(apps_path_file, 'w') as f:
            f.write("\n".join(sample_apps))

        print(f"Sample apps.path file created. Please edit '{apps_path_file}' and run again.")
        return []

    print(f"Reading application paths from '{apps_path_file}'...")

    with open(apps_path_file, 'r') as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()

            # Skip empty lines and comments
            if not line or line.startswith('#'):
                continue

            # Handle relative paths (from ZEPHYR_BASE)
            if not os.path.isabs(line):
                full_path = os.path.join(zephyr_base, line)
            else:
                full_path = line

            # Validate that the path exists and is a Zephyr app
            if os.path.exists(full_path):
                cmake_file = os.path.join(full_path, "CMakeLists.txt")
                prj_file = os.path.join(full_path, "prj.conf")

                if os.path.exists(cmake_file) and os.path.exists(prj_file):
                    apps.append(full_path)
                    print(f"  ✓ Added: {line}")
                else:
                    print(f"  ⚠️  Line {line_num}: '{line}' - Missing CMakeLists.txt or prj.conf")
            else:
                print(f"  ❌ Line {line_num}: '{line}' - Path does not exist")

    print(f"Loaded {len(apps)} valid application paths from '{apps_path_file}'.")
    return apps


def build_zephyr_app(app_path: str, output_base_dir: str, zephyr_base: str, compiled_apps_list: list, failed_apps_list: list):
    """
    Builds a Zephyr application and copies the output files.
    """
    try:
        relative_app_path = os.path.relpath(app_path, zephyr_base)
    except ValueError:
        # Handle case where app_path is not under zephyr_base
        relative_app_path = os.path.basename(app_path)

    app_output_dir = os.path.join(output_base_dir, relative_app_path)

    print(f"""
--------------------------------------------------------------------------------
Processing application: {relative_app_path}
  > App location: {app_path}
  > Building Zephyr application...""")

    build_dir = os.path.join(app_path, "build")
    error_log = []

    try:
        # Step 1: Clean and create build directory
        if os.path.exists(build_dir):
            shutil.rmtree(build_dir)
        os.makedirs(build_dir)

        # Step 2: Run CMake configuration
        cmake_process = subprocess.run(
            ['cmake', '-DBOARD=vision-soc', '..'],
            cwd=build_dir,
            capture_output=True,
            text=True,
            timeout=120
        )

        if cmake_process.returncode != 0:
            error_msg = f"CMake failed with return code {cmake_process.returncode}"
            if cmake_process.stderr:
                error_msg += f"\nSTDERR: {cmake_process.stderr[:500]}..."
            error_log.append(error_msg)
            print(f"  > ❌ CMake configuration FAILED for {relative_app_path}.")
            failed_apps_list.append({
                "app": relative_app_path,
                "stage": "cmake",
                "error": error_msg
            })
            return None

        # Step 3: Run make
        make_process = subprocess.run(
            ['make', '-j4'],  # Use 4 parallel jobs for faster builds
            cwd=build_dir,
            capture_output=True,
            text=True,
            timeout=300
        )

        if make_process.returncode != 0:
            error_msg = f"Make failed with return code {make_process.returncode}"
            if make_process.stderr:
                # Extract relevant error lines
                stderr_lines = make_process.stderr.split('\n')
                relevant_errors = [line for line in stderr_lines if 'error:' in line.lower() or 'fatal:' in line.lower()]
                if relevant_errors:
                    error_msg += f"\nKey errors: {'; '.join(relevant_errors[:3])}"
            error_log.append(error_msg)
            print(f"  > ❌ Make build FAILED for {relative_app_path}.")
            failed_apps_list.append({
                "app": relative_app_path,
                "stage": "make",
                "error": error_msg
            })
            return None

        # Step 4: Convert ELF to hex memory format
        zephyr_elf = os.path.join(build_dir, "zephyr", "zephyr.elf")
        if not os.path.exists(zephyr_elf):
            error_msg = "zephyr.elf file not generated"
            error_log.append(error_msg)
            print(f"  > ❌ ELF file missing for {relative_app_path}.")
            failed_apps_list.append({
                "app": relative_app_path,
                "stage": "elf_check",
                "error": error_msg
            })
            return None

        code_mini_mem = os.path.join(build_dir, "zephyr", "code-mini.mem")

        elf2hex_process = subprocess.run(
            ['riscv64-unknown-elf-elf2hex', '--bit-width', '64', '--input', zephyr_elf, '--output', code_mini_mem],
            cwd=build_dir,
            capture_output=True,
            text=True,
            timeout=60
        )

        if elf2hex_process.returncode != 0:
            error_msg = f"ELF2HEX failed with return code {elf2hex_process.returncode}"
            if elf2hex_process.stderr:
                error_msg += f"\nSTDERR: {elf2hex_process.stderr[:200]}..."
            error_log.append(error_msg)
            print(f"  > ❌ ELF2HEX conversion FAILED for {relative_app_path}.")
            failed_apps_list.append({
                "app": relative_app_path,
                "stage": "elf2hex",
                "error": error_msg
            })
            return None

        # Step 5: Extend memory file
        code_mem = os.path.join(build_dir, "zephyr", "code.mem")
        try:
            extend_mem_file(code_mini_mem, 4194304, code_mem, "0000000000000000")
        except Exception as e:
            error_msg = f"Memory file extension failed: {str(e)}"
            error_log.append(error_msg)
            print(f"  > ❌ Memory file extension FAILED for {relative_app_path}: {e}")
            failed_apps_list.append({
                "app": relative_app_path,
                "stage": "mem_extend",
                "error": error_msg
            })
            return None

        print(f"  > ✅ Build successful for {relative_app_path}.")

    except subprocess.TimeoutExpired as e:
        error_msg = f"Build timeout after {e.timeout} seconds"
        error_log.append(error_msg)
        print(f"  > ❌ Build TIMEOUT for {relative_app_path}.")
        failed_apps_list.append({
            "app": relative_app_path,
            "stage": "timeout",
            "error": error_msg
        })
        return None
    except Exception as e:
        error_msg = f"Unexpected error: {str(e)}"
        error_log.append(error_msg)
        print(f"  > ❌ Unexpected error during build for {relative_app_path}: {e}")
        failed_apps_list.append({
            "app": relative_app_path,
            "stage": "exception",
            "error": error_msg
        })
        return None

    # Step 6: Create output directory and copy files
    os.makedirs(app_output_dir, exist_ok=True)

    files_to_copy = {
        "zephyr.lst": os.path.join(build_dir, "zephyr", "zephyr.lst"),
        "code.mem": os.path.join(build_dir, "zephyr", "code.mem"),
        "zephyr.elf": os.path.join(build_dir, "zephyr", "zephyr.elf")
    }

    all_files_copied = True
    for name, src_path in files_to_copy.items():
        if os.path.exists(src_path):
            try:
                shutil.copy2(src_path, os.path.join(app_output_dir, name))
            except Exception as e:
                print(f"  > ❌ Failed to copy {name} for {relative_app_path}: {e}")
                all_files_copied = False
        else:
            print(f"  > ⚠️  File {name} not found for {relative_app_path}")
            all_files_copied = False

    if all_files_copied:
        compiled_apps_list.append(relative_app_path)
        return relative_app_path
    else:
        failed_apps_list.append({
            "app": relative_app_path,
            "stage": "file_copy",
            "error": "Failed to copy one or more output files"
        })
        return None


def create_compiled_apps_list(output_base_dir: str, compiled_apps: list, failed_apps: list):
    """Create files listing successfully compiled and failed applications."""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # Successful builds
    success_content = f"""# Successfully Compiled Zephyr Applications
# Generated on: {timestamp}
# Total applications compiled: {len(compiled_apps)}

"""

    for app in sorted(compiled_apps):
        success_content += f"{app}\n"

    success_file_path = os.path.join(output_base_dir, "compiled_applications.txt")
    with open(success_file_path, 'w') as f:
        f.write(success_content)

    # Failed builds
    failed_content = f"""# Failed Zephyr Application Builds
# Generated on: {timestamp}
# Total failed builds: {len(failed_apps)}

"""

    for failed in failed_apps:
        failed_content += f"{failed['app']} - Stage: {failed['stage']}\n"
        failed_content += f"  Error: {failed['error'][:100]}...\n\n"

    failed_file_path = os.path.join(output_base_dir, "failed_applications.txt")
    with open(failed_file_path, 'w') as f:
        f.write(failed_content)

    # JSON summary
    json_data = {
        "timestamp": timestamp,
        "total_compiled": len(compiled_apps),
        "total_failed": len(failed_apps),
        "compiled_apps": sorted(compiled_apps),
        "failed_apps": failed_apps
    }

    json_file_path = os.path.join(output_base_dir, "build_summary.json")
    with open(json_file_path, 'w') as f:
        json.dump(json_data, f, indent=2)

    print(f"""
📝 Created build reports:
  > Successful builds: {success_file_path}
  > Failed builds: {failed_file_path}
  > JSON summary: {json_file_path}""")


def main():
    """Main function to build selected Zephyr apps from apps.path file."""
    zephyr_base = os.environ.get("ZEPHYR_BASE")
    if not zephyr_base:
        print("Error: 'ZEPHYR_BASE' environment variable is not set.")
        print("Please set ZEPHYR_BASE to your Zephyr installation directory.")
        return

    print(f"Using ZEPHYR_BASE: {zephyr_base}")

    output_base_dir = os.path.join(os.getcwd(), "zephyr-test-apps")
    os.makedirs(output_base_dir, exist_ok=True)
    print(f"Output will be stored in: {output_base_dir}")

    # Read application paths from apps.path file
    apps_path_file = os.path.join(os.getcwd(), "apps.path")
    app_paths = read_apps_path_file(apps_path_file, zephyr_base)

    if not app_paths:
        print("No valid applications to compile.")
        return

    print(f"\nFound {len(app_paths)} applications to build.")

    # Shared lists to track results
    manager = multiprocessing.Manager()
    compiled_apps_list = manager.list()
    failed_apps_list = manager.list()

    # Prepare for multiprocessing
    tasks = [(app, output_base_dir, zephyr_base, compiled_apps_list, failed_apps_list) for app in app_paths]
    num_processes = min(os.cpu_count(), 4)  # Reduced to 4 to avoid overwhelming system
    print(f"🚀 Starting parallel build with {num_processes} processes...\n")

    # Run builds in parallel
    with multiprocessing.Pool(processes=num_processes) as pool:
        results = pool.starmap(build_zephyr_app, tasks)

    # Get the lists of compiled and failed applications
    compiled_apps = list(compiled_apps_list)
    failed_apps = list(failed_apps_list)

    # Create the build reports
    create_compiled_apps_list(output_base_dir, compiled_apps, failed_apps)

    # Print summary
    print("\n" + "=" * 80)
    print("🎉 BUILD SUMMARY")
    print(f"Applications attempted: {len(app_paths)}")
    print(f"Successfully compiled: {len(compiled_apps)}")
    print(f"Failed builds: {len(failed_apps)}")
    if app_paths:
        print(f"Success rate: {(len(compiled_apps) / len(app_paths) * 100):.1f}%")

    if failed_apps:
        print("\n❌ Common failure stages:")
        stages = {}
        for failed in failed_apps:
            stage = failed['stage']
            stages[stage] = stages.get(stage, 0) + 1
        for stage, count in sorted(stages.items()):
            print(f"  {stage}: {count} failures")

    print(f"\nOutput directory: {output_base_dir}")
    print("Check 'compiled_applications.txt' and 'failed_applications.txt' for details.")
    print("=" * 80)


if __name__ == "__main__":
    main()
