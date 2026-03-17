#!/usr/bin/env python3
import argparse

def extend_mem_file(input_file, target_lines, output_file, padding_value="0000000000000000"):
    """
    Extends a memory file to the specified number of lines
    :param input_file: Input .mem file path
    :param target_lines: Total number of lines required
    :param output_file: Output .mem file path
    :param padding_value: Value to use for padding (default: 32-bit zero)
    """
    # Read existing lines
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
    
    print(f"Extended {input_file} from {current_lines} to {target_lines} lines")
    print(f"Output file: {output_file}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Extend .mem file with padding')
    parser.add_argument('-i', '--input', required=True, help='Input .mem file')
    parser.add_argument('-o', '--output', required=True, help='Output .mem file')
    parser.add_argument('-n', '--lines', type=int, required=True, help='Target number of lines')
    parser.add_argument('-p', '--pad', default="0000000000000000", help='Padding value (default: 0000000000000000)')
    
    args = parser.parse_args()
    
    extend_mem_file(
        input_file=args.input,
        target_lines=args.lines,
        output_file=args.output,
        padding_value=args.pad
    )

