#!/usr/bin/env python3

import subprocess
import sys
import os

script_path = os.path.abspath(__file__)
script_directory = os.path.dirname(script_path)
btor2aig_path = script_directory + "/deps/btor2tools/build/bin/btor2aiger"
verbose = True


def modify_input_file(filepath):
    if verbose:
        print(f"Modifying input file: {filepath}")
    btor2aiger_cmd = btor2aig_path + " " + filepath + " -a > " + filepath + ".aig"
    try:
        subprocess.run(btor2aiger_cmd, shell=True)
    except Exception as e:
        print(f"Error modifying input file: {e}")
    return filepath + ".aig"


def modify_output_file(check_result, input_filepath, output_dir):
    check_result = check_result.split("\n")[-1].strip()

    # Check simplecar Result
    if check_result != "Unsafe":
        if verbose:
            print("No output file to modify.")
        return

    # Read btor2 input info
    input_file_name = input_filepath.split("/")[-1]
    aig_cex_filepath = output_dir + input_file_name + ".cex"
    if verbose:
        print(f"Modifying output file: {aig_cex_filepath}")

    sorts_dict = {}  # {id, bitvec_width}
    input_dict = {}  # {id, (input_id, input_index, end_index)}
    uninit_state_dict = {}  # {id, (state_id, state_index, end_index)}
    nonnext_state_dict = {}  # {id, (state_id, state_input_index, end_index)}
    uninit_state_set = set()
    nonnext_state_set = set()
    input_id = 0
    input_index = 0
    state_id = 0
    state_index = 0
    with open(input_filepath, "r") as btor2_file:
        if verbose:
            print("Reading input file.")
        # first scan
        for line in btor2_file:
            l = line.split(" ")
            if l[1] == "sort":
                sorts_dict[int(l[0])] = int(l[3])
            if l[1] == "input":
                input_dict[int(l[0])] = (
                    input_id,
                    input_index,
                    input_index + sorts_dict[int(l[2])],
                )
                input_id += 1
                input_index += sorts_dict[int(l[2])]
            if l[1] == "state":
                uninit_state_set.add(int(l[0]))
                nonnext_state_set.add(int(l[0]))
            if l[1] == "init":
                uninit_state_set.remove(int(l[3]))
            if l[1] == "next":
                nonnext_state_set.remove(int(l[3]))

        btor2_file.seek(0)
        # second scan
        for line in btor2_file:
            l = line.split(" ")
            if l[1] == "state":
                if int(l[0]) in nonnext_state_set:
                    nonnext_state_dict[int(l[0])] = (
                        state_id,
                        input_index,
                        input_index + sorts_dict[int(l[2])],
                    )
                    input_index += sorts_dict[int(l[2])]
                else:
                    if int(l[0]) in uninit_state_set:
                        uninit_state_dict[int(l[0])] = (
                            state_id,
                            state_index,
                            state_index + sorts_dict[int(l[2])],
                        )
                    state_index += sorts_dict[int(l[2])]
                state_id += 1

    btor2_cex_filepath = output_dir + input_file_name + ".cexb"

    # for key in input_dict:
    #     print(
    #         f"input {key} aig ({input_dict[key][0]},{input_dict[key][1]},{input_dict[key][2]})"
    #     )
    # for key in nonnext_state_dict:
    #     print(
    #         f"state as input {key} aig ({nonnext_state_dict[key][0]},{nonnext_state_dict[key][1]},{nonnext_state_dict[key][2]})"
    #     )
    # for key in uninit_state_dict:
    #     print(
    #         f"state {key} aig ({uninit_state_dict[key][0]},{uninit_state_dict[key][1]},{uninit_state_dict[key][2]})"
    #     )

    # Write Btor2 CEX file
    if verbose:
        print("Writing output file.")
    with open(aig_cex_filepath, "r") as aig_cex_file:
        next(aig_cex_file)  # 1
        next(aig_cex_file)  # b0
        with open(btor2_cex_filepath, "w+") as btor2_cex_file:
            btor2_cex_file.write("sat\n")
            btor2_cex_file.write("b0\n")
            frame = 0
            init_output = True
            init_output_cache = {}
            for line in aig_cex_file:
                line = line.strip()
                line = line.replace("x", "0")
                if line == ".":
                    btor2_cex_file.write(".\n")
                    break
                if init_output:
                    for key in uninit_state_dict:
                        init_output_cache[key] = (
                            f"{uninit_state_dict[key][0]} {line[uninit_state_dict[key][1]:uninit_state_dict[key][2]]}\n"
                        )
                    init_output = False
                    continue
                # state
                btor2_cex_file.write(f"#{frame}\n")
                if frame == 0:
                    for key in nonnext_state_dict:
                        init_output_cache[key] = (
                            f"{nonnext_state_dict[key][0]} {line[nonnext_state_dict[key][1]:nonnext_state_dict[key][2]]}\n"
                        )
                    for key in sorted(init_output_cache.keys()):
                        btor2_cex_file.write(init_output_cache[key])
                else:
                    for key in nonnext_state_dict:
                        btor2_cex_file.write(
                            f"{nonnext_state_dict[key][0]} {line[nonnext_state_dict[key][1]:nonnext_state_dict[key][2]]}\n"
                        )
                btor2_cex_file.write(f"@{frame}\n")
                # input
                for key in input_dict:
                    btor2_cex_file.write(
                        f"{input_dict[key][0]} {line[input_dict[key][1]:input_dict[key][2]]}\n"
                    )
                frame += 1


def main():
    if verbose:
        print("Solve Array-Free Btor2 as Aiger")

    simplecar_executable = sys.argv[1]
    simplecar_args = sys.argv[2:]

    input_file = None
    output_file = None
    simplecar_command_args = [simplecar_executable]

    # Parse arguments to find input and output files
    i = 0
    while i < len(simplecar_args):
        arg = simplecar_args[i]
        if arg == "-w" and i + 1 < len(simplecar_args):
            output_dir = simplecar_args[i + 1]
            simplecar_command_args.append(arg)
            simplecar_command_args.append(output_dir)
            i += 2
        elif not arg.startswith("-") and os.path.exists(arg):
            input_file = arg
            i += 1
        else:
            simplecar_command_args.append(arg)
            i += 1

    if not input_file:
        print("Error: Input file not specified or does not exist.")
        sys.exit(1)

    if not output_dir:
        print("Warning: Output file not specified.")
        sys.exit(1)

    # Modify the input file
    input_file_aig = modify_input_file(input_file)

    # Call the simplecar executable with the original arguments
    simplecar_command_args.append(input_file_aig)
    if verbose:
        print(f"Running simplecar with command: {' '.join(simplecar_command_args)}")
    try:
        result = subprocess.run(
            simplecar_command_args,
            capture_output=True,  # Capture stdout and stderr
            text=True,  # Decode output as text
            check=True,  # Raise an exception if simplecar returns a non-zero exit code
        )
        simplecar_result = result.stdout.strip()
        if verbose:
            print("simplecar finished successfully. Output:")
        print(simplecar_result)

    except FileNotFoundError:
        print(f"Error: simplecar executable not found at {simplecar_executable}.")
        sys.exit(1)
    except subprocess.CalledProcessError as e:
        print(f"Error: simplecar exited with a non-zero status code: {e.returncode}.")
        print(f"simplecar stderr:\n{e.stderr}")
        sys.exit(e.returncode)
    except KeyboardInterrupt:
        if verbose:
            print("Interrupt received. The simplecar process was terminated.")
        sys.exit(1)

    # Modify the output file after simplecar finishes
    modify_output_file(simplecar_result, input_file, output_dir)


if __name__ == "__main__":
    main()
