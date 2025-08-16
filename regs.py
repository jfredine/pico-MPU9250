#!/usr/bin/env python3
"Transform file of register address and register name to a header file"
import sys
import argparse

def parse_regs_file(in_f):
    # read the regs.txt file and store the information
    regs = []
    for line in in_f:
        line = line.strip()
        if line == "":
            continue
        tokens = line.split()
        regs.append({"name": tokens[0],
                     "val": int(tokens[1], 16)})

    return regs


def get_max_name_len(regs):
    reg = max(regs, key=lambda item: len(item["name"]))
    return len(reg["name"])


def create_defines(out_f, regs):
    max_name_len = get_max_name_len(regs)

    # create defines 
    for reg in regs:
        out_f.write("#define ")
        out_f.write(f"{reg["name"]:<{max_name_len}} 0x{reg["val"]:X}\n")


def create_regs_array(out_f, regs):

    out_f.write("typedef struct reg_s {\n")
    out_f.write("    const char *name;\n")
    out_f.write("    uint       val;\n")
    out_f.write("} reg_t;\n\n")

    # create regs array
    max_name_len = get_max_name_len(regs)
    out_f.write("reg_t regs[] = {\n")
    first = True
    for reg in regs:
        if not first:
            out_f.write(",\n")
        first = False
        out_f.write(f"    {{\"{reg["name"]}\",{" " * (max_name_len - len(reg["name"]))} {reg["name"]}}}")
    out_f.write("\n};\n")


def main():
    description="""Generate header information for registers"""
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('regs_file', nargs='?', default=None,
                        help="Register specification file")
    parser.add_argument('--output', '-o', default=None,
                        help="Specify output file (default: stdout)")
    args = parser.parse_args()

    if args.regs_file is None:
        in_f = sys.stdin
    else:
        in_f = open(args.regs_file)

    if args.output is None:
        out_f = sys.stdout
    else:
        out_f = open(args.output, "w")

    regs = parse_regs_file(in_f)
    if in_f is not sys.stdin:
        in_f.close()

    create_defines(out_f, regs)
    out_f.write("\n");
    create_regs_array(out_f, regs)

    if out_f is not sys.stdout:
        out_f.close()


if __name__ == "__main__":
    main()
