#!/usr/bin/python

#######################################################################
#
# INTEL CONFIDENTIAL
# Copyright 2009-2019 Intel Corporation All Rights Reserved.
# 
# The source code contained or described herein and all documents related to the
# source code ("Material") are owned by Intel Corporation or its suppliers or
# licensors. Title to the Material remains with Intel Corporation or its
# suppliers and licensors. The Material may contain trade secrets and proprietary
# and confidential information of Intel Corporation and its suppliers and
# licensors, and is protected by worldwide copyright and trade secret laws and
# treaty provisions. No part of the Material may be used, copied, reproduced,
# modified, published, uploaded, posted, transmitted, distributed, or disclosed
# in any way without Intel's prior express written permission.
# 
# No license under any patent, copyright, trade secret or other intellectual
# property right is granted to or conferred upon you by disclosure or delivery
# of the Materials, either expressly, by implication, inducement, estoppel or
# otherwise. Any license under such intellectual property rights must be
# express and approved by Intel in writing.
# 
# Unless otherwise agreed by Intel in writing, you may not remove or alter this
# notice or any other notice embedded in Materials by Intel or Intel's suppliers
# or licensors in any way.
# 
#  version: SDK-jenkins-FlexRAN-SDK-REL-448-g3be238
#
#######################################################################

"""This script converts mix file created by SDE to IACA file
using IACA to convert blocks in mix file
"""

import logging
import sys
import argparse
import re
import subprocess
import os
from datetime import datetime


def init_logger(console_level, logfile_level):
    """Initializes console and logfile logger with given logging levels"""
    # File logger
    logging.basicConfig(filename="mix_to_iaca.log",
                        filemode='w',
                        format="%(asctime)s: %(levelname)s: %(message)s",
                        level=logfile_level)
    # Console logger
    logger = logging.getLogger()
    handler = logging.StreamHandler()
    handler.setLevel(console_level)
    formatter = logging.Formatter("%(levelname)s: %(message)s")
    handler.setFormatter(formatter)
    logger.addHandler(handler)


def parse_args(args):
    """Configures parser and parses command line configuration"""
    # Parser configuration
    parser = argparse.ArgumentParser(description="Convert mix file to IACA file")
    parser.add_argument("input_files", nargs="+", help="input file names")
    parser.add_argument("--arch", action="store", default="SKX",
                        help="architecture: SKX/SNB/SKL/...", metavar="ARCH", dest="arch_name")
    parser.add_argument("--out", action="store", default="",
                        help="output file name", metavar="FILE", dest="output_file")
    # Parse arguments
    options = parser.parse_args(args)
    logging.debug("Options: input file names=%s, architecture=%s, output file name=%s",
                  options.input_files, options.arch_name, options.output_file)
    return options


def read_mix_file(input_file_name):
    """Reads data from input file"""
    logging.info("Read mix file: %s", input_file_name)
    with open(input_file_name, 'r') as input_file:
        data = input_file.readlines()
    return data


def create_iaca_input(content, file_name):
    """Creates iaca input file with given content"""
    # Markers IACA_START and IACA_END compiled to binary format
    iaca_start = "BB6F000000646790"
    iaca_end = "BBDE000000646790"
    # Put content wrapped in markers to file
    iaca_content = iaca_start + content + iaca_end
    logging.debug("Create IACA input file %s", file_name)
    iaca_content_hex = iaca_content.decode("hex")
    with open(file_name, "wb") as input_file:
        input_file.write(iaca_content_hex)

def call_iaca_cmd(iaca_cmd):
    """Call IACA with defined command and returns output"""
    try:
        out = subprocess.check_output(iaca_cmd, shell=True)
    except subprocess.CalledProcessError as subprocess_error:
        logging.error("Calling IACA failed with error: %d %s",
                      subprocess_error.returncode,
                      subprocess_error.output)
        if subprocess_error.returncode == 127:
            sys.exit("IACA not found. Set environment variable IACA_PATH to IACA binary path.")
        else:
            sys.exit("Calling IACA failed, returned code {0}".format(subprocess_error.returncode))
    if not out:
        logging.error("Empty output from calling IACA")
        sys.exit("Empty output from calling IACA")
    return out


def create_iaca_output(iaca_path, arch_name, iaca_input):
    """Creates IACA output using given parameters"""
    cmd = "{0} -arch {1} -reduceout '{2}'".format(iaca_path, arch_name, iaca_input)
    logging.debug("Call IACA: %s", cmd)
    out = call_iaca_cmd(cmd)

    data = out.splitlines()
    return data


def process_block(block_data, block_num, options_arch_name, block_address_instruction, iaca_path):
    """Processes a block found in mix file"""
    logging.debug("Process block: %d", block_num)
    # Find all "XDIS..." lines and extract data from each line
    block_line_regex = re.compile(r"XDIS ([\dabcdef]+):\s+\w+ ([\dABCDEF]+)(.+)")
    input_content = ""
    for line in block_data:
        match = block_line_regex.match(line)
        if match:
            address = match.group(1).lstrip("0")
            encode = match.group(2)
            instruction = match.group(3).strip()
            input_content += encode
            block_address_instruction.append((address, instruction))

    if input_content == "":
        logging.error("No data to process")
        return None

    # IACA temp input file name (input data for each block)
    block_input_filename = "iaca_input.bin"
    # Prepare input file and call IACA
    create_iaca_input(input_content, block_input_filename)
    data = create_iaca_output(iaca_path, options_arch_name, block_input_filename)
    return data


def insert_address_instruction_data(block_iaca_data, block_address_instruction):
    """Injects address and instruction to block data"""
    i = 0
    # Find line with table title
    table_title = "Ports pressure in cycles"
    while not table_title in block_iaca_data[i]:
        i += 1
    # 3 lines below title starts data in table
    i += 3
    if i >= len(block_iaca_data):
        logging.error("Block iaca data is too short")
        return block_iaca_data
    # Extract  address & instruction
    for j, (address, instruction) in enumerate(block_address_instruction):
        last_char = block_iaca_data[i+j].rfind('|')
        block_iaca_data[i+j] = "{0} 0x{1}: {2}".format(block_iaca_data[i+j][:last_char+1],
                                                       address,
                                                       instruction)
    return block_iaca_data


def get_bottleneck_throughput(data):
    """Finds values of bottleneck and throughput in block data"""
    throughput_regex = r"Block Throughput: (\d+\.\d+) Cycles"
    bottleneck_regex = r"Throughput Bottleneck: (.+)"

    data_str = "\n".join(data)
    match1 = re.search(throughput_regex, data_str)
    match2 = re.search(bottleneck_regex, data_str)
    if match1 and match2:
        throughput = float(match1.group(1))
        bottleneck = match2.group(1)
        return bottleneck, throughput

    logging.error("Block throughput and bottleneck not found")
    return None, None


def analyze_block(block_num, output, loop_executions, loop_percent, is_loop, iaca_summary):
    """Does block analysis to prepare data for IACA summary"""
    bottleneck, throughput = get_bottleneck_throughput(output)
    cycle_cnt_est = int(loop_executions * throughput)
    logging.debug("Block %d analysis results: bottleneck %s throughput %f cycle cnt estimate %d",
                  block_num, bottleneck, throughput, cycle_cnt_est)
    iaca_summary.append((cycle_cnt_est, loop_percent, bottleneck, block_num, is_loop))

    return cycle_cnt_est


def prepare_block_data(start_line, output, cycle_cnt_estimate):
    """Combines together block information needed in output file"""
    logging.debug("Prepare block data: %s", start_line[:15])
    # Block data contains:
    # block starting line from mix file, iaca output and line with cycle cnt estimate
    output_str = "\n".join(output)
    block_data = [start_line,
                  output_str,
                  "\nCycle cnt estimate: {0} cycles\n\n".format(cycle_cnt_estimate)]
    return block_data


def insert_iaca_summary(output_file_name, iaca_summary):
    """Prepares and saves to file IACA summary"""
    if not iaca_summary:
        logging.error("Empty iaca summary data")
        return -1

    # Unzip iaca_summary to extract data for summary table
    all_cycle_est, all_percents, _, _, _ = zip(*iaca_summary)
    # Prepare summary
    total_cycle_estimate = sum(all_cycle_est)
    summary = ["{0:<9d}".format(block_num) +
               "{0:<9s}".format("{0:.2f}%".format(loop_percent)) +
               "{0:<7s}".format("LOOP" if is_loop else "X") +
               "{0:<40s}".format(bottleneck) +
               "    {0:<1d}  ".format(cycle_est) +
               "({0:.2f}%)\n".format(cycle_est * 100.0 / total_cycle_estimate)
               for (cycle_est, loop_percent, bottleneck, block_num, is_loop)
               in sorted(iaca_summary, reverse=True)]

    summary_header = "BlockID".ljust(9) + "Inst%" .ljust(9) + "Loop?".ljust(7) + \
                     "Bottleneck".ljust(40) + "    Cycle cnt estimate (%)\n"
    summary.insert(0, "\n====================================== IACA Summary ====================="
                   "==================\n")
    summary.insert(1, summary_header)
    summary.insert(2, "---------------------------------------------------------------------------"
                   "----------------\n")
    summary.append("------------------------------------------------------------------------------"
                   "-------------\n")
    summary.append("{0:<18s}".format("Total:   {0:.2f}%".format(sum(all_percents))) +
                   "Cycle cnt estimate: {0} ({1:.2f}%)\n".format(total_cycle_estimate, 100))
    summary.append("=============================================================================="
                   "=============\n\n")

    # Save summary to output file
    with open(output_file_name, "a") as output_file:
        output_file.writelines(summary)

    # Cycle cnt estimates in % needed for updating block sections (for all blocks)
    cycle_est_percent = [x * 100.0 / total_cycle_estimate for x in all_cycle_est]
    return cycle_est_percent


def save_data(output_file_name, iaca_summary, iaca_blocks_data):
    """Saves data to IACA file"""
    logging.info("Save iaca data to output file %s", output_file_name)
    # First save summary
    cycle_est_percent = insert_iaca_summary(output_file_name, iaca_summary)
    # Save block (adding % cycle cnt estimates)
    with open(output_file_name, "a") as output_file:
        for i, block in enumerate(iaca_blocks_data):
            block[0] = block[0][:-1] + " cyc%%: %.2f\n" % cycle_est_percent[i]
            output_file.writelines(block)


def convert_to_iaca(data, options_arch_name, output_file_name, iaca_path):
    """Converts mix file data block by block and creates IACA file"""
    logging.info("Conversion to IACA started")
    # Mix file contains sections for blocks
    start_block_regex = re.compile(r"BLOCK:\s+(\d+)\s+PC:\s+\w+\s+ICOUNT:\s+\d+\s+"
                                   r"EXECUTIONS:\s+(\d+)\s+#BYTES:\s+\d+\s+%:\s+(\d+\.?\d*)")
    end_block_regex = re.compile(r"\n")
    end_top_stats_regex = re.compile(r"# END_TOP_BLOCK_STATS")

    block_num = -1
    start = 0
    block_address_instruction = []
    iaca_summary = []
    iaca_blocks_data = []
    loop_executions = 0
    loop_percent = 0

    for line_number, line in enumerate(data):
        # End of blocks section
        if not end_top_stats_regex.match(line):
            # Find beginning of the block
            match = start_block_regex.match(line)
            if match:
                block_num = int(match.group(1))
                loop_executions = int(match.group(2))
                loop_percent = float(match.group(3))
                start = line_number
                logging.debug("Block %d start line: %d, loop executions %d loop percent %f",
                              block_num, start, loop_executions, loop_percent)
            else:
                # Find end of the block and process the block
                # Code for find block beginning must have been already executed
                # start, block_num, loop_executions, loop_percent are set
                if end_block_regex.match(line) and start != 0:
                    end = line_number
                    logging.debug("Block %d end line: %d", block_num, end)
                    # Process found block
                    output = process_block(data[start:end], block_num, options_arch_name,
                                           block_address_instruction, iaca_path)

                    # Analysis of block
                    # Inject addresses and instructions to output
                    insert_address_instruction_data(output, block_address_instruction)
                    # Check if block is a loop: first column is address, second is instruction
                    # Check if first address is in last instruction
                    first_address = block_address_instruction[0][0]
                    last_instruction = block_address_instruction[-1][1]
                    is_loop = first_address in last_instruction
                    # Calculate cycle cnt estimation and update iaca summary data
                    cycle_cnt_estimation = analyze_block(block_num, output, loop_executions,
                                                         loop_percent, is_loop, iaca_summary)
                    # Save block data
                    iaca_blocks_data.append(prepare_block_data(data[start], output,
                                                               cycle_cnt_estimation))

                    start = 0
                    block_address_instruction = []
        else:
            # End of blocks section
            break

    save_data(output_file_name, iaca_summary, iaca_blocks_data)


def main():
    """Processes input files to produce IACA files"""
    # Find path to IACA
    iaca_path = os.getenv("IACA_PATH", "/opt/iaca/iaca-lin64/iaca")
    # Check if IACA is working and get IACA version as string
    iaca_version = call_iaca_cmd("{0} -v".format(iaca_path))

    # Set up logging with given level (DEBUG, INFO, ERROR) for console end logfile
    init_logger(logging.INFO, logging.DEBUG)
    logging.info("Started script: mix_to_iaca.py, IACA path %s", iaca_path)

    # Parse input arguments
    options = parse_args(sys.argv[1:])

    file_no = 0
    for input_file_name in options.input_files:
        # Read input file
        data = read_mix_file(input_file_name)

        # Prepare output file name
        if options.output_file != "":
            output_file_name = options.output_file
            if file_no != 0:
                name, extension = os.path.splitext(output_file_name)
                output_file_name = name + str(file_no) + extension
        else:
            name, extension = os.path.splitext(input_file_name)
            output_file_name = name + ".iaca"
        logging.debug("Output file name: " + output_file_name)
        # Write IACA version to output file
        with open(output_file_name, "w") as output_file:
            output_file.write(iaca_version)
        # In case more than 1 input file and output file name defined
        # number will be added to output file name
        file_no += 1

        # Process input file
        convert_to_iaca(data, options.arch_name, output_file_name, iaca_path)


if __name__ == '__main__':
    START_TIME = datetime.now()
    main()
    END_TIME = datetime.now()
    logging.debug("Start time: %s, end time: %s", START_TIME, END_TIME)
    logging.info("Execution time: %s", END_TIME - START_TIME)
    logging.shutdown()
    sys.exit(0)
