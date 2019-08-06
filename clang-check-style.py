#!/usr/bin/env python3

# This script borrows heavily from:
# https://github.com/ament/ament_lint/commit/42dd47df0d3eb8a2755bd639ebb1c6169b4a40db

# Copyright 2014-2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import subprocess
import sys
from xml.etree import ElementTree

def find_executable(file_names):
    paths = os.getenv('PATH').split(os.path.pathsep)
    for file_name in file_names:
        for path in paths:
            file_path = os.path.join(path, file_name)
            if os.path.isfile(file_path) and os.access(file_path, os.X_OK):
                return file_path
    return None


def get_files(paths, extensions):
    files = []
    for path in paths:
        if os.path.isdir(path):
            for dirpath, dirnames, filenames in os.walk(path):
                if 'AMENT_IGNORE' in filenames:
                    dirnames[:] = []
                    continue
                # ignore folder starting with . or _
                dirnames[:] = [d for d in dirnames if d[0] not in ['.', '_']]
                dirnames.sort()

                # select files by extension
                for filename in sorted(filenames):
                    _, ext = os.path.splitext(filename)
                    if ext in ['.%s' % e for e in extensions]:
                        files.append(os.path.join(dirpath, filename))
        if os.path.isfile(path):
            files.append(path)
    return [os.path.normpath(f) for f in files]


def find_index_of_line_start(data, offset):
    index_1 = data.rfind('\n', 0, offset) + 1
    index_2 = data.rfind('\r', 0, offset) + 1
    return max(index_1, index_2)


def find_index_of_line_end(data, offset):
    index_1 = data.find('\n', offset)
    if index_1 == -1:
        index_1 = len(data)
    index_2 = data.find('\r', offset)
    if index_2 == -1:
        index_2 = len(data)
    return min(index_1, index_2)


def get_line_number(data, offset):
    return data[0:offset].count('\n') + data[0:offset].count('\r') + 1


def main():
    # First find the executable to run
    bin_names = [
        'clang-format',
        'clang-format-3.8',
        'clang-format-6.0',
    ]
    clang_format_bin = find_executable(bin_names)
    if not clang_format_bin:
        print('Could not find %s executable' % ' / '.join(["'%s'" % n for n in bin_names]), file=sys.stderr)
        return 1

    # Now find the list of files to run it against
    extensions = ['cpp', 'h']
    files = get_files('.', extensions)

    # Now build up the command
    cmd = [clang_format_bin,
           '-output-replacements-xml',
           '-style=file']
    cmd.extend(files)

    # Run the command, collecting the output
    try:
        output = subprocess.check_output(cmd)
    except subprocess.CalledProcessError as e:
        print("The invocation of '%s' failed with error code %d: %s" %
              (os.path.basename(clang_format_bin), e.returncode, e),
              file=sys.stderr)
        return 1

    # Output errors
    report = {}
    for filename in files:
        report[filename] = []

    xmls = output.split(b"<?xml version='1.0'?>")[1:]
    for filename, xml in zip(files, xmls):
        try:
            root = ElementTree.fromstring(xml)
        except ElementTree.ParseError as e:
            print('Invalid XML in clang format output: %s' % str(e),
                  file=sys.stderr)
            return 1

        replacements = root.findall('replacement')
        if replacements:
            print("Code style divergence in file '%s':" % filename,
                  file=sys.stderr)

            with open(filename, 'r') as h:
                content = h.read()
            for replacement in replacements:
                data = {
                    'offset': int(replacement.get('offset')),
                    'length': int(replacement.get('length')),
                    'replacement': replacement.text or '',
                }
                # to-be-replaced snippet
                data['original'] = content[
                    data['offset']:data['offset'] + data['length']]
                # map global offset to line number and offset in line
                index_of_line_start = find_index_of_line_start(
                    content, data['offset'])
                index_of_line_end = find_index_of_line_end(
                    content, data['offset'] + data['length'])
                data['line_no'] = get_line_number(content, index_of_line_start)
                data['offset_in_line'] = data['offset'] - index_of_line_start

                # generate diff like changes
                subcontent = content[
                    index_of_line_start:index_of_line_end]
                data['deletion'] = subcontent
                data['addition'] = subcontent[0:data['offset_in_line']] + \
                    data['replacement'] + \
                    subcontent[data['offset_in_line'] + data['length']:]

                # make common control characters visible
                mapping = {'\n': '\\n', '\r': '\\r', '\t': '\\t'}
                for old, new in mapping.items():
                    data['replacement'] = data['replacement'].replace(old, new)
                    data['original'] = data['original'].replace(old, new)

                mapping = {'\r': '\n'}
                for old, new in mapping.items():
                    data['deletion'] = data['deletion'].replace(old, new)
                    data['addition'] = data['addition'].replace(old, new)

                # format deletion / addition as unified diff
                data['deletion'] = '\n'.join(
                    ['- ' + l for l in data['deletion'].split('\n')])
                data['addition'] = '\n'.join(
                    ['+ ' + l for l in data['addition'].split('\n')])

                report[filename].append(data)

                data = dict(data)
                data['filename'] = filename
                print('[%(filename)s:%(line_no)d:%(offset_in_line)d]: '
                      'Replace [%(original)s] with [%(replacement)s]' %
                      data, file=sys.stderr)
                print(data['deletion'], file=sys.stderr)
                print(data['addition'], file=sys.stderr)
            print('', file=sys.stderr)
        else:
            print("No code style divergence in file '%s'" % filename)

    file_count = sum(1 if report[k] else 0 for k in report.keys())
    replacement_count = sum(len(r) for r in report.values())
    if not file_count:
        print('No problems found')
        rc = 0
    else:
        print('%d files with %d code style divergences' %
              (file_count, replacement_count), file=sys.stderr)
        rc = 1

    return rc


if __name__ == '__main__':
    sys.exit(main())
