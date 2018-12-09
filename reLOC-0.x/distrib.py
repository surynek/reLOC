# Distribute - Distribution Generator
# Version: 1.0
# (C) Copyright 2018 Pavel Surynek
# http://www.surynek.com
# pavel@surynek.com

import fileinput
import string
import shutil
import os
import sys

class ModuleRecord:
  pass

def print_intro():
  print "Distribute 1.0 - Distribution Generator"
  print "(C) Copyright 2017 Pavel Surynek"
  print "---------------------------------------"

include_dirs = set()
program_dirs = set()
modules = list()
selected_modules = list()

def get_first_word(line):
  begin = -1
  end = -1
  for i in range(len(line)):
    if begin == -1:
      if string.find(string.whitespace, line[i]) < 0:
        begin = i
    else:
      if end == -1:
        if string.find(string.whitespace, line[i]) >= 0:
          end = i
          break
  if begin == -1:
    return ""
  return line[begin:end]


def get_remaining_words(line):
  begin = -1
  end = -1
  for i in range(len(line)):
    if begin == -1:
      if string.find(string.whitespace, line[i]) < 0:
        begin = i
    else:
      if end == -1:
        if string.find(string.whitespace, line[i]) >= 0:
          end = i
      else:
        if string.find(string.whitespace, line[i]) < 0:
          end = i
          break
  if begin == -1:
    return ""
  return line[end:len(line)]

              
def load_modules(modules_file):
  while True:
    module_record = ModuleRecord()
    module_identifier = modules_file.readline()
    
    while module_identifier != "" and get_first_word(module_identifier) == "":
      module_identifier = modules_file.readline()
      
    if module_identifier == "":
      break

    module_record.identifier = get_first_word(module_identifier)

    module_type = modules_file.readline()

    if string.find(module_type, "program") == 0:
      module_record.type = "program"
      module_directory = modules_file.readline()
      module_record.directory = get_first_word(module_directory)
      program_dirs.add(module_record.directory)

      module_headers = modules_file.readline()
      module_record.headers = list()

      header = get_first_word(module_headers)
      while header != "":
        module_headers = get_remaining_words(module_headers)
        module_record.headers.append(header)
        include_dirs.add(module_record.directory)
        header = get_first_word(module_headers)
      
      module_sources = modules_file.readline()
      module_record.sources = list()

      source = get_first_word(module_sources)
      while source != "":
        module_sources = get_remaining_words(module_sources)
        module_record.sources.append(source)
        source = get_first_word(module_sources)      
      
    elif string.find(module_type, "files") == 0:
      module_record.type = "files"
      module_directory = modules_file.readline()
      module_record.directory = get_first_word(module_directory)

      module_files = modules_file.readline()
      module_record.files = list()

      file = get_first_word(module_files)
      while file != "":
        module_files = get_remaining_words(module_files)
        module_record.files.append(file)
        file = get_first_word(module_files)
        
    else:
      print "Error: Unrecognized module type: " + module_type
      break;

    modules.append(module_record)


def load_selection(selection_file):
  for line in selection_file:
    module = get_first_word(line)
    selected_modules.append(module)

  for md in modules:
    sel = False
    for smd in selected_modules:
      if smd == md.identifier:
        sel = True
    md.selected = sel


def extract_directory(path):
  start = 0

  pos = string.find(path, "/", start)
  if pos < 0:
    return ""

  while pos >= 0:
    start = pos + 1
    pos = string.find(path, "/", start)
    
  return path[0:start-1]


def create_distribution(main_directory):
  files = set()
  for md in modules:
    if md.selected:
      if md.type == "program":
        for hd in md.headers:
          if hd[0] == '/':
            files.add(hd[1:len(hd)])
          else:
            if md.directory != "/":
              files.add(md.directory[1:len(md.directory)] + "/" + hd)
            else:
              files.add(hd)
        for sr in md.sources:
          if sr[0] == '/':
            files.add(sr[1:len(sr)])
          else:
            if md.directory != "/":
              files.add(md.directory[1:len(md.directory)] + "/" + sr)
            else:
              files.add(sr)
      elif md.type == "files":
        for fl in md.files:
          if fl[0] == '/':
            files.add(fl[1:len(fl)])
          else:
            if md.directory != "/":
              files.add(md.directory[1:len(md.directory)] + "/" + fl)
            else:
              files.add(fl)

  top_distribution_dir = "../distribution/" + main_directory
  if os.path.isdir(top_distribution_dir):
    shutil.rmtree(top_distribution_dir)

  for pt in files:
    print "Processing ... " + pt
    pt2 = "../distribution/" + main_directory + "/" + extract_directory(pt)
    pt3 = "../distribution/" + main_directory + "/" + pt

    if not os.path.isdir(pt2):
      os.makedirs(pt2)

    shutil.copy(pt, pt3)


def get_main_directory(product, version):
  return product + "-" + version


print_intro()

modules_file = open("modules", "r")
load_modules(modules_file)
modules_file.close()

selection_file = open(sys.argv[1], "r")
load_selection(selection_file)
selection_file.close()

version_file = open("version", "r")
version = version_file.readline()
version_file.close()

product_file = open("product", "r")
product = product_file.readline()
product_file.close()

main_directory = get_main_directory(product, version)
create_distribution(main_directory)
