disp("Setting up matlab environment and generating code...")
addpath("matlab/")

out_dir = fullfile(fileparts(mfilename('fullpath')), 'matlab/generated');
if ~isfolder(out_dir)
    mkdir(out_dir);
end
addpath("matlab/generated")

def_optimal_control
def_stop_primitive
def_stop_primitive_j0

generate_code

clear; clc; close all;
disp("Paths added and code generated");
