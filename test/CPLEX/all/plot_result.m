clear all
close all
clc

load test_result.mat

unfeasible_cpp = find(solution_flag_cpp~=1);
unfeasible_matlab = find(solution_flag_matlab~=1);
feasible_cpp = find(solution_flag_cpp==1);
feasible_matlab = find(solution_flag_matlab==1);

num_unfeasible_cpp = length(unfeasible_cpp);
num_unfeasible_matlab = length(unfeasible_matlab);
num_feasible_cpp = length(feasible_cpp);
num_feasible_matlab = length(feasible_matlab);

disp(['Total feasible problems (CPP): ', mat2str(length(feasible_cpp))]);
disp(['Total unfeasible problems (CPP): ', mat2str(length(unfeasible_cpp))]);
disp(['Total feasible problems (matlab): ', mat2str(length(feasible_matlab))]);
disp(['Total unfeasible problems (matlab): ', mat2str(length(unfeasible_matlab))]);

solution_index = 1:1:length(solution_error);
figure,plot(solution_index(find(solution_error>0)),solution_error(find(solution_error>0)),'x'),grid
