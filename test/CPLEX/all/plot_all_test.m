clear all
close all
clc

% Parameters
folders = {
    'cost_function',
    'cost_function+INconstr',
    'cost_function+INconstr+QINconstr',
    'cost_function+LUbound',
    'cost_function+LUbound+INconstr',
    'cost_function+LUbound+INconstr+EQconstr',
    'cost_function+LUbound+INconstr+QINconstr',
    'cost_function+LUbound+INconstr+EQconstr+QINconstr',
};

for j=1:length(folders)
    fileName = ['../', folders{j}, '/test_result.mat'];
    load(fileName)
    
    solution_index = 1:1:length(solution_error);
    figure,plot(solution_index(find(solution_error>0)),solution_error(find(solution_error>0)),'x'),grid,...
        xlabel('Problem number'),ylabel('Error'),title(strrep(folders{j},'_',' '))
    
    clearvars -EXCEPT folders j
end
