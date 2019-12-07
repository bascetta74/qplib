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

numScript = 1000;

for j=1:length(folders)
    for k=1:numScript
        scriptName = ['../', folders{j}, '/script/test_', mat2str(k), '.m'];
        run(scriptName);
        
        outputName = ['../', folders{j}, '/output/test_', mat2str(k), '.mat'];
        save(outputName);
        
        clearvars -EXCEPT numScript folders j
    end
    
    solution_error = -ones(1,numScript);
    for k=1:numScript
        outputName = ['../', folders{j}, '/output/test_', mat2str(k), '.mat'];
        script_output = load(outputName);
        
        if (script_output.exitflag==1) & (script_output.exitflag_cpp==1)
            solution_error(k) = norm(script_output.x-script_output.x_cpp);
        end
        solution_flag_matlab(k) = script_output.exitflag;
        solution_flag_cpp(k) = script_output.exitflag_cpp;
    end
    
    outputName = ['../', folders{j}, '/test_result.mat'];
    save(outputName, 'solution_flag_matlab', 'solution_flag_cpp', 'solution_error');
    
    clearvars -EXCEPT numScript folders j
end
