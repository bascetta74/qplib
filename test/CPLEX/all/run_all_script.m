clear all
close all
clc

numScript = 1000;

for k=1:numScript
    scriptName = ['./script/test_', mat2str(k), '_script.m'];
    run(scriptName);
    
    outputName = ['./output/test_', mat2str(k), '_script.mat'];
    save(outputName);
end

clearvars -EXCEPT numScript

solution_error = -ones(1,numScript);
for k=1:numScript
    outputName = ['./output/test_', mat2str(k), '_script.mat'];
    script_output = load(outputName);
    
    if (script_output.exitflag==1) & (script_output.exitflag_cpp==1)
        solution_error(k) = norm(script_output.x-script_output.x_cpp);
    end
    solution_flag_matlab(k) = script_output.exitflag;
    solution_flag_cpp(k) = script_output.exitflag_cpp;
end

save test_result.mat solution_flag_matlab solution_flag_cpp solution_error
