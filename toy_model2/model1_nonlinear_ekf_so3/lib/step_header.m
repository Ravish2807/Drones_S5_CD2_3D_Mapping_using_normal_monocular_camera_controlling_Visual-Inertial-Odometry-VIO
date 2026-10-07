function step_header(model, n, title, eqs)
%STEP_HEADER  Prints a framed banner for a toy-model step.
%   INPUT : model number, n step number, title (char), eqs cellstr of equations
bar = repmat('=', 1, 78);
fprintf('\n%s\n  MODEL %d | STEP %d | %s\n%s\n', bar, model, n, upper(title), bar);
for i = 1:numel(eqs)
    fprintf('    %s\n', eqs{i});
end
fprintf('%s\n', repmat('-', 1, 78));
end
