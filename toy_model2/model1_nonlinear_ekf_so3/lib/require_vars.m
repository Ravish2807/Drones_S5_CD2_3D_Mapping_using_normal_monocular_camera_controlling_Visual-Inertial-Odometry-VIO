function S = require_vars(matfile, names, producer)
%REQUIRE_VARS  Loads variables saved by an earlier step and checks they exist.
%   INPUT : matfile (char), names cellstr, producer (char) step that creates them
%   OUTPUT: S struct holding the requested variables
if ~isfile(matfile)
    error('%s not found. Run %s first.', matfile, producer);
end
S = load(matfile);
for i = 1:numel(names)
    if ~isfield(S, names{i})
        error('Missing "%s" in %s. Run %s first.', names{i}, matfile, producer);
    end
end
end
