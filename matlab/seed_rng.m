function seed_rng(m)
% Seed both rand and randn (MATLAB and Octave).
if exist('OCTAVE_VERSION','builtin')
    rand('state',m); randn('state',m);
else
    rng(m);
end
end
