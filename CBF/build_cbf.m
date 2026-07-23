function [A, b, a] = build_cbf(h, f, g, x, u, opts)

arguments
    h sym
    f sym
    g sym
    x sym
    u sym
    opts.TimeVarying (1,1) logical = false
    opts.TimeVaryingVars sym = [];
    opts.TimeVaryingFcn sym = [];
    opts.InputConstrained (1,1) logical = false
    opts.InputLimits sym = []
end

%% Pre-conditions

assert(isequal(size(f), size(x)), "The state drift function must be the same size as the state.")
assert(isequal(width(g), height(u)), "The width of the state control function must be the same size as the number of control inputs.")
assert(isequal(height(g), height(x)), "The height of the state control function must be the same size as the number of states.")

assert(~opts.TimeVarying || ~isempty(opts.TimeVaryingVars), "TimeVaryingParams must be specified when TimeVarying is true.")
assert(~opts.TimeVarying || ~isempty(opts.TimeVaryingFcn), "TimeVaryingFcn must be specified when TimeVarying is true.")
assert(~opts.TimeVarying || isequal(size(opts.TimeVaryingVars), size(opts.TimeVaryingFcn)), "The size of TimeVaryingVars and TimeVaryingFcn must be the same.")

assert(~opts.InputConstrained || ~isempty(opts.InputLimits), "InputLimits must be specified when InputConstrained is true.")
assert(~opts.InputConstrained || isequal(size(opts.InputLimits), size(u)), "The size of InputLimits and u must be the same.")

%% Build CBF

if opts.TimeVarying
    % Add the time varying variables as a "state" whose drift function 
    % describes the time evolution but are not controllable.
    x = [x; opts.TimeVaryingVars]; 
    f = [f; opts.TimeVaryingFcn];
    g = [g; zeros(height(opts.TimeVaryingVars), height(u))];
end

Lf = @(h) jacobian(h, x') * f; % Lie derivative wrt. f;
Lg = @(h) jacobian(h, x') * g; % Lie derivative wrt. g

% Recursively apply invariance condition until we get a sub-set of the 
% safe-set we can be guaranteed to stay inside (HOCBF).
bN = h;
LfbN = Lf(bN);
LgbN = Lg(bN);

k = 1;

% Initialize class-k gain vector
a = sym([]);
a = [a; sym("a" + num2str(k-1))];

while all(LgbN == 0)
    bN = LfbN + LgbN*u + a(k)*bN;
    LfbN = Lf(bN);
    LgbN = Lg(bN);
    k = k+1;

    a = [a; sym("a" + num2str(k-1))];
end

% If we are not input constrained, then we are done here.
if opts.InputConstrained == false
    A = LgbN;
    b = LfbN + a(k)*bN;

    return;
end

% Take the infimum of bN = LfbN + LgbN*u + a(k)*bN (ICCBF). 
% Note: inf_{|u| <= u_max }(bN) = -|LgbN|*u_max. This covers the cases of
% a >= 0 and a <= 0.
bN = LfbN - abs(LgbN)*opts.InputLimits + a(k)*bN;

% Apply invariance condition.
k = k+1;
a = [a; sym("a" + num2str(k-1))];

A = Lg(bN);
b = Lf(bN) + a(k)*bN;

end           