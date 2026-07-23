function p = patchline(varargin)
% Plot lines as patches (efficiently)

args = varargin;

if isgraphics(args{1}, 'axes')
    ax = args{1};
    args = args(2:end);
else
    ax = gca;
end

if numel(args) < 3
    error('Minimum arguments is 3');
end

X = args{1};
Y = args{2};

if isnumeric(args{3}) && numel(args) < 4
    error('Must include C');
elseif isnumeric(args{3})
    Z = args{3};
    C = args{4};
    args = args(5:end);
else
    Z = [];
    C = args{3};
    args = args(4:end);
end

if numel(args) > 0 && rem(numel(args),2) ~= 0
    error('Must have even number of options');
elseif numel(args) > 0
    opts = args;
else
    opts = {};
end

if isempty(Z)
    p = patch(ax, [X(:);NaN],[Y(:);NaN],C);
else
    p = patch(ax, [X(:);NaN],[Y(:);NaN],[Z(:);NaN],C);
end

for i = 1:2:numel(opts)
    set(p,opts{i},opts{i+1})
end

if nargout == 0
    clear p
end