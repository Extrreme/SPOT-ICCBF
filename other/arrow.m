function h = drawArrow(x,y,varargin)
	% drawArrow Draw an arrow as a single patch object
	%
	% h = drawArrow([x1 x2],[y1 y2],'PropertyName',PropertyValue,...)
	%
	% Custom properties:
	%   'HeadLength' (fraction of arrow length, default 0.1)
	%   'HeadWidth'  (fraction of arrow length, default 0.06)
	%   'ShaftWidth' (fraction of arrow length, default 0.02)
	%
	% All remaining name-value pairs are passed directly to PATCH.

	% Validate inputs
	if numel(x) ~= 2 || numel(y) ~= 2
		error('x and y must be 1x2 vectors: [x1 x2], [y1 y2]')
	end

	x1 = x(1);	x2 = x(2);
	y1 = y(1);	y2 = y(2);

	% Defaults
	headLength = 0.1;
	headWidth  = 0.06;
	shaftWidth = 0.02;

	% Parse custom parameters
	names = {'HeadLength','HeadWidth','ShaftWidth'};
	for k = 1:numel(names)
		idx = strcmpi(varargin,names{k});
		if any(idx)
			eval([lower(names{k}) ' = varargin{find(idx)+1};'])
			varargin([find(idx),find(idx)+1]) = [];
		end
	end

	% Direction
	dx = x2 - x1;
	dy = y2 - y1;
	L = hypot(dx,dy);
	if L == 0
		h = gobjects(0);
		return
	end

	ux = dx / L;
	uy = dy / L;
	px = -uy;
	py =  ux;

	% Key point (base of head)
	xb = x2 - headLength*L*ux;
	yb = y2 - headLength*L*uy;

	sw = shaftWidth*L/2;
	hw = headWidth*L/2;

	% Polygon vertices (clockwise)
	X = [ ...
		x1 + sw*px, ...
		xb + sw*px, ...
		xb + hw*px, ...
		x2, ...
		xb - hw*px, ...
		xb - sw*px, ...
		x1 - sw*px ];

	Y = [ ...
		y1 + sw*py, ...
		yb + sw*py, ...
		yb + hw*py, ...
		y2, ...
		yb - hw*py, ...
		y1 - sw*py ];

	h = patch(X,Y,'k',varargin{:});
end
