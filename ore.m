function [N, Xe] = ore(equarec, N, x0, varargin)
x0 = x0(:); % ensures a column vector
dim = length(x0); % state vector dimension
Duree = N(2) - N(1) + 1; % time length
N = N(1):N(2);
Xe = zeros(Duree, dim);
xe = x0;
indmatlab = 1;
for n = N
    Xe(indmatlab,:) = xe';
    xe = equarec(n, xe, varargin{:});
    indmatlab = indmatlab + 1;
end
end