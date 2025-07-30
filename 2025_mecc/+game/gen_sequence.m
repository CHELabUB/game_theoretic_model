function y = gen_sequence(n, N)
%{
Adapted from comvec
%}
if N == 0
    y = [];
else
    y = 1:n;
    for i = 2:N
        z = 1:n;
        y = [copy_blocked(y,size(z,2)); copy_interleaved(z,size(y,2))];
    end
end
end

%=========================================================
function b = copy_blocked(m,n)
[mr,mc] = size(m);
b = zeros(mr,mc*n);
ind = 1:mc;
for i=[0:(n-1)]*mc
    b(:,ind+i) = m;
end
end

function b = copy_interleaved(m,n)
[mr,mc] = size(m);
b = zeros(mr*n,mc);
ind = 1:mr;
for i=[0:(n-1)]*mr
  b(ind+i,:) = m;
end
b = reshape(b,mr,n*mc);
end
