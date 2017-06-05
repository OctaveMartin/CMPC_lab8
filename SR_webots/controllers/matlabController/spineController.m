function qs=spineController(theta, r)
%%
%     input:
%         theta - vector of cpg phases (20 x 1)
%         r - vector of cpg amplitudes (20 x 1)

%     output:
%         qs - spine joint angles (10 x 1)

qs = r(1:10).*(1+cos(theta(1:10)))-r(11:20).*(1+cos(theta(11:20)));
end

