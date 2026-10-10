function y = rocketPlantInterpolate(grid,values,u)
%ROCKETPLANTINTERPOLATE Clamped linear interpolation of numeric column data.
% grid must be strictly increasing and values the same length.
n = numel(grid);
if n==1 || u<=grid(1)
    y = values(1);
elseif u>=grid(n)
    y = values(n);
else
    k = 1;
    for i=1:n-1
        if u>=grid(i) && u<grid(i+1)
            k=i;
            break
        end
    end
    fraction=(u-grid(k))/(grid(k+1)-grid(k));
    y=values(k)+(values(k+1)-values(k))*fraction;
end
end
