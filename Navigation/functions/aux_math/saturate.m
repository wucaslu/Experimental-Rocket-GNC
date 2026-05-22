function y = saturate(x, xmin, xmax)

    y = x;
    
    if y < xmin
        y = xmin;
    end
    
    if y > xmax
        y = xmax;
    end

end