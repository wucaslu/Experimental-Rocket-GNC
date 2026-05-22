function S = skew3(x)

    S = [  0.0,  -x(3),   x(2);
           x(3),   0.0,  -x(1);
          -x(2),  x(1),   0.0 ];

end