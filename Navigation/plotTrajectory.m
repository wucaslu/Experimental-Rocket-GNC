close all
plot3(out.East.signals.values, out.North.signals.values, out.Up.signals.values); 
grid on; 
xlabel('East(t)'); 
ylabel('North(t)'); 
zlabel('Up(t)'); 
hold on;  
plot3(out.EastTrue.signals.values, out.NorthTrue.signals.values, out.UpTrue.signals.values);