


load gridSearch1.mat

surf(GS.var1_grid, GS.var2_grid, reshape(GS.distance, size(GS.var1_grid)));
xlabel('total phase lag [rad]')
ylabel('frequency [Hz]')
zlabel('distance [m]')