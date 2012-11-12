points = [ 6, 3.2;
    9, 2.35;
    15, 1.65;
    20, 1.3;
    25, 1.1;
    30, 0.9;
    40, 0.75;
    50, 0.6;
    60, 0.5;
    70, 0.45;
    80, 0.4
    80, 0];

linArray = [];

for i=2:size(points)
    v1ADC = points(i-1,2)/5 * 1024; 
    v2ADC = points(i,2)/5 * 1024;
    linArray = [linArray linspace(points(i-1,1),points(i,1),v1ADC-v2ADC)];
end

linArray = linArray(end:-1:1);
csvwrite('linarray.dat',round(  linArray .* 10))