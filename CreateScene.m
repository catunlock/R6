
%Loads the spiral model and translates and rotates it to its place
chapa = stlread('ChapaEspiral.stl');
aux = [chapa.vertices(:,1:3)'; ones(size(chapa.vertices(:,1)))'];
aux2 = (transl(750, -225, 850)*trotx(-20, 'deg')*aux)';
chapa.vertices = aux2(:,1:3);

%Loads the tube model and translates and rotates it to its place
tubo = stlread('Tubo.stl');
aux = [tubo.vertices(:,1)'; tubo.vertices(:,2)'; tubo.vertices(:,3)'; ones(size(tubo.vertices(:,1)))'];
aux2 = (transl(750, -225, 850)*trotx(-20, 'deg')*transl(100, 0, 100)*aux)';
tubo.vertices = aux2(:,1:3);

%%%% Definition of the Panel vertices and faces  
v1 = [0 0 0; 3000 0 0; 3000 0 200; 0 0 200; 3000 500 1700; ...
      0 500 1700; 3000 700 1700; 0 700 1700; 3000 700 0; ...
      0 700 0; 0 700 200; 3000 700 200];

f1 = [1 2 3 4; 4 3 5 6; 6 5 7 8; 8 7 9 10; 1 10 9 2; ...
      2 9 12 3; 3 12 7 5; 1 4 11 10; 4 6 8 11];

%%%% Definition of the Base vertices and faces 
v2 = [240 -1500 0; 2740 -1500 0; 2740 -1500 200; ...
    240 -1500 200; 2740 -500 200; 240 -500 200; ...
    2740 -500 0; 240 -500 0];
f2 = [1 2 3 4; 4 3 5 6; 6 5 7 8; 8 7 2 1; 2 7 5 3; 1 4 6 8];

%%%% Definition of a circle 
radius = 350;      
theta = linspace(0,2*pi);          
X = radius.*cos(theta);  
Y = radius.*sin(theta);  
C = trotx(70, 'deg')*transl(950, 950, 100)*[X; Y; zeros(size(X)); ones(size(X))];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Plot the models

figure(2);

grid on;
axis([-1000 4000 -2000 1000 0 3000]);
daspect([1 1 1]);
view(3); 
camlight;
lighting gouraud;
hold on;

patch(chapa,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);

patch(tubo,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
     
%material('dull');

patch('vertices',v1,'faces',f1,'facecolor','r'); 
patch('vertices',v2,'faces',f2,'facecolor','g');

fill3(H(1,:),H(2,:),H(3,:),'b');  


