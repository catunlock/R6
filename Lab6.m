close all;
clear all;


%Loads the spiral model and translates and rotates it to its place
chapa = stlread('ChapaEspiral.stl');
chapaAux = [chapa.vertices(:,1:3)'; ones(size(chapa.vertices(:,1)))'];
aux2 = (transl(750, -225, 850)*trotx(-20, 'deg')*chapaAux)';
chapa.vertices = aux2(:,1:3);

%Loads the tube model and translates and rotates it to its place
tubo = stlread('Tubo.stl');
tuboAux = [tubo.vertices(:,1)'; tubo.vertices(:,2)'; tubo.vertices(:,3)'; ones(size(tubo.vertices(:,1)))'];
aux2 = (transl(750, -225, 850)*trotx(-20, 'deg')*transl(100, 0, 100)*tuboAux)';
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

handler = figure(1);

grid on;
axis([-4000 4000 -4000 4000 0 3000]);
daspect([1 1 1]);
view(3); 
camlight;
lighting gouraud;
hold on;

L2 = patch(chapa,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);

L1 = patch(tubo,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
     
%material('dull');

patch('vertices',v1,'faces',f1,'facecolor','r'); 
patch('vertices',v2,'faces',f2,'facecolor','g');

%fill3(H(1,:),H(2,:),H(3,:),'b');  

pos = [0,500,0];

pause(1);



tuboPos = (transl(750, -225, 850)*trotx(-20, 'deg')*transl(200, 0, 200)*eye(4));
trplot(tuboPos);
% camtarget([750 -225 850]);

ultimaPos = eye(4);
for i=1:3:540
     pose = tuboPos*troty(i-260,'deg')*transl(190,-220,0)*transl(100,440-0.82*i,0)*...
         trotx(-90,'deg')*troty(-45,'deg');
     
     eje = tuboPos*troty(i-260,'deg')*transl(100,440-0.82*i,0)*...
         trotx(-90,'deg')*troty(-45,'deg');
     trplot(eje);
     pose = inv(transl(1000,-1000,1250))*pose;
    %trplot(pose);
     angulos = PumaIK(pose, -1, -1, -1);
     ultimaPos = simulador(angulos);
     drawnow;
end

pause(1);

   

Tdest = [0,-12,12,0,0,0];
tuboCojer = (transl(750, -225, 850)*trotx(-20, 'deg')*transl(200, 0, 200)*trotx(-pi/2)*eye(4));
trplot(tuboCojer);
Torig = PumaIK(inv(transl(1000,-1000,1250))*transl(0,-125,0)*tuboCojer, 1, 1, -1);
pasos = jtraj(Torig, Tdest, 100);

for i = 1:100
    transfo = simulador(pasos(i,:))*transl(-100,100,175)* trotx(pi/2);
    transfo2 = simulador(pasos(i,:))*transl(-200,200,175)* trotx(pi/2);
    auxi = (transfo * tuboAux)';
    auxi2 = (transfo2 * chapaAux)';
    set(L1, 'Vertices', auxi(:,1:3) );
    set(L2, 'Vertices', auxi2(:,1:3) );
    drawnow;
end
    

% trplot(tuboPos);
% pasos = ctraj(inv(transl(1000,-1000,1250))*tuboPos, inv(transl(1000,-1000,1250))*transl(1750,-500,1000), 100);
% for i = 1:100
%     angulos = PumaIK(pasos(:,:,i), -1, -1, 1);
%     trplot(transl(1000,-1000,1250)*pasos(:,:,i));
%     % simulador(angulos);
%     drawnow;
% end


% pasos = ctraj(inv(transl(1000,-1000,1250))*transl(1750,-500,1000),inv(transl(1000,-1000,1250))*transl(1750,-500,0), 100);
% for i = 1:100
%     angulos = PumaIK(pasos(:,:,i), -1, -1, 1);
%    % trplot(transl(1000,-1000,1250)*pasos(:,:,i));
%      simulador(angulos);
%     drawnow;
% end
    