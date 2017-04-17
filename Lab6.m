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

% Posicion original del cilindro * por la inclinacion del cilindro *
% centrado en l origen.
tuboPos = (transl(750, -225, 850)*trotx(-20, 'deg')*transl(200, 0, 200)*eye(4));
trplot(tuboPos);

%Generar las posiciones de soldadura (nos piden 20)
eje = zeros(4,4,20);
for i = 1:27:540
    eje(:,:,i) = tuboPos*troty(i-260,'deg')*transl(100,440-0.82*i,0)*...
         trotx(-90,'deg')*troty(-45,'deg');
     
    trplot(eje(:,:,i), 'length', 100);
end

while true
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Proceso de soldadura por puntos.
posRobot = transl(1000,-1000,1250);

standBy = simulador([0 0 0 0 0 0]);
primera = tuboPos*troty(1-260,'deg')*transl(190,-220,0)*transl(100,440-0.82,0)*...
         trotx(-90,'deg')*troty(-45,'deg'); 

interpolacionTransformaciones(posRobot, standBy, primera, 100, 1, 1, 1);


for i=1:27:540
     posePrev = tuboPos*troty(i-260,'deg')*transl(190,-220,0)*transl(100,440-0.82*i,0)*...
         trotx(-90,'deg')*troty(-45,'deg'); 
    
     poseAct = tuboPos*troty((i+27)-260,'deg')*transl(190,-220,0)*transl(100,440-0.82*(i+27),0)*...
         trotx(-90,'deg')*troty(-45,'deg');
     
     interpolacionTransformaciones(posRobot, posePrev, poseAct, 14, 1, 1, 1);
end

ultima = tuboPos*troty((540+27)-260,'deg')*transl(190,-220,0)*transl(100,440-0.82*(540+27),0)*...
         trotx(-90,'deg')*troty(-45,'deg');

cojerTubo = (transl(750, -225, 850)*trotx(-20, 'deg')*transl(200, 0, 200)*trotx(-pi/2)*eye(4)); 
cojerTubo = transl(0,-125,0)*cojerTubo;

interpolacionTransformaciones(posRobot, ultima, cojerTubo, 40, 1, 1, 1);

cojerTuboFin = transl(0,-100,0)*cojerTubo;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Torig = inv(transl(1000,-1000,1250))*cojerTubo;
Tdest = inv(transl(1000,-1000,1250))*cojerTuboFin;
pasos = ctraj(Torig, Tdest, 40);

for i = 1:40
    transfo = simulador(PumaIK(pasos(:,:,i),1,1,1)) * transl(-100,100,175)* trotx(pi/2);
    transfo2 = simulador(PumaIK(pasos(:,:,i),1,1,1)) * transl(-200,200,175)* trotx(pi/2);
    auxi = (transfo * tuboAux)';
    auxi2 = (transfo2 * chapaAux)';
    set(L1, 'Vertices', auxi(:,1:3) );
    set(L2, 'Vertices', auxi2(:,1:3) );
    drawnow;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cambiar los angulos del robot de -1, -1, -1 a 1, 1, -1.
% Tdest = PumaIK(inv(transl(1000,-1000,1250))*cojerTuboFin, 1, 1, 1);
% Torig = PumaIK(inv(transl(1000,-1000,1250))*cojerTuboFin, 1, 1, 1);
% pasos = jtraj(Torig, Tdest, 100);
% for i = 1:100
%     simulador(pasos(i,:));
%     drawnow;
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ponemos el tubo en el suelo.
Tdest = [0,-12,12,0,0,0];
Torig = PumaIK(inv(transl(1000,-1000,1250))*cojerTuboFin, 1, 1, 1);
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Colocamos el siguiente tubo a soldar.
transfo = transl(750, -225, 850)*trotx(-20, 'deg');
transfo2 = transl(750, -225, 850)*trotx(-20, 'deg')*transl(100, 0, 100);
auxi = (transfo2 * tuboAux)';
auxi2 = (transfo * chapaAux)';
set(L1, 'Vertices', auxi(:,1:3) );
set(L2, 'Vertices', auxi2(:,1:3) );
end
