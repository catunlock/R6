function interpolacionAngulos(PosRobot, Torig, Tdest, pasos, a, b, c)
    
    Torig = inv(PosRobot)*Torig;
    Tdest = inv(PosRobot)*Tdest;

    Aorig = PumaIK(Torig, a,b,c);
    Adest = PumaIK(Tdest, a,b,c);

    datos = jtraj(Aorig, Adest, pasos);

    for i = 1:pasos
        simulador(datos(i,:));
        drawnow;
    end

end