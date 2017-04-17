function interpolacionTransformaciones(PosRobot, Torig, Tdest, pasos, a, b, c)
    
    Torig = inv(PosRobot)*Torig;
    Tdest = inv(PosRobot)*Tdest;

    datos = ctraj(Torig, Tdest, pasos);

    for i = 1:pasos
        simulador(PumaIK(datos(:,:,i),a,b,c));
        drawnow;
    end

end