function sysCall_init() 
    usensors={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}
    for i=1,16,1 do
        usensors[i]=sim.getObjectHandle("Pioneer_p3dx_ultrasonicSensor"..i)
    end
    motorLeft=sim.getObjectHandle("Pioneer_p3dx_leftMotor")
    motorRight=sim.getObjectHandle("Pioneer_p3dx_rightMotor")
    noDetectionDist=0.3 
    maxDetectionDist=0.1
    detect={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
    braitenbergL={-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
    braitenbergR={-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
    v0=2
    
--Basic variables, entry data for sensors-- 

 
end

function sysCall_actuation() 
--continuous loop that reads the data from ultrasonic sensors--
--and then proceeds to random wandering and wall following--
    for i=1,16,1 do
        res,dist=sim.readProximitySensor(usensors[i])
        if (res>0) and (dist<noDetectionDist) then
            if (dist<maxDetectionDist) then
                dist=maxDetectionDist
            end
            detect[i]=1-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist))
        else
            detect[i]=0
        end
    end
    
    vLeft=v0
    vRight=v0
    --random wandering starts here--
    for i=1,16,1 do
    --if there are no detections on sensors, proceed with random wandering--
        if(detect[i]==0)then
        vLeft=math.random(0,5)--range from 0 to 5 proved to be optimal by many tests--
        vRight=math.random(0,5)
        printf("No walls detected, wandering")
        elseif(detect[i]>0)then --if a wall is detected, break the loop and proceed with the next task--
        printf("Wall detected, following")
        break
        end
 --random wandering ends here--
 --**************************--
    end    
     --**************************--
    --wall following starts here--
        for i=1,16,1 do
        if(detect[4]>=0.01 and detect[8]>=0.01)then
        vLeft=vLeft+braitenbergL[i]*detect[i]
        vRight=vRight+braitenbergR[i]*detect[i]
        elseif(detect[4]>=0.01 and detect[9]>=0.01)then
        vLeft=vLeft+braitenbergL[i]*detect[i]
        vRight=vRight+braitenbergR[i]*detect[i]
        elseif(detect[4]>=0.01 and detect[10]>=0.01)then
        vLeft=vLeft+braitenbergL[i]*detect[i]
        vRight=vRight+braitenbergR[i]*detect[i]
        elseif(detect[4]>=0.01 and detect[11]>=0.01)then
        vLeft=vLeft+braitenbergL[i]*detect[i]
        vRight=vRight+braitenbergR[i]*detect[i]
        elseif(detect[4]>=0.01 and detect[1]>=0.01) then
        vLeft=vLeft+braitenbergL[i]*detect[i]
        vRight=vRight+braitenbergR[i]*detect[i]
        elseif(detect[4]>=0.01 and detect[2]>=0.01) then
        vLeft=vLeft+braitenbergL[i]*detect[i]
        vRight=vRight+braitenbergR[i]*detect[i]
        elseif(detect[4]>=0.01 and detect[3]>=0.01) then
        vLeft=vLeft+braitenbergL[i]*detect[i]
        vRight=vRight+braitenbergR[i]*detect[i]
        elseif(detect[4]>=0.01 and detect[16]>=0.01)then
        vLeft=vLeft+braitenbergL[i]*detect[i]
        vRight=vRight+braitenbergR[i]*detect[i]
        elseif(detect[2]-detect[14]>=0.01) then
        vLeft=vLeft+braitenbergL[i]*detect[i]
        vRight=vRight+braitenbergR[i]*detect[i]
        elseif(detect[7]-detect[11]>0.01) then
        vLeft=vLeft+braitenbergL[i]*detect[i]
        vRight=vRight+braitenbergR[i]*detect[i]
        elseif(detect[8]<=0.01 and detect[4]~=0)then
        vLeft=vLeft+braitenbergL[i]*detect[i]
        vRight=vRight+braitenbergR[i]*detect[i]
        elseif(detect[1]<=0.01 and detect[4]~=0)then
        vLeft=vLeft+braitenbergL[i]*detect[i]
        vRight=vRight+braitenbergR[i]*detect[i]
        elseif(detect[7]<detect[10])then
        vLeft=vLeft+braitenbergL[i]*detect[i]+0.3
        vRight=vRight+braitenbergR[i]*detect[i]
        elseif(detect[2]<detect[15])then
        vLeft=vLeft+braitenbergL[i]*detect[i]
        vRight=vRight+braitenbergR[i]*detect[i]+0.3
        elseif(detect[4]<=0.1 and detect[8]>=0.2)then
        repeat
        vLeft=vLeft+braitenbergL[i]*detect[i]+0.5
        vRight=vRight+braitenbergR[i]*detect[i]+0.1
        until(detect[4]<=0.1 and detect[8]>=0.2)
        elseif(detect[4]<=0.1 and detect[1]>=0.2)then
        repeat
        vLeft=vLeft+braitenbergL[i]*detect[i]+0.1
        vRight=vRight+braitenbergR[i]*detect[i]+0.5
        until(detect[4]<=0.1 and detect[1]>=0.2)
        end
        end
        
--above if conditions, added values were chosen through many experiments--
--and measurements when a sensor detects a wall and what are the values it can display--
    sim.setJointTargetVelocity(motorLeft,vLeft)
    sim.setJointTargetVelocity(motorRight,vRight)
    
end 
