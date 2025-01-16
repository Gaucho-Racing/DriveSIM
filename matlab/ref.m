function [tfinal,topspeed,eused,eregen,trackarray,topspeedsect] = TimeForTrack_EMRAX_AMK_REV2(grrear,grhubs,trackdistarray,graphspeedvsdistance)

  % Parameters:
      % Arrays
      speedvsdistance = zeros(1,8); % used for plots
      lapvaluesarray = zeros(1,6); % used to mark turns in plots (distance, speed, time, rear power output, front power output, edefused_aero, eused)
      eregenarray = [0,0]; % used for plotting energy values (time, energy regened, )
      torquearray = [0,0,0];

      % Time
      dt = 0.01;           % seconds
     
      % Aero
      Cd = 1.09;       % drag coefficient
      Cl = -2.45;     % coefficient of lift
      
      rho = 1.293;    % density of air kg/m^3
      Acs = 0.6294092;      % crosssectional area of car m^2 from front
      Acl = 1.913493;  %cross sectional area of car m^2 from top
      dH = 0.3;       % height from the ground of resultant drag force, m
  
      % Mass and Distribution
      CGH = 0.2032;   % center of gravity height m
      m = 280.155;        % mass of car kg
      g = 9.81;       % acceleration due to gravity m/s^2
      W = m*(g);      % weight of car N
      L = 1.525;      % Wheelbase, meters
      Bf = 0.450;      % Front Fmass bias
      Rf = 0.550;      % Rear mass bias
      Lf = Bf*L;      % Front axle to CG horizontal distance, meters
      Lr = L - Lf;    % Rear axle to CG horizontal distance, meters
      theta = 0;      % road incline, radians
      TW = 1.194; % track width, m

      % Motor
      eff = 0.96*0.92;  % drivetrain efficiency
      grrearinv = 1/grrear; % driven/driving
      grhubsinv = 1/grhubs; % driven/driving
  
      % Tires
      us = 1.67; % coefficient of static friction for tires
      uk = us*0.25; % coefficient of kinetic friction for tires
      rin = 8;       % radius of wheels in
      r = rin*2.54/100;     % radius of tire m
      Crr = 0.014;    % rolling resistance coefficient for each wheel
      wheelrpm = 0;
      
      % Electrical 
      MaxPowerKW = 80; % KW
      MaxPowerW = MaxPowerKW*1000; % W
      regen_eff = 0.67; % regen efficiency

      %Rear Motor
      function [torque] = getMaxTorquerear(motorRPM) %returns theoretical max torque in nm
              torque = min([220, 809549.3/motorRPM]); %MAXXXXXXXXXXXXXXXX % Power limit of 80kW with torque cap at 220nm (maybe can push harder)
             
      end
      
      function [maxRPM] = getMaxRPMrear(currentTorque, energyUsed) %torque in nm, energy used in kwh
          kv = -0.0204090909091*currentTorque + 10.14;
          voltage = -0.0176037735849 * energyUsed + 532; %this is such a rough calc its not even funny
          maxRPM = kv * voltage;
          maxRPM = 5300;
      end

      %Hub Motor
      

      function [torque] = getMaxTorquehubs(motorRPM) %returns theoretical max torque in nm
            if motorRPM <= 13412.5
                  torque = 21.25*2;
            elseif motorRPM > 13412.5 && motorRPM <= 18500
                  torque = 1*(-0.0008*(motorRPM) + 53.196);
            else
                  torque = 1*(-0.0055*(motorRPM) + 130.52);
            end
            %torque = 0; %get rid of hub motors
      end
      
      function [maxRPM] = getMaxRPMhubs(currentTorque, energyUsed) %torque in nm, energy used in kwh
         
          maxRPM =18500;
         
      end

  MaxPowerOutputEmrax = 0; % Max power output by emrax
  PowerOutputEmrax = 0; %  Power output by emrax
  MaxTorqueEmrax = 0; % Max torque output by emrax
  MaxPowerOutputhub = 0; % Max power output by hub motor
  PowerOutputhub = 0; % Power output by hub motor
  MaxTorquehub = 0; % Max torque output by hub motor
  



  % Kinetics:
  FDrag               = @(velocity) 0.5 * rho * velocity^2 * Cd * Acs; % Drag Force
  FSlope              = @(weight, theta) weight * sin(theta);         % Force of gravity against car
  FLift               = @(velocity) 0.5 * Cl * rho * Acl * velocity^2;% Force of lift
  FNormalf            = @(velocity,acceleration) (Lr/L)*(W*cos(theta)-FLift(velocity)) - (CGH/L)*(W*sin(theta)+1*m*acceleration)-(CGH/L)*FDrag(velocity); % normal force on front tires N (longitudinal load transfer)
  FNormalr            = @(velocity,acceleration) (Lf/L)*(W*cos(theta)-FLift(velocity)) + (CGH/L)*(W*sin(theta)+1*m*acceleration)+(CGH/L)*FDrag(velocity); % normal force on rear tires N (longitudinal load transfer)
  FNormalin           = @(velocity, radius, weight) weight/2 - weight*((velocity^2)/(g*radius))*CGH/TW; % normal force on inner wheel N (lateral load transfer)
  FNormalout          = @(velocity, radius, weight) weight/2 + weight*((velocity^2)/(g*radius))*CGH/TW; % normal force on outer wheel N (lateral load transfer)
  Fresf               = @(velocity,acceleration) FNormalf(velocity,acceleration)*2*Crr;               % Force of Rolling Resistance on Front wheel N
  Fresr               = @(velocity,acceleration) FNormalr(velocity,acceleration)*2*Crr;               % Force of Rolling Resistance on Rear wheel N
  TractiveLimitrear       = @(velocity,acceleration) us*FNormalr(velocity,acceleration);                 % TractiveLimit  of rear in straight N
  TractiveLimithubs       = @(velocity,acceleration) us*FNormalf(velocity,acceleration);                 % TractiveLimit  of front in straight N
  TractiveLimitturnrear   = @(velocity,acceleration, radius) us*2* FNormalin(velocity, radius,FNormalr(velocity,acceleration));                 % TractiveLimit of rear in turn N
  TractiveLimitturnfrontin   = @(velocity,acceleration, radius) us* FNormalin(velocity, radius,FNormalf(velocity,acceleration));                 % TractiveLimit of front inner wheel in turn N
  TractiveLimitturnfrontout   = @(velocity,acceleration, radius) us* FNormalout(velocity, radius,FNormalf(velocity,acceleration));                 % TractiveLimit of front outer wheel in turn N
  Fxrrear                 = @(motorRPM) getMaxTorquerear(motorRPM)*grrear*eff/r;                                                                                  % calculate tractive force from motor on rear wheel
  Fxrhubs                 = @(motorRPM) getMaxTorquehubs(motorRPM)*grhubs*eff/r;                                                                                  % calculate tractive force from motor on rear wheel

  % Braking
  maxbrakeforcerear = @(velocity,acceleration) us*(FNormalr(velocity,acceleration)+(Lf/L)*(FLift(velocity))); % Max braking decceleration
  maxbrakeforcefront = @(velocity,acceleration) us*(FNormalf(velocity,acceleration)+(Lr/L)*(FLift(velocity)));
  maxbrakeforceinner = @(velocity,acceleration,radius) us*FNormalin(velocity,radius,FNormalf(velocity,acceleration)+(Lr/L)*(FLift(velocity)));
  maxbrakeforceouter = @(velocity,acceleration,radius) us*FNormalout(velocity,radius,FNormalf(velocity,acceleration)+(Lr/L)*(FLift(velocity)));

  % Kinematics
  sectvmaxbymotorrear = @(currentTorque,energyUsed) getMaxRPMrear(currentTorque, energyUsed)*((2*pi*grrearinv*r)/(60));  % max speed in section due to rear motor  
  sectvmaxbymotorhubs = @(currentTorque,energyUsed) getMaxRPMhubs(currentTorque, energyUsed)*((2*pi*grhubsinv*r)/(60));  % max speed in section due to hub motor
  sectvin = @(velocity, radius) velocity - TW*velocity/(2*radius); % speed of inner wheel
  sectvout = @(velocity, radius) velocity + TW*velocity/(2*radius); % speed of outer wheel

  % Initial values
  topspeed = 0; % m/s
  topspeedsect = 0;
  v = 0; % m/s
  a = 0; % m/s
  d = 0; % m
  eused = 0; % KWh
  eregen = 0; %KWhsecta
  time = 0;  % s 
  Fxrreart =0; %N
  Fxrhubst = 0; %N
  edefused_aero = 0; %KWh



trackarray = zeros(height(trackdistarray),3); % array to store section times and final speeds

% whileloop to iterate through each section of track
i = 1;
while i <= height(trackdistarray)
  trackarray(i,1) = i; % label section number in track array

  % reset the section values after each section:
  sectt = 0; % time spent in section
  sectd = 0; % distance travled in section
  sectv = v; % speed in section
  secta = a; % acceleration in section
  sectvmaxbyturn = inf; % speed limit due to turn
  sectvmaxturnnext = inf;
  eused = speedvsdistance(end,8);
  


  stopwhileloop = false; % variable to determine if vehicle can still accelerate in section
  willeventualturn = false; % variable to determine if the next section is a turn
  lastsect = false; % variable to determine if it is last section
  
  % initialize certain values in section
  
  secttfinal = 0;
  sectvfinal = 0;
  sectafinal = 0;
  secttfinalprediction = 0;
  sectvfinalprediction = 0;
  sectafinalprediction = 0;
  sectbrakingregenprediction = 0;
  storedbrakingspeedvsdistance = zeros(1,8);
  stored_eused_deccel_prediction = 0;
  hitmaxspeed = false;
  MAXSPEEDINSECTION = 0;
     

  if trackdistarray(i,2) > 0 % determine if current section is turn or not
      % if in turn, find max speed in turn:
      turnr = trackdistarray(i,2); % turn radius of current section
      acmax = g*us; % max centripetal acceleration in turn
      sectvmaxbyturn = sqrt(acmax*turnr); % max speed in section due to turn
  end % end of if statement to determine if current section is in turn or not

  if i < height(trackdistarray) % if current section is not last section of track, do the following
      if  trackdistarray(i+1,2) > 0 % determine if next section is turn or not
          % if next section is turn, find max speed in next section
          turnrnext = trackdistarray(i+1,2); % turn radius of next section
          acmaxnext = g*us; % max centripetal acceleration in next section turn
          sectvmaxturnnext = sqrt(acmaxnext*turnrnext); % max speed in next section turn
          willeventualturn = true; % remmeber that the next section is a turn, this means the final speed in the section must be less than or equal to the max speed of the next section
      elseif trackdistarray(i+1,2) < 0 % determine if current section is at end
          lastsect = true;
      else  
          % there is a straight in the next section, so max speed is
          % sectvmaxturnnext = inf (see above)
      end % end of if statement to determine if next section is in turn or not
  end % end of if statement for conditions if current section is last or not
       


  while sectd < trackdistarray(i) && stopwhileloop == false % while loop to calculate distance, speed, and time while accelerating in a section    
      
      % determine motor rpms:
      motorrpmrear = (sectv/r)*grrear*60/(2*pi); % calculate current motor rpm as a function of vehicle speed
      motorrpmhubs = (sectv/r)*grhubs*60/(2*pi); % calculate current motor rpm as a function of vehicle speed
      turnradius = trackdistarray(i,2); % determine turn radius
     
      % determine tractive forces:
      Fxrreart = min([Fxrrear(motorrpmrear) TractiveLimitrear(sectv,secta)]); % limit tractive force based on tractive limit due to load transfer (longitudinal transfer on rear wheels)
      Fxrhubstin  =  min([Fxrhubs(motorrpmhubs) TractiveLimithubs(sectv,secta)])/2; % limit tractive force based on tractive limit due to load transfer (longitudinal transfer on front inner wheel)
      Fxrhubstout  =  min([Fxrhubs(motorrpmhubs) TractiveLimithubs(sectv,secta)])/2; % limit tractive force based on tractive limit due to load transfer (longitudunal transfer on front outer wheel)
      

      if turnradius > 0 %if section is a turn modify the tractive limit stuff to include lateral load transfer
          Fxrreart = min([Fxrrear(motorrpmrear) TractiveLimitturnrear(sectv,secta,turnradius)]); % limit tractive force based on tractive limit due to load transfer (long and lat transfer on rear wheels)   
          Fxrhubstin = min([Fxrhubs(motorrpmhubs)/2 TractiveLimitturnfrontin(sectvin(sectv,turnradius),secta,turnradius)]); % limit tractive force based on tractive limit due to load transfer (long and lat transfer on inner wheel)
          Fxrhubstout = min([Fxrhubs(motorrpmhubs)/2 TractiveLimitturnfrontout(sectvout(sectv,turnradius),secta,turnradius)]); % limit tractive force based on tractive limit due to load transfer (long and lat transfer on inner wheel)
      end

      Fxrhubst = Fxrhubstin+Fxrhubstout; % total tractive force by hub motors
      Fxrt_from_motors =  Fxrreart+Fxrhubst; % tractive force by both motors
  
       if (Fxrt_from_motors*sectv > MaxPowerW) %Limit Tractive Force based on RPM
          Fxrt_from_motors_High = Fxrt_from_motors;
          Fxrt_from_motors = MaxPowerW/sectv;
          Fxrreart = Fxrreart*Fxrt_from_motors/Fxrt_from_motors_High;
          Fxrhubst = Fxrhubst*Fxrt_from_motors/Fxrt_from_motors_High;           
       end

      Fxrt = Fxrreart+Fxrhubst-(FSlope(W,theta)-FDrag(sectv)-Fresf(sectv,secta)-Fresr(sectv,secta)); % sum total tractive force

      if hitmaxspeed % if hit max speed
          sectv_prediction = MAXSPEEDINSECTION; % set velocity equal to max speed
          secta = 0; % set acceleartion to 0      
          Fxrt_from_motors = FSlope(W,theta)+FDrag(sectv)+Fresf(sectv,secta)+Fresr(sectv,secta); % set force from wheels equal to wind and wheel resistances, required to keep constant speed
          %Determine which wheel has most load transfer and determine
          %force contributed by them (used later to determind power)
          if Rf >= Bf % Rear has most load transfer
              TMrear = (Fxrreart*r/grrear); % split force with rear wheel contributing most
              TMhubs = ((Fxrt_from_motors-Fxrreart)*r/grhubs); % Assign remaining required force to hubs
          else % Front has most load transfer
              TMhubs = (Fxrhubst*r/grhubs);  % split force with hub wheels contributing most
              TMrear = ((Fxrt_from_motors-Fxrhubst)*r/grrear);% Assign remaining required force to rear motor
          end    
          TMrear = Fxrt_from_motors*0.5/grrear;
          TMhubs = Fxrt_from_motors*0.5/grhubs;
      else

          % determine torque
          TMrear = (Fxrreart*r/grrear);
          TMhubs = (Fxrhubst*r/grhubs);
          
          % determine acceleration
          secta = Fxrt/m; % calculate acceleration based on tractive force, air resistance, and rolling resistance

          % predict velocity
          sectv_prediction = sectv + dt*(secta);
      end
      % predict energy used
      eused_prediction = eused + (Fxrt_from_motors*dt*sectv)*2.77778*10^(-7); %KWH

      % predict energy defused by aero
      edefused_aero_prediction = edefused_aero + (FDrag(sectv)*dt*sectv)*2.77778*10^(-7); %KWh
      
      % limit velocity prediction due to motor and track constraints
      sectv_prediction = min([sectv_prediction sectvmaxbymotorrear(TMrear,eused_prediction) sectvmaxbymotorhubs(TMhubs,eused_prediction) sectvmaxbyturn]);
      
      % predict location 
      sectd_prediction = sectd + dt*sectv;
      sectacurrentdeccel= secta;

      % if statement to determine if braking at current instance of time will allow driver to clear incoming turn
      if willeventualturn && sectv > sectvmaxturnnext
          brakingspeedvsdistance = [sectd+d, sectv, sectacurrentdeccel, time+sectt,0, 0,edefused_aero_prediction,eused]; % temporary array to store speed and distance in braking section
          tempsecttaccel = sectt;
          tempsectdaccel = sectd;
          ddeccel = trackdistarray(i)-tempsectdaccel;

          % formula for decelleration of car
          sectadeccel = @(vel,acc)(-FSlope(W,theta)-FDrag(vel)-(4*Crr*m*g))/m-us*(g); % total acceleration in braking (decceleration m/s^2)
          sectadeccel_frombraking = @(vel,acc)(-1*us*g); % acceleration caused from just braking (decceleration m/s^2)

          sectbrakingregen = eregen; % preidcted energy regened in braking
          dtdeccel = 0.01; % time interval in braking
          secttdeccel = 0; % time in braking
          sectddeccel = 0; % distance in braking
          sectddeccelprediction = 0;
          sectvdeccelprediction = 0;
          eused_deccel_prediction = eused;
          sectvdeccel = sectv; % speed in braking
          sectacurrentdeccel = sectadeccel(sectvdeccel,sectacurrentdeccel); % calculate initial decceleeration from current speed and acceleration of car (m/s^2)
          sectacurrentdeccel_braking = sectadeccel_frombraking(sectvdeccel,sectacurrentdeccel); % calculate initial decceleeration (FORCES FROM THE BRAKES ONLY, SO DOES NOT INCLUDE ROLLING RESIST OR DRAG) from current speed and acceleration of car (m/s^2)

          % while loop to determine if can't clear incoming turn
          while sectddeccelprediction < ddeccel
               % start prediction array with current values:
               edefused_aero_prediction_braking = edefused_aero;
              
              % predict velocity
              sectvdeccelprediction = sectvdeccel + dtdeccel*(sectadeccel(sectvdeccel,sectacurrentdeccel));
                             
              % sectvdeccel can't be lower than sectvmaxturn next
              if sectvdeccelprediction< sectvmaxturnnext
                 sectvdeccelprediction = sectvmaxturnnext; 
              end
              
              % predict location 
              sectddeccelprediction = sectddeccel + dtdeccel*sectvdeccel ;    
              
   
              % if not exceed section distance
              if sectddeccelprediction < ddeccel
                  sectvdeccel = sectvdeccelprediction;
                  sectddeccel = sectddeccelprediction;
                  secttdeccel = secttdeccel + dtdeccel;
                  
                  % predict energy regen by braking
                  wheelrpm = sectvdeccel*60/(r*2*pi); % wheel speed (rpm)
                  wheelradpersec = sectvdeccel/r; % wheel speed (rad/s)
                  
                  
                  
                  brakeforcerear = maxbrakeforcerear(sectvdeccel,sectacurrentdeccel_braking); % calculate max brake force from rear wheels (N)
                  if turnradius > 0  % if in turn, will vary brake force due to lateral and long load transfer  
                      brakeforceinner = maxbrakeforceinner(sectvdeccel,sectacurrentdeccel_braking,turnradius); % calculate max brake force from front inner wheels (N)
                      brakeforceouter = maxbrakeforceouter(sectvdeccel,sectacurrentdeccel_braking,turnradius); % calculate max brake force from front outer wheels (N)
                  else % if not in turn, will base brake force in hubs due to only longitudinal load transfer
                      brakeforceinner = maxbrakeforcefront(sectvdeccel,sectacurrentdeccel_braking)/2; % calculate max brake force from front  wheels (N)
                      brakeforceouter = maxbrakeforcefront(sectvdeccel,sectacurrentdeccel_braking)/2; % calculate max brake force from front  wheels (N)
                  end

                  sectbrakingregen_rear = regen_eff*dtdeccel/3600*min([sectvdeccel*brakeforcerear/1000 wheelradpersec*grrear*getMaxTorquerear(wheelrpm*grrear)/1000]); % calculates regen energy (KWh) from rear, limits torque based on motor
                  sectbrakingregen_inner = regen_eff*dtdeccel/3600*min([sectvdeccel*brakeforceinner/1000 wheelradpersec*grhubs*getMaxTorquehubs(wheelrpm*grhubs)/2000]); % calculates regen energy (KWh) from inner, limits torque based on motor
                  sectbrakingregen_outer = regen_eff*dtdeccel/3600*min([sectvdeccel*brakeforceouter/1000 wheelradpersec*grhubs*getMaxTorquehubs(wheelrpm*grhubs)/2000]); % calculates regen energy (KWh) from outer, limits torque based on motor
                 

                  disp("KWh Max vs Allowed front outer: "+[sectvdeccel*brakeforceouter/1000 wheelradpersec*grhubs*getMaxTorquehubs(wheelrpm*grhubs)/2000]) % kW (energy disapated by braking energy total can take for regen)
                  disp("KWh Max vs Allowed front inner: "+[sectvdeccel*brakeforceinner/1000 wheelradpersec*grhubs*getMaxTorquehubs(wheelrpm*grhubs)/2000]) % kW (energy disapated by braking energy total can take for regen)
                  disp("KWh Max vs Allowed rear: "+[sectvdeccel*brakeforcerear/1000 wheelradpersec*grrear*getMaxTorquerear(wheelrpm*grrear)/1000]) %kW
                  disp("Total power availible for regen: "+(sectbrakingregen_rear+sectbrakingregen_inner+sectbrakingregen_outer)/(regen_eff*dtdeccel/3600)) %kW
                  disp("Total power regen after efficiency: "+(sectbrakingregen_rear+sectbrakingregen_inner+sectbrakingregen_outer)/(dtdeccel/3600))

                  sectbrakingregen = sectbrakingregen + sectbrakingregen_rear+sectbrakingregen_inner+sectbrakingregen_outer; % calulates total braking regen in this section (KWh)
                  %disp((sectbrakingregen_rear+sectbrakingregen_inner+sectbrakingregen_outer)/(dtdeccel/3600*(sectvdeccel*brakeforcerear/1000+sectvdeccel*brakeforceinner/1000+sectvdeccel*brakeforceouter/1000)))
                  %disp([sectacurrentdeccel  (brakeforcerear+brakeforceinner+brakeforceouter+FDrag(sectvdeccel)+(4*Crr*g*m))/m]) %brakeforcerear brakeforceinner brakeforceouter])
                  eused_deccel_prediction = eused_deccel_prediction - (sectbrakingregen_rear+sectbrakingregen_inner+sectbrakingregen_outer); % predicts energy used (KWh)
           

                  %predict energy defused by aero in breaking
                  edefused_aero_prediction_braking = edefused_aero + FDrag(sectvdeccel)*dtdeccel*2.77778*10^(-7); %KWh


                  %update plot array in section:
                  brakingspeedvsdistance = vertcat(brakingspeedvsdistance, [d+sectd+sectddeccel, sectvdeccel, sectacurrentdeccel, time+sectt+secttdeccel, 0, 0, edefused_aero_prediction_braking, eused_deccel_prediction]);
                  sectacurrentdeccel = sectadeccel(sectvdeccel,sectacurrentdeccel);
                  sectacurrentdeccel_braking = sectadeccel_frombraking(sectvdeccel,sectacurrentdeccel_braking);
              end
              % if it does exceed section distance, sectvdeccel and
              % sectddeccel will be the values before it does
              % exceeding happens
             
          end
          
          %calculate time for decceleration
          dleft = ddeccel-sectddeccel;
          dtdeccel = dleft/sectvdeccel;
          %take regular step to land on final point
          secttdeccel = secttdeccel +  dtdeccel;
          sectddeccel = sectvdeccel*dtdeccel;
          
          % predict energy lost by braking
          sectvdeccel = sectvdeccel + dtdeccel*(sectadeccel(sectvdeccel,sectacurrentdeccel)); % m/s
          wheelrpm = sectvdeccel*60/(r*2*pi); % wheel rpm
          wheelradpersec = sectvdeccel/r; % wheel speed (rad/s)

          brakeforcerear = maxbrakeforcerear(sectvdeccel,sectacurrentdeccel); % calculate max brake force from rear wheels (N)
           if turnradius > 0  % if in turn, will vary brake force due to lateral and long load transfer  
                      brakeforceinner = maxbrakeforceinner(sectvdeccel,sectacurrentdeccel,turnradius); % calculate max brake force from front inner wheels (N)
                      brakeforceouter = maxbrakeforceouter(sectvdeccel,sectacurrentdeccel,turnradius); % calculate max brake force from front outer wheels (N)
           else % if not in turn, will base brake force in hubs due to only longitudinal load transfer
                      brakeforceinner = maxbrakeforcefront(sectvdeccel,sectacurrentdeccel)/2; % calculate max brake force from front  wheels (N)
                      brakeforceouter = maxbrakeforcefront(sectvdeccel,sectacurrentdeccel)/2; % calculate max brake force from front  wheels (N)
           end

          sectbrakingregen_rear = regen_eff*dtdeccel/3600*min([sectvdeccel*brakeforcerear/1000 wheelradpersec*grrear*getMaxTorquerear(wheelrpm*grrear)/1000]); % calculates regen energy (KWh) from rear, limits torque based on motor
          sectbrakingregen_inner = regen_eff*dtdeccel/3600*min([sectvdeccel*brakeforceinner/1000 wheelradpersec*grhubs*getMaxTorquehubs(wheelrpm*grhubs)/1000]); % calculates regen energy (KWh) from inner, limits torque based on motor
          sectbrakingregen_outer = regen_eff*dtdeccel/3600*min([sectvdeccel*brakeforceouter/1000 wheelradpersec*grhubs*getMaxTorquehubs(wheelrpm*grhubs)/1000]); % calculates regen energy (KWh) from outer, limits torque based on motor
                  
          sectbrakingregen = sectbrakingregen + sectbrakingregen_rear+sectbrakingregen_inner+sectbrakingregen_outer; % calulates total braking regen in this section (KWh)
          
          % if final velocity is greater than next section max v
          if sectvdeccel >= sectvmaxturnnext
              % won't clear turn, need to brake in previous time
              % instance. Set sectvfinal and secttfinal equal to the
              % prediction from previous instance
              % set final kinmatic values equal to their predictions
              secttfinal = secttfinalprediction; % time
              sectvfinal = sectvfinalprediction; % speed
              wheelrpm = sectvfinal*60/(r*2*pi); % wheel rpm
              sectafinal = sectafinalprediction; % acceleration
              eregen = sectbrakingregenprediction; % energy regen
              eregenarray = vertcat(eregenarray,[eregen,time]);% Update eregen to array
              speedvsdistance = vertcat(speedvsdistance,storedbrakingspeedvsdistance); % concatenate brakingspeedvsdistance array to speedvsdistance array
              %disp(storedbrakingspeedvsdistance)
              %pause(30)
            
              stopwhileloop = true; % stop the while loop
              eused = stored_eused_deccel_prediction;
          else
              % will clear turn, store speed and distances of current accel and deccel sections
              % continue iterating
              % set final kinmatic values equal to their predictions
              sectvfinalprediction = sectvdeccel;
              sectafinalprediction = sectadeccel(sectvdeccel, sectacurrentdeccel);
              secttfinalprediction = secttdeccel + tempsecttaccel;   
              sectbrakingregenprediction = sectbrakingregen;
              storedbrakingspeedvsdistance = brakingspeedvsdistance; % store previous array of braking speed vs distance;
              stored_eused_deccel_prediction =  eused_deccel_prediction;
          end
      
      

      end % end if of statement to determine if braking at current instance of time will allow driver to clear incoming turn

  % if iteration prediction exceeds section distance:
      if sectd_prediction > trackdistarray(i) && stopwhileloop == false
         %limit dt to not pass section distance
         dtlastaccel = (trackdistarray(i) - sectd)/sectv;
         %calculate final time, distance, and velocity
         sectt = sectt + dtlastaccel;
         sectd = sectd + dtlastaccel*sectv;
         sectv = sectv + dtlastaccel*secta;  
         wheelrpm = sectv*60/(r*2*pi); % wheel rpm
         % set final velocity
          secttfinal = sectt;
          sectvfinal = sectv;
          sectafinal = secta;
          
          if sectv > topspeed
              topspeed = sectv;
              topspeedsect = i;
          end
          eused = eused_prediction;
          stopwhileloop = true;
          
          edefused_aero_prediction = edefused_aero + (FDrag(sectv)*dtlastaccel*sectvfinal)*2.77778*10^(-7); %KWh
          edefused_aero = edefused_aero_prediction;

          PowerOutputEmrax = Fxrreart*sectv/1000; %Emrax power output in kW
          MaxPowerOutputEmrax = max(MaxPowerOutputEmrax,PowerOutputEmrax); % update value of MaxPowerOutputEmrax if PowerOutputEmrax exceeds MaxPowerOutputEmrax                
          MaxTorqueEmrax = max(MaxTorqueEmrax,TMrear);
          
          PowerOutputhub = Fxrhubst*sectv/1000; % Hub power output in kW
          MaxPowerOutputhub = max(MaxPowerOutputhub,PowerOutputhub); % update value of MaxPowerOutputHub if PowerOutputHub exceeds MaxPowerOutputEmrax                
          MaxTorquehub = max(MaxTorquehub,TMhubs);
          torquearray = vertcat(torquearray, [TMhubs,TMrear,time+sectt]);

          speedvsdistance = vertcat(speedvsdistance,[d+sectd, sectv, secta, time + secttfinal, PowerOutputEmrax, PowerOutputhub,edefused_aero, eused]);
          if trackdistarray(i,2) > 0
             lapvaluesarray = vertcat(lapvaluesarray, [d+sectd, sectv, time + secttfinal, PowerOutputEmrax, PowerOutputhub, edefused_aero]);
          end
          eregenarray = vertcat(eregenarray,[eregen,time+secttfinal]);
      end
      
      % if iteration doesn't exceed section distance and can clear
      % next section without braking yet:
      if stopwhileloop == false
          sectt = sectt + dt;
          sectv = sectv_prediction;
          % THIS LINE FOR VIEWING WHEN NOT COMPLETE SECTION
          % =========================================================
          %disp(TractiveLimitrear(sectv,secta)+TractiveLimithubs(sectv,secta))
          %disp("Tractive Force "+Fxrt)
          %disp("Accel "+secta)
          
          edefused_aero = edefused_aero_prediction;
          
          if sectv >= sectvmaxbymotorrear(TMrear,eused_prediction)||sectv >= sectvmaxbymotorhubs(TMhubs,eused_prediction) || sectv >=  sectvmaxbyturn % limit max speed
              hitmaxspeed = true;
              MAXSPEEDINSECTION = sectv;
          end
          if sectv > topspeed % update top speed of track
              topspeed = sectv;
              topspeedsect = i;               
          end

          sectd = sectd_prediction; % set section distance equal to prediction distance
          eused = eused_prediction; % set energy used equal to prediction energy used
          
          PowerOutputEmrax = Fxrreart*sectv/1000; % Emrax power output in this dt
          MaxPowerOutputEmrax = max(MaxPowerOutputEmrax,PowerOutputEmrax); % update value of MaxPowerOutputEmrax if PowerOutputEmrax exceeds MaxPowerOutputEmrax                
          MaxTorqueEmrax = max(MaxTorqueEmrax,TMrear); % update value of MaxTorqueEmrax if TMrear exceeds MaxTorqueEmrax                

         PowerOutputhub = Fxrhubst*sectv/1000; % Hub power output in
          %this dt kW
          MaxPowerOutputhub = max(MaxPowerOutputhub,PowerOutputhub); % update value of MaxPowerOutputhub if PowerOutputhub exceeds MaxPowerOutputhub                
          MaxTorquehub = max(MaxTorquehub,TMhubs);% update value of MaxTorquehub if TMhubs exceeds MaxTorquehub
           torquearray = vertcat(torquearray, [TMhubs,TMrear,time+sectt]);
          

          speedvsdistance = vertcat(speedvsdistance,[d+sectd, sectv, secta,time + sectt, PowerOutputEmrax, PowerOutputhub,edefused_aero, eused]); % concatenate speed vs distance array with values from this section
           if trackdistarray(i,2) > 0 % if section is turn
                  lapvaluesarray = vertcat(lapvaluesarray, [d+sectd, sectv,time + sectt, PowerOutputEmrax, PowerOutputhub,edefused_aero]); % concatenate blue square mark on if turn graph
           end
      
      end
  end % end of while loop to calculate distance, speed, and time while accelerating in a section  

     
time = time + secttfinal; % iterate t er section/before next section
d = d+trackdistarray(i,1); %iterate total distance
trackarray(i,2) = secttfinal; % record section time
trackarray(i,3) = sectvfinal; % record section final speed
v = sectvfinal; % update v to equal to final v in section
a = sectafinal; % update a to equal final a in section

% update/concatenate speedvsdistance and straightorturn arrays
speedvsdistance = vertcat(speedvsdistance,[d, sectvfinal, sectafinal,time,PowerOutputEmrax, PowerOutputhub, edefused_aero, eused]);
if trackdistarray(i,2) > 0
 lapvaluesarray = vertcat(lapvaluesarray, [d, sectvfinal,time,PowerOutputEmrax, PowerOutputhub,edefused_aero]);
end

i = i+1; % iterate next section
if lastsect % if at last section
  i = height(trackdistarray); % set i to end
end

end % end of while loops that iterate through distancearray lenghts
tfinal = time; % determine final time
disp(trackarray) % for reference

if graphspeedvsdistance % if need graph


  % SPEED AND ACCEL VS DISTANCE
  figure
  hold on
  title ("Acceleration and Speed " + " with " + grrear+ " rear gear ratio and " + grhubs + " hub gear ratio vs distance")
  plot(speedvsdistance(:,1),speedvsdistance(:,2),"g")
  plot(speedvsdistance(:,1),speedvsdistance(:,3),"r")

  j = 1;

  while j < height(lapvaluesarray)
      plot(lapvaluesarray(j,1), lapvaluesarray(j,2),"squareb")
      j= j+1;
  end
  legend('Speed (straight)','Acceleration','Speed (turn)')
  ylabel ("Speed (m/s) Acceleration (m/s^2)") 
  xlabel ("distance (m)") 
  legend('Speed (straight)','Acceleration','Speed (turn)')
  hold off
  
 
  
  % SPEED AND ACCEL VS TIME 
  figure
  hold on
  title ("Acceleration and Speed " +" with " + grrear+ " rear gear ratio and " + grhubs + " hub gear ratio vs time")
  plot(speedvsdistance(:,4),speedvsdistance(:,2),"g")
  plot(speedvsdistance(:,4),speedvsdistance(:,3),"r")

  j = 1;
  while j < height(lapvaluesarray)
      plot(lapvaluesarray(j,3), lapvaluesarray(j,2),"squareb")
      j= j+1;
  end
  legend('Speed (straight)','Acceleration','Speed (turn)')
  ylabel ("Speed (m/s) Acceleration (m/s^2)") 
  xlabel ("Time (s)") 
  hold off

  % POWER OUTPUT OF MOTORS VS TIME
  figure
  hold on
  title ("Power output of motors" + " with " + grrear+ " rear gear ratio and " + grhubs + " hub gear ratio")
  matlaborange = "#0072BD";
  matlabblue = 	"#D95319";
  plot(speedvsdistance(:,4),speedvsdistance(:,6))
  plot(speedvsdistance(:,4),speedvsdistance(:,5))
  plot(speedvsdistance(:,4),speedvsdistance(:,5)+speedvsdistance(:,6))
  

  %j = 1;
  %while j <= length(straightorturn)
  %plot(straightorturn(j,3), straightorturn(j,4),"squareb")
  %j= j+1;
  %end
  
  legend('OUTPUT HUBS','OUTPUT EMRAX',"OUTPUT TOTAL")
  ylabel ("POWER OUTPUT (kW)") 
  xlabel ("Time (s)") 
  hold off
  
  % Energy regened in braking vs time
  figure
  hold on % regen graph
  title ("Energy regened in braking vs time" + " with " + grrear+ " rear gear ratio and " + grhubs + " hub gear ratio")

  plot(eregenarray(:,2),eregenarray(:,1),"b-")
  
  ylabel ("Energy (kWh)") 
  xlabel ("Time (s)") 
  hold off

  disp("Max power output from Emrax: " + MaxPowerOutputEmrax + " kW")
  disp("Max torque output from Emrax: " + MaxTorqueEmrax + " Nm")
  disp("Max power output from hub motors: " + MaxPowerOutputhub + " kW")
  disp("Max torque output from hub motors: " + MaxTorquehub + " Nm")
 

  % Energy Defused by Aero vs time
  figure
  hold on
  title ("Energy defused by Aero" + " with " + grrear+ " rear gear ratio and " + grhubs + " hub gear ratio")
  matlabblue = 	"#D95319";
  plot(speedvsdistance(:,4),speedvsdistance(:,7))
  
  legend('Total Energy Defused by Aero')
  ylabel ("POWER OUTPUT (KWh)") 
  xlabel ("Time (s)") 
  hold off

  % NET ENERGY USED VS TIME 
  figure
  hold on
  title ("NET ENERGY USED VS TIME" +" with " + grrear+ " rear gear ratio and " + grhubs + " hub gear ratio vs time")
  plot(speedvsdistance(:,4),speedvsdistance(:,8),"r-")

  legend('Net Energy Used')
  ylabel ("Energy (KWh)") 
  xlabel ("Time (s)") 
  hold off

 
  % Torque of hubs and rear vs time

  figure
  hold on
  title ("Torque vs time")
  plot(torquearray(:,3),torquearray(:,2),"r-")
  plot(torquearray(:,3),torquearray(:,1),"b-")
  legend('Rear (Nm)','Front (Nm)')
  ylabel ("Torque (Nm)") 
  xlabel ("Time (s)") 
  hold off
end
%disp(mean(torquearray))
%disp(torquearray)

end

     