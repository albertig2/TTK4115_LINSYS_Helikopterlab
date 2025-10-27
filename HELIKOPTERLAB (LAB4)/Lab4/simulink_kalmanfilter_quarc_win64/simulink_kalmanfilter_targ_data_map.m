  function targMap = targDataMap(),

  ;%***********************
  ;% Create Parameter Map *
  ;%***********************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 8;
    sectIdxOffset = 0;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc paramMap
    ;%
    paramMap.nSections           = nTotSects;
    paramMap.sectIdxOffset       = sectIdxOffset;
      paramMap.sections(nTotSects) = dumSection; %prealloc
    paramMap.nTotData            = -1;
    
    ;%
    ;% Auto data (simulink_kalmanfilter_P)
    ;%
      section.nData     = 10;
      section.data(10)  = dumData; %prealloc
      
	  ;% simulink_kalmanfilter_P.Ad
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% simulink_kalmanfilter_P.Bd
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 36;
	
	  ;% simulink_kalmanfilter_P.Cd
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 48;
	
	  ;% simulink_kalmanfilter_P.F
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 78;
	
	  ;% simulink_kalmanfilter_P.Joystick_gain_x
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 82;
	
	  ;% simulink_kalmanfilter_P.Joystick_gain_y
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 83;
	
	  ;% simulink_kalmanfilter_P.K_LQR
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 84;
	
	  ;% simulink_kalmanfilter_P.Qd
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 94;
	
	  ;% simulink_kalmanfilter_P.Rd
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 130;
	
	  ;% simulink_kalmanfilter_P.Vs_0
	  section.data(10).logicalSrcIdx = 9;
	  section.data(10).dtTransOffset = 155;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(1) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% simulink_kalmanfilter_P.HILWriteAnalog_channels
	  section.data(1).logicalSrcIdx = 10;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(2) = section;
      clear section
      
      section.nData     = 59;
      section.data(59)  = dumData; %prealloc
      
	  ;% simulink_kalmanfilter_P.HILInitialize_OOTerminate
	  section.data(1).logicalSrcIdx = 11;
	  section.data(1).dtTransOffset = 0;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_OOExit
	  section.data(2).logicalSrcIdx = 12;
	  section.data(2).dtTransOffset = 1;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_OOStart
	  section.data(3).logicalSrcIdx = 13;
	  section.data(3).dtTransOffset = 2;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_OOEnter
	  section.data(4).logicalSrcIdx = 14;
	  section.data(4).dtTransOffset = 3;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_AOFinal
	  section.data(5).logicalSrcIdx = 15;
	  section.data(5).dtTransOffset = 4;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_POFinal
	  section.data(6).logicalSrcIdx = 16;
	  section.data(6).dtTransOffset = 5;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_AIHigh
	  section.data(7).logicalSrcIdx = 17;
	  section.data(7).dtTransOffset = 6;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_AILow
	  section.data(8).logicalSrcIdx = 18;
	  section.data(8).dtTransOffset = 7;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_AOHigh
	  section.data(9).logicalSrcIdx = 19;
	  section.data(9).dtTransOffset = 8;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_AOLow
	  section.data(10).logicalSrcIdx = 20;
	  section.data(10).dtTransOffset = 9;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_AOInitial
	  section.data(11).logicalSrcIdx = 21;
	  section.data(11).dtTransOffset = 10;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_AOWatchdog
	  section.data(12).logicalSrcIdx = 22;
	  section.data(12).dtTransOffset = 11;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_POFrequency
	  section.data(13).logicalSrcIdx = 23;
	  section.data(13).dtTransOffset = 12;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_POLeading
	  section.data(14).logicalSrcIdx = 24;
	  section.data(14).dtTransOffset = 13;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_POTrailing
	  section.data(15).logicalSrcIdx = 25;
	  section.data(15).dtTransOffset = 14;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_POInitial
	  section.data(16).logicalSrcIdx = 26;
	  section.data(16).dtTransOffset = 15;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_POWatchdog
	  section.data(17).logicalSrcIdx = 27;
	  section.data(17).dtTransOffset = 16;
	
	  ;% simulink_kalmanfilter_P.RateTransitionx_InitialConditio
	  section.data(18).logicalSrcIdx = 28;
	  section.data(18).dtTransOffset = 17;
	
	  ;% simulink_kalmanfilter_P.DeadZonex_Start
	  section.data(19).logicalSrcIdx = 29;
	  section.data(19).dtTransOffset = 18;
	
	  ;% simulink_kalmanfilter_P.DeadZonex_End
	  section.data(20).logicalSrcIdx = 30;
	  section.data(20).dtTransOffset = 19;
	
	  ;% simulink_kalmanfilter_P.Gainx_Gain
	  section.data(21).logicalSrcIdx = 31;
	  section.data(21).dtTransOffset = 20;
	
	  ;% simulink_kalmanfilter_P.UnitDelay_InitialCondition
	  section.data(22).logicalSrcIdx = 32;
	  section.data(22).dtTransOffset = 21;
	
	  ;% simulink_kalmanfilter_P.UnitDelay1_InitialCondition
	  section.data(23).logicalSrcIdx = 33;
	  section.data(23).dtTransOffset = 22;
	
	  ;% simulink_kalmanfilter_P.Constant_Value
	  section.data(24).logicalSrcIdx = 34;
	  section.data(24).dtTransOffset = 23;
	
	  ;% simulink_kalmanfilter_P.Constant_Value_m
	  section.data(25).logicalSrcIdx = 35;
	  section.data(25).dtTransOffset = 24;
	
	  ;% simulink_kalmanfilter_P.Constant1_Value
	  section.data(26).logicalSrcIdx = 36;
	  section.data(26).dtTransOffset = 25;
	
	  ;% simulink_kalmanfilter_P.Memory_InitialCondition
	  section.data(27).logicalSrcIdx = 37;
	  section.data(27).dtTransOffset = 26;
	
	  ;% simulink_kalmanfilter_P.Gain2_Gain
	  section.data(28).logicalSrcIdx = 38;
	  section.data(28).dtTransOffset = 36;
	
	  ;% simulink_kalmanfilter_P.Gain1_Gain
	  section.data(29).logicalSrcIdx = 39;
	  section.data(29).dtTransOffset = 45;
	
	  ;% simulink_kalmanfilter_P.Constant1_Value_n
	  section.data(30).logicalSrcIdx = 40;
	  section.data(30).dtTransOffset = 54;
	
	  ;% simulink_kalmanfilter_P.RateTransitiony_InitialConditio
	  section.data(31).logicalSrcIdx = 41;
	  section.data(31).dtTransOffset = 55;
	
	  ;% simulink_kalmanfilter_P.DeadZoney_Start
	  section.data(32).logicalSrcIdx = 42;
	  section.data(32).dtTransOffset = 56;
	
	  ;% simulink_kalmanfilter_P.DeadZoney_End
	  section.data(33).logicalSrcIdx = 43;
	  section.data(33).dtTransOffset = 57;
	
	  ;% simulink_kalmanfilter_P.Gainy_Gain
	  section.data(34).logicalSrcIdx = 44;
	  section.data(34).dtTransOffset = 58;
	
	  ;% simulink_kalmanfilter_P.Integrator_IC
	  section.data(35).logicalSrcIdx = 45;
	  section.data(35).dtTransOffset = 59;
	
	  ;% simulink_kalmanfilter_P.Integrator1_IC
	  section.data(36).logicalSrcIdx = 46;
	  section.data(36).dtTransOffset = 60;
	
	  ;% simulink_kalmanfilter_P.TravelCounttorad_Gain
	  section.data(37).logicalSrcIdx = 47;
	  section.data(37).dtTransOffset = 61;
	
	  ;% simulink_kalmanfilter_P.TravelTransferFcn_A
	  section.data(38).logicalSrcIdx = 48;
	  section.data(38).dtTransOffset = 62;
	
	  ;% simulink_kalmanfilter_P.TravelTransferFcn_C
	  section.data(39).logicalSrcIdx = 49;
	  section.data(39).dtTransOffset = 63;
	
	  ;% simulink_kalmanfilter_P.TravelTransferFcn_D
	  section.data(40).logicalSrcIdx = 50;
	  section.data(40).dtTransOffset = 64;
	
	  ;% simulink_kalmanfilter_P.PitchCounttorad_Gain
	  section.data(41).logicalSrcIdx = 51;
	  section.data(41).dtTransOffset = 65;
	
	  ;% simulink_kalmanfilter_P.PitchTransferFcn_A
	  section.data(42).logicalSrcIdx = 52;
	  section.data(42).dtTransOffset = 66;
	
	  ;% simulink_kalmanfilter_P.PitchTransferFcn_C
	  section.data(43).logicalSrcIdx = 53;
	  section.data(43).dtTransOffset = 67;
	
	  ;% simulink_kalmanfilter_P.PitchTransferFcn_D
	  section.data(44).logicalSrcIdx = 54;
	  section.data(44).dtTransOffset = 68;
	
	  ;% simulink_kalmanfilter_P.ElevationCounttorad_Gain
	  section.data(45).logicalSrcIdx = 55;
	  section.data(45).dtTransOffset = 69;
	
	  ;% simulink_kalmanfilter_P.Constant_Value_p
	  section.data(46).logicalSrcIdx = 56;
	  section.data(46).dtTransOffset = 70;
	
	  ;% simulink_kalmanfilter_P.ElevationTransferFcn_A
	  section.data(47).logicalSrcIdx = 57;
	  section.data(47).dtTransOffset = 71;
	
	  ;% simulink_kalmanfilter_P.ElevationTransferFcn_C
	  section.data(48).logicalSrcIdx = 58;
	  section.data(48).dtTransOffset = 72;
	
	  ;% simulink_kalmanfilter_P.ElevationTransferFcn_D
	  section.data(49).logicalSrcIdx = 59;
	  section.data(49).dtTransOffset = 73;
	
	  ;% simulink_kalmanfilter_P.Backgain_Gain
	  section.data(50).logicalSrcIdx = 60;
	  section.data(50).dtTransOffset = 74;
	
	  ;% simulink_kalmanfilter_P.Frontgain_Gain
	  section.data(51).logicalSrcIdx = 61;
	  section.data(51).dtTransOffset = 75;
	
	  ;% simulink_kalmanfilter_P.FrontmotorSaturation_UpperSat
	  section.data(52).logicalSrcIdx = 62;
	  section.data(52).dtTransOffset = 76;
	
	  ;% simulink_kalmanfilter_P.FrontmotorSaturation_LowerSat
	  section.data(53).logicalSrcIdx = 63;
	  section.data(53).dtTransOffset = 77;
	
	  ;% simulink_kalmanfilter_P.BackmotorSaturation_UpperSat
	  section.data(54).logicalSrcIdx = 64;
	  section.data(54).dtTransOffset = 78;
	
	  ;% simulink_kalmanfilter_P.BackmotorSaturation_LowerSat
	  section.data(55).logicalSrcIdx = 65;
	  section.data(55).dtTransOffset = 79;
	
	  ;% simulink_kalmanfilter_P.Integrator_IC_k
	  section.data(56).logicalSrcIdx = 66;
	  section.data(56).dtTransOffset = 80;
	
	  ;% simulink_kalmanfilter_P.Integrator_UpperSat
	  section.data(57).logicalSrcIdx = 67;
	  section.data(57).dtTransOffset = 81;
	
	  ;% simulink_kalmanfilter_P.Integrator_LowerSat
	  section.data(58).logicalSrcIdx = 68;
	  section.data(58).dtTransOffset = 82;
	
	  ;% simulink_kalmanfilter_P.K_ei_Gain
	  section.data(59).logicalSrcIdx = 69;
	  section.data(59).dtTransOffset = 83;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(3) = section;
      clear section
      
      section.nData     = 10;
      section.data(10)  = dumData; %prealloc
      
	  ;% simulink_kalmanfilter_P.HILInitialize_CKChannels
	  section.data(1).logicalSrcIdx = 70;
	  section.data(1).dtTransOffset = 0;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_DOWatchdog
	  section.data(2).logicalSrcIdx = 71;
	  section.data(2).dtTransOffset = 3;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_EIInitial
	  section.data(3).logicalSrcIdx = 72;
	  section.data(3).dtTransOffset = 4;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_POModes
	  section.data(4).logicalSrcIdx = 73;
	  section.data(4).dtTransOffset = 5;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_POConfiguration
	  section.data(5).logicalSrcIdx = 74;
	  section.data(5).dtTransOffset = 6;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_POAlignment
	  section.data(6).logicalSrcIdx = 75;
	  section.data(6).dtTransOffset = 7;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_POPolarity
	  section.data(7).logicalSrcIdx = 76;
	  section.data(7).dtTransOffset = 8;
	
	  ;% simulink_kalmanfilter_P.HILReadEncoderTimebase_Clock
	  section.data(8).logicalSrcIdx = 77;
	  section.data(8).dtTransOffset = 9;
	
	  ;% simulink_kalmanfilter_P.StreamCall1_SendBufferSize
	  section.data(9).logicalSrcIdx = 78;
	  section.data(9).dtTransOffset = 10;
	
	  ;% simulink_kalmanfilter_P.StreamCall1_ReceiveBufferSize
	  section.data(10).logicalSrcIdx = 79;
	  section.data(10).dtTransOffset = 11;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(4) = section;
      clear section
      
      section.nData     = 8;
      section.data(8)  = dumData; %prealloc
      
	  ;% simulink_kalmanfilter_P.HILInitialize_AIChannels
	  section.data(1).logicalSrcIdx = 80;
	  section.data(1).dtTransOffset = 0;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_AOChannels
	  section.data(2).logicalSrcIdx = 81;
	  section.data(2).dtTransOffset = 8;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_EIChannels
	  section.data(3).logicalSrcIdx = 82;
	  section.data(3).dtTransOffset = 16;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_EIQuadrature
	  section.data(4).logicalSrcIdx = 83;
	  section.data(4).dtTransOffset = 24;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_POChannels
	  section.data(5).logicalSrcIdx = 84;
	  section.data(5).dtTransOffset = 25;
	
	  ;% simulink_kalmanfilter_P.HILReadEncoderTimebase_Channels
	  section.data(6).logicalSrcIdx = 85;
	  section.data(6).dtTransOffset = 33;
	
	  ;% simulink_kalmanfilter_P.HILReadEncoderTimebase_SamplesI
	  section.data(7).logicalSrcIdx = 86;
	  section.data(7).dtTransOffset = 36;
	
	  ;% simulink_kalmanfilter_P.StreamFormattedWrite_MaxUnits
	  section.data(8).logicalSrcIdx = 87;
	  section.data(8).dtTransOffset = 37;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(5) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% simulink_kalmanfilter_P.GameController_BufferSize
	  section.data(1).logicalSrcIdx = 88;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(6) = section;
      clear section
      
      section.nData     = 41;
      section.data(41)  = dumData; %prealloc
      
	  ;% simulink_kalmanfilter_P.HILInitialize_Active
	  section.data(1).logicalSrcIdx = 89;
	  section.data(1).dtTransOffset = 0;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_AOTerminate
	  section.data(2).logicalSrcIdx = 90;
	  section.data(2).dtTransOffset = 1;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_AOExit
	  section.data(3).logicalSrcIdx = 91;
	  section.data(3).dtTransOffset = 2;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_DOTerminate
	  section.data(4).logicalSrcIdx = 92;
	  section.data(4).dtTransOffset = 3;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_DOExit
	  section.data(5).logicalSrcIdx = 93;
	  section.data(5).dtTransOffset = 4;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_POTerminate
	  section.data(6).logicalSrcIdx = 94;
	  section.data(6).dtTransOffset = 5;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_POExit
	  section.data(7).logicalSrcIdx = 95;
	  section.data(7).dtTransOffset = 6;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_CKPStart
	  section.data(8).logicalSrcIdx = 96;
	  section.data(8).dtTransOffset = 7;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_CKPEnter
	  section.data(9).logicalSrcIdx = 97;
	  section.data(9).dtTransOffset = 8;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_CKStart
	  section.data(10).logicalSrcIdx = 98;
	  section.data(10).dtTransOffset = 9;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_CKEnter
	  section.data(11).logicalSrcIdx = 99;
	  section.data(11).dtTransOffset = 10;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_AIPStart
	  section.data(12).logicalSrcIdx = 100;
	  section.data(12).dtTransOffset = 11;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_AIPEnter
	  section.data(13).logicalSrcIdx = 101;
	  section.data(13).dtTransOffset = 12;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_AOPStart
	  section.data(14).logicalSrcIdx = 102;
	  section.data(14).dtTransOffset = 13;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_AOPEnter
	  section.data(15).logicalSrcIdx = 103;
	  section.data(15).dtTransOffset = 14;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_AOStart
	  section.data(16).logicalSrcIdx = 104;
	  section.data(16).dtTransOffset = 15;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_AOEnter
	  section.data(17).logicalSrcIdx = 105;
	  section.data(17).dtTransOffset = 16;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_AOReset
	  section.data(18).logicalSrcIdx = 106;
	  section.data(18).dtTransOffset = 17;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_DOPStart
	  section.data(19).logicalSrcIdx = 107;
	  section.data(19).dtTransOffset = 18;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_DOPEnter
	  section.data(20).logicalSrcIdx = 108;
	  section.data(20).dtTransOffset = 19;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_DOStart
	  section.data(21).logicalSrcIdx = 109;
	  section.data(21).dtTransOffset = 20;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_DOEnter
	  section.data(22).logicalSrcIdx = 110;
	  section.data(22).dtTransOffset = 21;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_DOReset
	  section.data(23).logicalSrcIdx = 111;
	  section.data(23).dtTransOffset = 22;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_EIPStart
	  section.data(24).logicalSrcIdx = 112;
	  section.data(24).dtTransOffset = 23;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_EIPEnter
	  section.data(25).logicalSrcIdx = 113;
	  section.data(25).dtTransOffset = 24;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_EIStart
	  section.data(26).logicalSrcIdx = 114;
	  section.data(26).dtTransOffset = 25;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_EIEnter
	  section.data(27).logicalSrcIdx = 115;
	  section.data(27).dtTransOffset = 26;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_POPStart
	  section.data(28).logicalSrcIdx = 116;
	  section.data(28).dtTransOffset = 27;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_POPEnter
	  section.data(29).logicalSrcIdx = 117;
	  section.data(29).dtTransOffset = 28;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_POStart
	  section.data(30).logicalSrcIdx = 118;
	  section.data(30).dtTransOffset = 29;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_POEnter
	  section.data(31).logicalSrcIdx = 119;
	  section.data(31).dtTransOffset = 30;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_POReset
	  section.data(32).logicalSrcIdx = 120;
	  section.data(32).dtTransOffset = 31;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_OOReset
	  section.data(33).logicalSrcIdx = 121;
	  section.data(33).dtTransOffset = 32;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_DOFinal
	  section.data(34).logicalSrcIdx = 122;
	  section.data(34).dtTransOffset = 33;
	
	  ;% simulink_kalmanfilter_P.HILInitialize_DOInitial
	  section.data(35).logicalSrcIdx = 123;
	  section.data(35).dtTransOffset = 34;
	
	  ;% simulink_kalmanfilter_P.HILReadEncoderTimebase_Active
	  section.data(36).logicalSrcIdx = 124;
	  section.data(36).dtTransOffset = 35;
	
	  ;% simulink_kalmanfilter_P.StreamCall1_NonBlocking
	  section.data(37).logicalSrcIdx = 125;
	  section.data(37).dtTransOffset = 36;
	
	  ;% simulink_kalmanfilter_P.StreamCall1_Active
	  section.data(38).logicalSrcIdx = 126;
	  section.data(38).dtTransOffset = 37;
	
	  ;% simulink_kalmanfilter_P.HILWriteAnalog_Active
	  section.data(39).logicalSrcIdx = 127;
	  section.data(39).dtTransOffset = 38;
	
	  ;% simulink_kalmanfilter_P.GameController_AutoCenter
	  section.data(40).logicalSrcIdx = 128;
	  section.data(40).dtTransOffset = 39;
	
	  ;% simulink_kalmanfilter_P.GameController_Enabled
	  section.data(41).logicalSrcIdx = 129;
	  section.data(41).dtTransOffset = 40;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(7) = section;
      clear section
      
      section.nData     = 5;
      section.data(5)  = dumData; %prealloc
      
	  ;% simulink_kalmanfilter_P.HILReadEncoderTimebase_Overflow
	  section.data(1).logicalSrcIdx = 130;
	  section.data(1).dtTransOffset = 0;
	
	  ;% simulink_kalmanfilter_P.StringConstant_Value
	  section.data(2).logicalSrcIdx = 131;
	  section.data(2).dtTransOffset = 1;
	
	  ;% simulink_kalmanfilter_P.StreamCall1_URI
	  section.data(3).logicalSrcIdx = 132;
	  section.data(3).dtTransOffset = 256;
	
	  ;% simulink_kalmanfilter_P.StreamCall1_Endian
	  section.data(4).logicalSrcIdx = 133;
	  section.data(4).dtTransOffset = 257;
	
	  ;% simulink_kalmanfilter_P.GameController_ControllerNumber
	  section.data(5).logicalSrcIdx = 134;
	  section.data(5).dtTransOffset = 258;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(8) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (parameter)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    paramMap.nTotData = nTotData;
    


  ;%**************************
  ;% Create Block Output Map *
  ;%**************************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 2;
    sectIdxOffset = 0;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc sigMap
    ;%
    sigMap.nSections           = nTotSects;
    sigMap.sectIdxOffset       = sectIdxOffset;
      sigMap.sections(nTotSects) = dumSection; %prealloc
    sigMap.nTotData            = -1;
    
    ;%
    ;% Auto data (simulink_kalmanfilter_B)
    ;%
      section.nData     = 30;
      section.data(30)  = dumData; %prealloc
      
	  ;% simulink_kalmanfilter_B.RateTransitionx
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% simulink_kalmanfilter_B.Joystick_gain_x
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 1;
	
	  ;% simulink_kalmanfilter_B.x_bar
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 2;
	
	  ;% simulink_kalmanfilter_B.Switch
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 8;
	
	  ;% simulink_kalmanfilter_B.p
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 18;
	
	  ;% simulink_kalmanfilter_B.Gain1
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 19;
	
	  ;% simulink_kalmanfilter_B.e
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 22;
	
	  ;% simulink_kalmanfilter_B.RateTransitiony
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 23;
	
	  ;% simulink_kalmanfilter_B.Joystick_gain_y
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 24;
	
	  ;% simulink_kalmanfilter_B.Gain
	  section.data(10).logicalSrcIdx = 9;
	  section.data(10).dtTransOffset = 25;
	
	  ;% simulink_kalmanfilter_B.Sum
	  section.data(11).logicalSrcIdx = 10;
	  section.data(11).dtTransOffset = 27;
	
	  ;% simulink_kalmanfilter_B.TravelCounttorad
	  section.data(12).logicalSrcIdx = 11;
	  section.data(12).dtTransOffset = 29;
	
	  ;% simulink_kalmanfilter_B.TravelTransferFcn
	  section.data(13).logicalSrcIdx = 12;
	  section.data(13).dtTransOffset = 30;
	
	  ;% simulink_kalmanfilter_B.PitchCounttorad
	  section.data(14).logicalSrcIdx = 13;
	  section.data(14).dtTransOffset = 31;
	
	  ;% simulink_kalmanfilter_B.PitchTransferFcn
	  section.data(15).logicalSrcIdx = 14;
	  section.data(15).dtTransOffset = 32;
	
	  ;% simulink_kalmanfilter_B.ElevationCounttorad
	  section.data(16).logicalSrcIdx = 15;
	  section.data(16).dtTransOffset = 33;
	
	  ;% simulink_kalmanfilter_B.Sum_m
	  section.data(17).logicalSrcIdx = 16;
	  section.data(17).dtTransOffset = 34;
	
	  ;% simulink_kalmanfilter_B.ElevationTransferFcn
	  section.data(18).logicalSrcIdx = 17;
	  section.data(18).dtTransOffset = 35;
	
	  ;% simulink_kalmanfilter_B.Sum_o
	  section.data(19).logicalSrcIdx = 18;
	  section.data(19).dtTransOffset = 36;
	
	  ;% simulink_kalmanfilter_B.y_vector
	  section.data(20).logicalSrcIdx = 19;
	  section.data(20).dtTransOffset = 37;
	
	  ;% simulink_kalmanfilter_B.FrontmotorSaturation
	  section.data(21).logicalSrcIdx = 20;
	  section.data(21).dtTransOffset = 42;
	
	  ;% simulink_kalmanfilter_B.BackmotorSaturation
	  section.data(22).logicalSrcIdx = 21;
	  section.data(22).dtTransOffset = 43;
	
	  ;% simulink_kalmanfilter_B.Sum1
	  section.data(23).logicalSrcIdx = 22;
	  section.data(23).dtTransOffset = 44;
	
	  ;% simulink_kalmanfilter_B.Sum2
	  section.data(24).logicalSrcIdx = 23;
	  section.data(24).dtTransOffset = 45;
	
	  ;% simulink_kalmanfilter_B.K_ei
	  section.data(25).logicalSrcIdx = 24;
	  section.data(25).dtTransOffset = 46;
	
	  ;% simulink_kalmanfilter_B.x_bar_k
	  section.data(26).logicalSrcIdx = 25;
	  section.data(26).dtTransOffset = 47;
	
	  ;% simulink_kalmanfilter_B.P_bar
	  section.data(27).logicalSrcIdx = 26;
	  section.data(27).dtTransOffset = 53;
	
	  ;% simulink_kalmanfilter_B.x_hat
	  section.data(28).logicalSrcIdx = 27;
	  section.data(28).dtTransOffset = 89;
	
	  ;% simulink_kalmanfilter_B.P_hat
	  section.data(29).logicalSrcIdx = 28;
	  section.data(29).dtTransOffset = 95;
	
	  ;% simulink_kalmanfilter_B.euler_rates
	  section.data(30).logicalSrcIdx = 29;
	  section.data(30).dtTransOffset = 131;
	
      nTotData = nTotData + section.nData;
      sigMap.sections(1) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% simulink_kalmanfilter_B.StreamCall1_o2
	  section.data(1).logicalSrcIdx = 30;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      sigMap.sections(2) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (signal)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    sigMap.nTotData = nTotData;
    


  ;%*******************
  ;% Create DWork Map *
  ;%*******************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 10;
    sectIdxOffset = 2;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc dworkMap
    ;%
    dworkMap.nSections           = nTotSects;
    dworkMap.sectIdxOffset       = sectIdxOffset;
      dworkMap.sections(nTotSects) = dumSection; %prealloc
    dworkMap.nTotData            = -1;
    
    ;%
    ;% Auto data (simulink_kalmanfilter_DW)
    ;%
      section.nData     = 14;
      section.data(14)  = dumData; %prealloc
      
	  ;% simulink_kalmanfilter_DW.UnitDelay_DSTATE
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% simulink_kalmanfilter_DW.UnitDelay1_DSTATE
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 6;
	
	  ;% simulink_kalmanfilter_DW.HILInitialize_AIMinimums
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 42;
	
	  ;% simulink_kalmanfilter_DW.HILInitialize_AIMaximums
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 50;
	
	  ;% simulink_kalmanfilter_DW.HILInitialize_AOMinimums
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 58;
	
	  ;% simulink_kalmanfilter_DW.HILInitialize_AOMaximums
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 66;
	
	  ;% simulink_kalmanfilter_DW.HILInitialize_AOVoltages
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 74;
	
	  ;% simulink_kalmanfilter_DW.HILInitialize_FilterFrequency
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 82;
	
	  ;% simulink_kalmanfilter_DW.HILInitialize_POSortedFreqs
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 90;
	
	  ;% simulink_kalmanfilter_DW.HILInitialize_POValues
	  section.data(10).logicalSrcIdx = 9;
	  section.data(10).dtTransOffset = 98;
	
	  ;% simulink_kalmanfilter_DW.RateTransitionx_Buffer0
	  section.data(11).logicalSrcIdx = 10;
	  section.data(11).dtTransOffset = 106;
	
	  ;% simulink_kalmanfilter_DW.Memory_PreviousInput
	  section.data(12).logicalSrcIdx = 11;
	  section.data(12).dtTransOffset = 107;
	
	  ;% simulink_kalmanfilter_DW.RateTransitiony_Buffer0
	  section.data(13).logicalSrcIdx = 12;
	  section.data(13).dtTransOffset = 117;
	
	  ;% simulink_kalmanfilter_DW.HILWriteAnalog_Buffer
	  section.data(14).logicalSrcIdx = 13;
	  section.data(14).dtTransOffset = 118;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(1) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% simulink_kalmanfilter_DW.StreamCall1_Stream
	  section.data(1).logicalSrcIdx = 14;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(2) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% simulink_kalmanfilter_DW.GameController_Controller
	  section.data(1).logicalSrcIdx = 15;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(3) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% simulink_kalmanfilter_DW.HILInitialize_Card
	  section.data(1).logicalSrcIdx = 16;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(4) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% simulink_kalmanfilter_DW.HILReadEncoderTimebase_Task
	  section.data(1).logicalSrcIdx = 17;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(5) = section;
      clear section
      
      section.nData     = 18;
      section.data(18)  = dumData; %prealloc
      
	  ;% simulink_kalmanfilter_DW.Enc_PWORK.LoggedData
	  section.data(1).logicalSrcIdx = 18;
	  section.data(1).dtTransOffset = 0;
	
	  ;% simulink_kalmanfilter_DW.IMU_v_Enc_PWORK.LoggedData
	  section.data(2).logicalSrcIdx = 19;
	  section.data(2).dtTransOffset = 4;
	
	  ;% simulink_kalmanfilter_DW.Scope_PWORK.LoggedData
	  section.data(3).logicalSrcIdx = 20;
	  section.data(3).dtTransOffset = 8;
	
	  ;% simulink_kalmanfilter_DW.Scope1_PWORK.LoggedData
	  section.data(4).logicalSrcIdx = 21;
	  section.data(4).dtTransOffset = 14;
	
	  ;% simulink_kalmanfilter_DW.Scope2_PWORK.LoggedData
	  section.data(5).logicalSrcIdx = 22;
	  section.data(5).dtTransOffset = 16;
	
	  ;% simulink_kalmanfilter_DW.Scope4_PWORK.LoggedData
	  section.data(6).logicalSrcIdx = 23;
	  section.data(6).dtTransOffset = 18;
	
	  ;% simulink_kalmanfilter_DW.ToFile2_PWORK.FilePtr
	  section.data(7).logicalSrcIdx = 24;
	  section.data(7).dtTransOffset = 21;
	
	  ;% simulink_kalmanfilter_DW.ToFile3_PWORK.FilePtr
	  section.data(8).logicalSrcIdx = 25;
	  section.data(8).dtTransOffset = 22;
	
	  ;% simulink_kalmanfilter_DW.ElevationScoperads_PWORK.LoggedData
	  section.data(9).logicalSrcIdx = 26;
	  section.data(9).dtTransOffset = 23;
	
	  ;% simulink_kalmanfilter_DW.ElevationScoperad_PWORK.LoggedData
	  section.data(10).logicalSrcIdx = 27;
	  section.data(10).dtTransOffset = 24;
	
	  ;% simulink_kalmanfilter_DW.PitchScoperad_PWORK.LoggedData
	  section.data(11).logicalSrcIdx = 28;
	  section.data(11).dtTransOffset = 25;
	
	  ;% simulink_kalmanfilter_DW.PtichrateScoperads_PWORK.LoggedData
	  section.data(12).logicalSrcIdx = 29;
	  section.data(12).dtTransOffset = 26;
	
	  ;% simulink_kalmanfilter_DW.TravelrateScoperads_PWORK.LoggedData
	  section.data(13).logicalSrcIdx = 30;
	  section.data(13).dtTransOffset = 27;
	
	  ;% simulink_kalmanfilter_DW.TravelScoperad_PWORK.LoggedData
	  section.data(14).logicalSrcIdx = 31;
	  section.data(14).dtTransOffset = 28;
	
	  ;% simulink_kalmanfilter_DW.HILWriteAnalog_PWORK
	  section.data(15).logicalSrcIdx = 32;
	  section.data(15).dtTransOffset = 29;
	
	  ;% simulink_kalmanfilter_DW.Connected_PWORK.LoggedData
	  section.data(16).logicalSrcIdx = 33;
	  section.data(16).dtTransOffset = 30;
	
	  ;% simulink_kalmanfilter_DW.XScope_PWORK.LoggedData
	  section.data(17).logicalSrcIdx = 34;
	  section.data(17).dtTransOffset = 31;
	
	  ;% simulink_kalmanfilter_DW.YScope_PWORK.LoggedData
	  section.data(18).logicalSrcIdx = 35;
	  section.data(18).dtTransOffset = 32;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(6) = section;
      clear section
      
      section.nData     = 7;
      section.data(7)  = dumData; %prealloc
      
	  ;% simulink_kalmanfilter_DW.HILInitialize_ClockModes
	  section.data(1).logicalSrcIdx = 36;
	  section.data(1).dtTransOffset = 0;
	
	  ;% simulink_kalmanfilter_DW.HILInitialize_QuadratureModes
	  section.data(2).logicalSrcIdx = 37;
	  section.data(2).dtTransOffset = 3;
	
	  ;% simulink_kalmanfilter_DW.HILInitialize_InitialEICounts
	  section.data(3).logicalSrcIdx = 38;
	  section.data(3).dtTransOffset = 11;
	
	  ;% simulink_kalmanfilter_DW.HILInitialize_POModeValues
	  section.data(4).logicalSrcIdx = 39;
	  section.data(4).dtTransOffset = 19;
	
	  ;% simulink_kalmanfilter_DW.HILInitialize_POAlignValues
	  section.data(5).logicalSrcIdx = 40;
	  section.data(5).dtTransOffset = 27;
	
	  ;% simulink_kalmanfilter_DW.HILInitialize_POPolarityVals
	  section.data(6).logicalSrcIdx = 41;
	  section.data(6).dtTransOffset = 35;
	
	  ;% simulink_kalmanfilter_DW.HILReadEncoderTimebase_Buffer
	  section.data(7).logicalSrcIdx = 42;
	  section.data(7).dtTransOffset = 43;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(7) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% simulink_kalmanfilter_DW.HILInitialize_POSortedChans
	  section.data(1).logicalSrcIdx = 43;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(8) = section;
      clear section
      
      section.nData     = 2;
      section.data(2)  = dumData; %prealloc
      
	  ;% simulink_kalmanfilter_DW.ToFile2_IWORK.Count
	  section.data(1).logicalSrcIdx = 44;
	  section.data(1).dtTransOffset = 0;
	
	  ;% simulink_kalmanfilter_DW.ToFile3_IWORK.Count
	  section.data(2).logicalSrcIdx = 45;
	  section.data(2).dtTransOffset = 1;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(9) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% simulink_kalmanfilter_DW.StreamCall1_State
	  section.data(1).logicalSrcIdx = 46;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(10) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (dwork)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    dworkMap.nTotData = nTotData;
    


  ;%
  ;% Add individual maps to base struct.
  ;%

  targMap.paramMap  = paramMap;    
  targMap.signalMap = sigMap;
  targMap.dworkMap  = dworkMap;
  
  ;%
  ;% Add checksums to base struct.
  ;%


  targMap.checksum0 = 3855355860;
  targMap.checksum1 = 3663055974;
  targMap.checksum2 = 3582113415;
  targMap.checksum3 = 1460462062;

