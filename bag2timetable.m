function bag2timetable(bagName)

    %% load bag
    bagreader = rosbagreader(bagName);
    bagreader.AvailableTopics;
    
    bagVelo     = select(bagreader,"Topic","/WheelLoaderVelodynePackets");
    bagIMU      = select(bagreader,"Topic","/WheelLoaderIMUData");
    bagCamera   = select(bagreader,"Topic","/WheelLoaderCamera");
    bagOdom     = select(bagreader,"Topic","/WheelLoaderOdometry");
    
    msgsVelo = readMessages(bagVelo,"DataFormat","struct");
    veloReader = velodyneROSMessageReader(msgsVelo,'VLP16');
    
    %% create lidarPointClouds
    lidarPointClouds = timetable();
    n_frame = max(size(veloReader.Timestamps));
    for i_frame = 1:n_frame
        timeDuration = veloReader.Timestamps(i_frame);
        ptCloudObj = readFrame(veloReader,timeDuration);
        ptCloudObj = removeInvalidPoints(ptCloudObj);
        lidarPointClouds(i_frame,:) = {ptCloudObj};
    end
    lidarPointClouds.Time = duration(veloReader.Timestamps,'Format','mm:ss.SSSS');
    lidarPointClouds.Time = lidarPointClouds.Time - lidarPointClouds.Time(1); 
    lidarPointClouds.Properties.VariableNames = "PointCloud";
    
    %% create imuOrientations
    imuOrientations = timetable();
    n_meas = size(bagIMU.MessageList,1);
    msgs = readMessages(bagIMU,"DataFormat","struct");
    for i_meas = 1:n_meas
        orientStr = msgs{i_meas}.Orientation;
        orient = [orientStr.X, orientStr.Y, orientStr.Z, orientStr.W];
        imuOrientations(i_meas,:) = {orient};
    end
    imuOrientations.Properties.VariableNames = "Orientation";
    timeIMU = seconds( bagIMU.MessageList.Time - bagIMU.StartTime );
    imuOrientations.Time = duration(timeIMU,'Format','mm:ss.SSSS');
    
    %% align time
    if bagIMU.StartTime < bagVelo.StartTime
        diff = seconds(bagVelo.StartTime - bagIMU.StartTime);
        lidarPointClouds.Time = lidarPointClouds.Time + diff;
    end
    
    if bagIMU.StartTime > bagVelo.StartTime
        diff = seconds(bagIMU.StartTime - bagVelo.StartTime);
        imuOrientations.Time = imuOrientations.Time + diff;
    end
    
    sceneName = erase(bagName,'.bag');
    save(strcat('SLAMData/lidarROS_',sceneName), 'lidarPointClouds','-v7.3');
    save(strcat('SLAMData/imuROS_',sceneName),'imuOrientations','-v7.3');
    
    %% save odometry
    odom = timetable();
    n_meas = size(bagOdom.MessageList,1);
    msgs = readMessages(bagOdom,"Dataformat","struct");
    for i_meas = 1:n_meas
        odomStr = msgs{i_meas}.Pose.Pose;
        Position = [odomStr.Position.X, odomStr.Position.Y, odomStr.Position.Z];
        Orientation = [odomStr.Orientation.X, odomStr.Orientation.Y, odomStr.Orientation.Z, odomStr.Orientation.W];
        PosOri = [Position, Orientation];
        odom(i_meas,:) = {PosOri};
    end
    odom.Properties.VariableNames = "PositionOrientation";
    timeOdom = seconds( bagOdom.MessageList.Time - bagOdom.StartTime );
    odom.Time = duration(timeOdom,'Format','mm:ss.SSS');
    save(strcat('SLAMData/odomROS_',sceneName), 'odom','-v7.3');

end