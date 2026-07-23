void EtRobocon2026::start()
{
  Logger::info("Hello KATLAB");

  RealNetworkSystem real;
  SocketClient client(real);

  Robot robot(client);

  robot.getCameraSocketClientInstance().connectToServer();


  Pid::PidGain camPid{0.002, 0.0005, 0.001};
  Pid::PidGain rightPid{0.00535, 0.00115, 0.00};
  Pid::PidGain leftPid{0.00578, 0.0008535, 0.00};


  CameraServer::ColorRegionDetectorRequest request;
  request.requireLargestColorIndex = true;
  request.hsvRangeCount = 1;
  request.hsvRanges[0].lower = {0,0,0,0};
  request.hsvRanges[0].upper = {180,255,30,0};
  request.roi = {0,0,1920,1080};
  request.resolution = {1920,1080};


  CameraTracking cameraTracking(
      robot,
      std::make_unique<DistanceCondition>(robot,1500.0),
      300.0,
      960,
      camPid,
      rightPid,
      leftPid,
      request
  );

  cameraTracking.run();
}