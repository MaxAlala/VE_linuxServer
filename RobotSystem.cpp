#include "RobotSystem.h"
#include "ServerController.h"
#include <mutex>
#include "FaceDetector.h"
#include "serialController.h"
#include "WebServer.h"
// #include <Eigen/Dense>
void shared_cout(std::string msg);
RobotSystem::~RobotSystem()
{
	std::cout << "~RobotSystem count" << inverseForwardKinematicsModel.use_count() << "\n";
}

RobotSystem::RobotSystem()
{
	inverseForwardKinematicsModel = std::make_shared<InverseForwardKinematicsModel>();
	std::cout << "RobotSystem count" << inverseForwardKinematicsModel.use_count() << "\n";

	motorController = std::make_shared<MotorController>(inverseForwardKinematicsModel);
	ws.reset(new WebServer());
	voiceController.reset(new VoiceController());
	serverController.reset(new ServerController(motorController, voiceController));

	pixelToMotorStepsConverter.reset(new PixelToMotorStepsConverter(640, 480, 561, 22, 12, 2.905, 1.57));

	currentState = States::ALIVE;
	currentRoboticSystem = CurrentRoboticSystem::TURRET;
	shouldTurnOffVision = true;

	if (!shouldTurnOffVision)
	{
		visionController.reset(new Eye(0, inverseForwardKinematicsModel, false, false));
	}
}

void RobotSystem::startFaceDetectionForOneSec()
{

	auto autohoming0 = [this]()
	{
		bool wasTrackerOfDetectedFaceStarted = false;
		bool shouldStopFaceDetectionBecauseNoFaceWasDetected = false;
		bool canSendGreetings = false;

		visionController->faceDetector.startUpdatingFaces();

		// searches for face for 2 sec
		// stop timer
		auto detectFaceFor2SecLmbda = [&shouldStopFaceDetectionBecauseNoFaceWasDetected, this]()
		{
			int counter = 0;
			while (!shouldStopFaceDetectionBecauseNoFaceWasDetected)
			{

				std::this_thread::sleep_for(std::chrono::milliseconds(100));

				++counter;
				if (visionController->isFaceDetected())
				{
					std::cout << "face was detected \n";
					counter = 0;
				}
				std::cout << "counter = " << counter << "\n";

				if (counter > 20)
					shouldStopFaceDetectionBecauseNoFaceWasDetected = true;
			}
		};

		std::thread detectFaceFor2SecThread(detectFaceFor2SecLmbda);
		detectFaceFor2SecThread.detach();

		auto greetings_timer_lmbda = [&canSendGreetings, this]()
		{
			while (!canSendGreetings)
			{
				canSendGreetings = true;
				std::this_thread::sleep_for(std::chrono::milliseconds(2000));
			}
		};

		std::thread greetings_timer_th(greetings_timer_lmbda);
		greetings_timer_th.detach();

		while (!shouldStopFaceDetectionBecauseNoFaceWasDetected)
		{

			// sends command to direct a robot`s tool to detected face
			std::cout << "FaceDetectionProcess \n";
			if (visionController->isFaceDetected())
			{
				DetectedFaceAndConfidence detectedFace = visionController->faceDetector.currentDetectedFace;

				int threshold = 1;
				if (detectedFace.faceConfidence > threshold)
				{
					// std::cout << " eye.get()->isFaceDetected()=" << eye.get()->isFaceDetected() << std::endl;
					std::cout << " face xy=" << detectedFace.faceCoordinate.x << " " << detectedFace.faceCoordinate.y << std::endl;
					std::cout << " detectedFaces.size()=" << visionController->faceDetector.detectedFaces.size() << std::endl;

					std::vector<int> angles{0, 0};
					pixelToMotorStepsConverter->calculateAnglesUsingLogic(detectedFace.faceCoordinate, angles[0], angles[1]);
					// stepperMotorController->setMovementCommand(angles);
					motorController->turretMoveAxesToSomeAngle(true, angles);
					// sends a speaking command
					// std::vector<char> speakingGreetingsCommand;
					// speakingGreetingsCommand.push_back('G');
					// speakingGreetingsCommand.push_back('R');

					voiceController->sayGreetings();
				}
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		visionController->faceDetector.stopUpdatingFaces();
	};

	std::thread thread_(autohoming0);
	thread_.join();
}

void RobotSystem::startLifeFunc()
{
	for (;;)
	{
		bool shouldPatrol = true;
		if (currentState == States::ALIVE)
		{

			// shared_cout(" patrol1 \n");

			cv::Point chosenPoint = visionController->run();
			//  std::cout << " patrol2 \n";
			if (chosenPoint == cv::Point(0, 0))
			{
				// std::cout << " patrol3 \n";
			}
			else
			{
				// std::cout << " patrol4 \n";
				std::cout << "TurretSystem::run().received point from EYE: " << chosenPoint.x << " " << chosenPoint.y << std::endl;
				int stepsForFirstMotor = 0;
				int stepsForSecondMotor = 0;
				int angles1 = 0;
				int angles2 = 0;
				//                pixelToMotorStepsConverter->calculateStepsByPixel(chosenPoint, stepsForFirstMotor, stepsForSecondMotor);
				//                pixelToMotorStepsConverter->calculateStepsUsingCalibratedValue(chosenPoint, stepsForFirstMotor, stepsForSecondMotor);
				// pixelToMotorStepsConverter->calculateStepsUsingInverseKinematics(chosenPoint, stepsForFirstMotor, stepsForSecondMotor);
				std::vector<int> angles{0, 0};
				pixelToMotorStepsConverter->calculateAnglesUsingLogic(chosenPoint, angles[0], angles[1]);

				motorController->turretMoveAxesToSomeAngle(true, angles);

				// stepperMotorController->setMovementCommand(angles);

				// stepperMotorController->moveAFewDegrees(angles1, angles2, true);
				// inverseForwardKinematicsModel->updateThetasUsingCustomAngles(angles);
				// std::array<int, NUMBER_OF_AXES> anglesToReach;
				//
				// for (int i = 0; i < NUMBER_OF_AXES; i++) {
				//     anglesToReach[i] = angles[i];
				// }

				// stepperMotorController->moveAFewSteps(stepsForFirstMotor, stepsForSecondMotor);
			}

			static int wasSerialListenerStarted = 0;

			// if (wasSerialListenerStarted == 0) {
			//     std::thread th(&SerialListener::doListening, serialListener.get());
			//     th.detach();
			//     ++wasSerialListenerStarted;
			// }

			// serialListener->doListening();

			int k = cv::waitKey(10);

			static int z = 0;
			static int p = 0;

			// v || V = starts homing
			if (k == 118 || k == 86)
			{
				// z++;
				shared_cout("TS::run(). starts a voice command.\n");

				// sends a speaking command
				// std::vector<char> speakingGreetingsCommand;
				// speakingGreetingsCommand.push_back('G');
				// speakingGreetingsCommand.push_back('R');

				// stepperMotorController->setSpeakingCommand(speakingGreetingsCommand);
				voiceController->sayGreetings();
			}

			//// d || D
			if (k == 68 || k == 100)
			{
				static bool isFaceDetectionRunning = false;

				isFaceDetectionRunning = !isFaceDetectionRunning;

				if (isFaceDetectionRunning)
				{
					auto autohoming0 = [this]()
					{
						visionController->faceDetector.startUpdatingFaces();
						while (isFaceDetectionRunning)
						{

							std::cout << "FaceDetectionProcess \n";
							if (visionController->isFaceDetected())
							{
								DetectedFaceAndConfidence detectedFace = visionController->faceDetector.currentDetectedFace;

								int threshold = 1;
								if (detectedFace.faceConfidence > threshold)
								{
									// std::cout << " eye.get()->isFaceDetected()=" << eye.get()->isFaceDetected() << std::endl;
									std::cout << " face xy=" << detectedFace.faceCoordinate.x << " " << detectedFace.faceCoordinate.y << std::endl;
									std::cout << " detectedFaces.size()=" << visionController->faceDetector.detectedFaces.size() << std::endl;

									std::vector<int> angles{0, 0};
									pixelToMotorStepsConverter->calculateAnglesUsingLogic(detectedFace.faceCoordinate, angles[0], angles[1]);
									motorController->turretMoveAxesToSomeAngle(true, angles);

									// stepperMotorController->setMovementCommand(angles);
								}
							}
							std::this_thread::sleep_for(std::chrono::milliseconds(400));
						}
						visionController->faceDetector.stopUpdatingFaces();
					};

					std::thread thread_(autohoming0);
					thread_.detach();
				}
			}

			// t T
			if (k == 84 || k == 116)
			{
			}

			//// f & F
			if (k == 70 || k == 102)
			{
				static Eigen::Vector3d pointToScan1 = {100, 0, 45};
				static Eigen::Vector3d pointToScan2 = {95, 30, 45};

				// inverseForwardKinematicsModel->doExtendedInverseKinematics(pointToScan);
				inverseForwardKinematicsModel->doInverseKinematics(inverseForwardKinematicsModel->getCalculationThetas(), pointToScan1);

				inverseForwardKinematicsModel->calculateRelativeMovementAngles();

				inverseForwardKinematicsModel->doInverseKinematics(inverseForwardKinematicsModel->getCalculationThetas(), pointToScan2);

				inverseForwardKinematicsModel->calculateRelativeMovementAngles();

				// updates thetas
				// inverseForwardKinematicsModel->updateThetasUsingAnglesFromInverseKinematics();

				// creates new DH. receives a new H2_0 with rotated 0 axis
				// for (int i = 0; i < inverseForwardKinematicsModel->getCalculationThetas().size(); i++) {
				//	std::cout << "calc thetas i = " << i << " " << inverseForwardKinematicsModel->getCalculationThetas().at(i) << std::endl;
				//}

				// inverseForwardKinematicsModel->calculateDH(inverseForwardKinematicsModel->getCalculationThetas());

				//////////////////////////////////////////////////////////////////
				// inverseForwardKinematicsModel->doForwardKinematics();
				// inverseForwardKinematicsModel->printActionAngles();
				// inverseForwardKinematicsModel->printStepAngles();
				// inverseForwardKinematicsModel->printThetas();
				// std::cout << eye->getCurrentLidarDistance() << std::endl;

				// stepperMotorController->setMovementCommand(-8000, -8000);
				// int maxAngle = 360;
				// int numberOfPoints = 20;
				// int radius = 100;
				// int angleInOneStep = maxAngle / numberOfPoints; // 20 degrees per step
				// int x = 0;
				// int y = 0;

				// for (int i = 0; i < numberOfPoints; ++i) {
				//     x = cos(inverseForwardKinematicsModel->fromGradToRad(i * angleInOneStep)) * radius;
				//     y = sin(inverseForwardKinematicsModel->fromGradToRad(i * angleInOneStep)) * radius;
				//     std::cout << x << " " << y << std::endl;
				// }
			}

			// H || h = starts homing
			if (k == 72 || k == 104)
			{
				// z++;
				shared_cout("TS::run(). sets homing command.\n");
				// int axis1HomingDirection = 1;
				// std::vector<double> thetas = inverseForwardKinematicsModel->getThetas();

				// if (!(thetas[1] < 87 && thetas[1] > -87)) {
				//     std::cerr << "Theta 1 is out of the range \n";
				//     return;
				// }

				// if ((thetas[0] >= 90 && thetas[0] <= 180) || (thetas[0] >= 181 && thetas[0] <=270)) {
				//     axis1HomingDirection = -1;
				// } else if ((thetas[0] < 90 && thetas[0] >= -90) || (thetas[0] >= 271 && thetas[0] <= 450)) {
				//     axis1HomingDirection = 1;
				// } else {
				//     std::cerr << "Theta 0 is out of the range \n";
				//     return;
				// }

				if (currentRoboticSystem == CurrentRoboticSystem::TURRET)
					motorController->startAutohoming();
			}

			// startPatrolTerritory();
			//  P || p = starts patrolling
			if (k == 80 || k == 112)
			{
				static std::vector<int> angles;
				std::cout << "p was pressed \n";
				// static bool shouldThreadRun = false;
				if (p == 0)
				{
					// shouldThreadRun = true;
					p++;
					std::cout << "p is 1 now \n";
				}
				else if (p == 1)
				{
					std::cout << "p is 0 now \n";
					p = 0;
				}
				// else
				//{
				//	p = 0;
				//	shouldThreadRun = false;
				//	break;
				// }

				// inverseForwardKinematicsModel->doExtendedInverseKinematics(Eigen::Vector3d(100,100,100));
				// inverseForwardKinematicsModel->doExtendedInverseKinematics(Eigen::Vector3d(-100,100,100));
				// inverseForwardKinematicsModel->doInverseKinematics(Eigen::Vector3d(50, -50, 100));

				static bool shouldSkipOneLoop = false;
				// std::cout << motorController->isTheAutohomingRunning << "=isTheAutohomingRunning=\n";

				if (!motorController->isTheAutohomingRunning && !motorController->isThereMovement())
				{
					std::cout << "t2 \n";
					auto autohoming0 = [this]()
					{
						int ic = 0;
						while (p == 1)
						{
							// std::cout << "w123 \n";
							// std::cout << "t3 \n";
							if (motorController->isThereMovement())
								continue;

							if (!shouldSkipOneLoop)
							{
								if (ic != 0)
									// if calculated unequals to current angles = then move to calculated. should move a turret to calculated angles
									if ((int)(inverseForwardKinematicsModel->inverseKinematicsCalculatedAngles.at(0) != (int)inverseForwardKinematicsModel->getThetasWhereStartIsZero().at(0) ||
											  (int)inverseForwardKinematicsModel->inverseKinematicsCalculatedAngles.at(1) != (int)inverseForwardKinematicsModel->getThetasWhereStartIsZero().at(1)))
									{
										// stepperMotorController->setMovementCommand(angles, false);
										motorController->turretMoveAxesToSomeAngle(false, angles);
										// static int dispcounter = 0;
										// if (dispcounter == 9999999) {
										std::cout << "invKin= " << (int)(inverseForwardKinematicsModel->inverseKinematicsCalculatedAngles.at(0)) << " " << (int)(inverseForwardKinematicsModel->inverseKinematicsCalculatedAngles.at(1)) << "\n";
										//	<< std::endl;
										std::cout << "currentZeroAngls= " << (int)inverseForwardKinematicsModel->getThetasWhereStartIsZero().at(0) << " " << (int)inverseForwardKinematicsModel->getThetasWhereStartIsZero().at(1) << "\n";
										//	<< std::endl;
										// dispcounter = 0;

										//}
										std::this_thread::sleep_for(std::chrono::milliseconds(1000));
										//++dispcounter;
										continue;
									}
							}
							else
							{
								shouldSkipOneLoop = false;
							}

							// int wallSize = 1000;
							// int wallSizeY = 10000;
							int wallSizeZ = 45; // height of the point between lidar and camera

							int wallSizeDivider = 10;
							// std::cout << "t4 \n";
							//  GOES THROUGH LINE
							// if (i < 20) {

							//    // from y changes from -100 to 100
							//    Eigen::Vector3d pointToScan;
							//    pointToScan = { (double)wallSize, (double)-wallSizeY + ((double)wallSizeY / wallSizeDivider) * i, (double)wallSizeZ };
							//    std::string str = "Current coordinate = " + std::to_string(pointToScan.x()) + " " + std::to_string(pointToScan.y()) + " " + std::to_string(pointToScan.z());
							//    shared_cout(str);
							//    inverseForwardKinematicsModel->doExtendedInverseKinematics(pointToScan);

							//    Eigen::Vector2d vec2d = inverseForwardKinematicsModel->calculateStepAngles();
							//    stepperMotorController->setMovementCommand(vec2d.x() * stepperMotorController->stepsPer1Angle1, vec2d.y() * stepperMotorController->stepsPer1Angle2);
							//    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

							//    i++;
							//}

							int maxAngle = 360;
							int numberOfPoints = 20;
							int radius = 100;
							int angleInOneStep = maxAngle / numberOfPoints; // 20 degrees per step
							int x = 0;
							int y = 0;
							static Eigen::Vector3d pointToScan;

							// obstacle coordinate in 0 coordinate system
							Eigen::Vector3d obstacleCoordinate = visionController->calculateObstacleCoordinate();

							std::cout << "i is " << ic << "\n";
							std::cout << "pointToScan is " << pointToScan.x() << " " << pointToScan.y() << " " << pointToScan.z() << "\n";
							std::cout << "obstacleCoordinate is " << obstacleCoordinate.x() << " " << obstacleCoordinate.y() << " " << obstacleCoordinate.z() << "\n";
							visionController->drawCircleAtMap2d(cv::Point(pointToScan.x(), pointToScan.y()), cv::Scalar(0, 0, 255));
							visionController->drawCircleAtMap2d(cv::Point(obstacleCoordinate.x(), obstacleCoordinate.y()), cv::Scalar(255, 0, 0));

							x = cos(inverseForwardKinematicsModel->fromGradToRad(ic * angleInOneStep)) * radius;
							y = sin(inverseForwardKinematicsModel->fromGradToRad(ic * angleInOneStep)) * radius;
							std::cout << x << " " << y << std::endl;
							pointToScan = {(double)x, (double)y, (double)wallSizeZ};

							// inverseForwardKinematicsModel->doExtendedInverseKinematics(pointToScan);
							// inverseForwardKinematicsModel->getCalculationThetas() = inverseForwardKinematicsModel->getCurrentThetas();
							inverseForwardKinematicsModel->doInverseKinematics(inverseForwardKinematicsModel->getCalculationThetas(), pointToScan);

							// updates thetas
							// inverseForwardKinematicsModel->updateThetasUsingAnglesFromInverseKinematics(inverseForwardKinematicsModel->getCalculationThetas());

							// now eye directed to point
							// inverseForwardKinematicsModel->calculateDH(inverseForwardKinematicsModel->getCalculationThetas());

							if (inverseForwardKinematicsModel->inverseKinematicsCalculatedAngles.at(0) > 140 ||
								inverseForwardKinematicsModel->inverseKinematicsCalculatedAngles.at(0) < -140)
							{
								shouldSkipOneLoop = true;
								++ic;
								continue;
							}

							if (inverseForwardKinematicsModel->inverseKinematicsCalculatedAngles.at(1) > 140 ||
								inverseForwardKinematicsModel->inverseKinematicsCalculatedAngles.at(1) < -140)
							{
								shouldSkipOneLoop = true;
								++ic;
								continue;
							}

							// angles = inverseForwardKinematicsModel->calculateRelativeMovementAngles();
							std::cout << " ic= " << ic << std::endl;

							angles.clear();
							// angles = inverseForwardKinematicsModel->getInverseKinematicsAngles();
							angles.push_back((int)round(inverseForwardKinematicsModel->getInverseKinematicsAngles()[0]));
							angles.push_back((int)round(inverseForwardKinematicsModel->getInverseKinematicsAngles()[1]));

							angles[1] = -angles[1];
							std::cout << " patrolAngles togo=" << std::to_string(angles[0]) << " " << std::to_string(angles[1]) << "\n";
							// std::vector<AxisBorders> axisBorders;

							////default_thetas{180, 0, 90, 0},
							////axisBorders.push_back(AxisBorders(-60, 300, 120));
							////axisBorders.push_back(AxisBorders(-13, 167, 77));

							// axisBorders.push_back(AxisBorders(-40, 280, 120));
							// axisBorders.push_back(AxisBorders(0, 167, 60));

							// if ((angles[0]+axisBorders[0].home) > axisBorders[0].right || (angles[0] + axisBorders[0].home) < axisBorders[0].left
							//	|| (angles[1] + axisBorders[1].home) > axisBorders[1].right || (angles[1] + axisBorders[1].home) < axisBorders[1].left) {

							// shouldSkipOneLoop = true;
							// std::cout << "out of border. Face scanning/ angles =" << angles[0] << " " << angles[1] << std::endl;
							// continue;
							//}

							++ic;
							std::this_thread::sleep_for(std::chrono::milliseconds(700));

							// stepperMotorController->setMovementCommand(angles, false);
							motorController->turretMoveAxesToSomeAngle(false, angles);
							//////////////////////////////////////
							// startFaceDetectionForOneSec();

							//////////////////////////////////////
							std::this_thread::sleep_for(std::chrono::milliseconds(200));
							if (ic == (numberOfPoints + 1))
							{
								ic = 0;
								// p = 0;

								// break;
							}
						}
					};
					// std::cout << "t4 \n";
					std::thread startP(autohoming0);
					startP.detach();
				}
			}

			// // K || k = starts scanning
			// if (k == 75 || k == 107) {
			// 	z++;
			// 	//inverseForwardKinematicsModel->doExtendedInverseKinematics(Eigen::Vector3d(100,100,100));
			// 	//inverseForwardKinematicsModel->doExtendedInverseKinematics(Eigen::Vector3d(-100,100,100));
			// 	//inverseForwardKinematicsModel->doInverseKinematics(Eigen::Vector3d(50, -50, 100));
			// }
			// static int i = 0;

			// if (z == 1)
			// 	if (StepperMotorController::isMovementRunning == 0 && StepperMotorController::isHomingRunning == 0 &&
			// 		(inverseForwardKinematicsModel->inverseKinematicsCalculatedAngles.at(0) == inverseForwardKinematicsModel->getCurrentThetas().at(0) &&
			// 		 inverseForwardKinematicsModel->inverseKinematicsCalculatedAngles.at(1) == inverseForwardKinematicsModel->getCurrentThetas().at(1))
			// 		)
			// 	{

			// 		int wallSize = 1000;
			// 		int wallSizeY = 10000;
			// 		int wallSizeZ = 45; // height of the point between lidar and camera

			// 		int wallSizeDivider = 10;

			// 		// GOES THROUGH LINE
			// 		//if (i < 20) {

			// 		//    // from y changes from -100 to 100
			// 		//    Eigen::Vector3d pointToScan;
			// 		//    pointToScan = { (double)wallSize, (double)-wallSizeY + ((double)wallSizeY / wallSizeDivider) * i, (double)wallSizeZ };
			// 		//    std::string str = "Current coordinate = " + std::to_string(pointToScan.x()) + " " + std::to_string(pointToScan.y()) + " " + std::to_string(pointToScan.z());
			// 		//    shared_cout(str);
			// 		//    inverseForwardKinematicsModel->doExtendedInverseKinematics(pointToScan);

			// 		//    Eigen::Vector2d vec2d = inverseForwardKinematicsModel->calculateStepAngles();
			// 		//    stepperMotorController->setMovementCommand(vec2d.x() * stepperMotorController->stepsPer1Angle1, vec2d.y() * stepperMotorController->stepsPer1Angle2);
			// 		//    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

			// 		//    i++;
			// 		//}

			// 		int maxAngle = 360;
			// 		int numberOfPoints = 120;
			// 		int radius = 100;
			// 		int angleInOneStep = maxAngle / numberOfPoints; // 20 degrees per step
			// 		int x = 0;
			// 		int y = 0;
			// 		static Eigen::Vector3d pointToScan;

			// 		if (i > 0) {
			// 			Eigen::Vector3d obstacleCoordinate = eye->calculateObstacleCoordinate();

			// 			std::cout << "i is " << i << "\n";
			// 			std::cout << "pointToScan is " << pointToScan.x() << " " << pointToScan.y() << " " << pointToScan.z() << "\n";
			// 			std::cout << "obstacleCoordinate is " << obstacleCoordinate.x() << " " << obstacleCoordinate.y() << " " << obstacleCoordinate.z() << "\n";
			// 			eye->drawCircleAtMap2d(cv::Point(pointToScan.x(), pointToScan.y()), cv::Scalar(0, 0, 255));
			// 			eye->drawCircleAtMap2d(cv::Point(obstacleCoordinate.x(), obstacleCoordinate.y()), cv::Scalar(255, 0, 0));
			// 		}

			// 		x = cos(inverseForwardKinematicsModel->fromGradToRad(i * angleInOneStep)) * radius;
			// 		y = sin(inverseForwardKinematicsModel->fromGradToRad(i * angleInOneStep)) * radius;
			// 		std::cout << x << " " << y << std::endl;
			// 		pointToScan = { (double)x, (double)y, (double)wallSizeZ };

			// 		//inverseForwardKinematicsModel->doExtendedInverseKinematics(pointToScan);
			// 		inverseForwardKinematicsModel->doInverseKinematics(inverseForwardKinematicsModel->getCalculationThetas(), pointToScan);

			// 		// updates thetas
			// 		inverseForwardKinematicsModel->updateThetasUsingAnglesFromInverseKinematics(inverseForwardKinematicsModel->getCalculationThetas());

			// 		// creates new DH. receives a new H2_0 with rotated 0 axis
			// 		//inverseForwardKinematicsModel->calculateDH(inverseForwardKinematicsModel->getCalculationThetas());

			// 		std::vector<int> vec2d = inverseForwardKinematicsModel->calculateRelativeMovementAngles();

			// 		stepperMotorController->setMovementCommand(vec2d);
			// 		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

			// 		++i;

			// 		if (i == numberOfPoints) {
			// 			i = 0;
			// 			z = 0;
			// 		}
			// 	}
		}
		else
		{
		}
	}
}

void RobotSystem::startServer()
{
	std::thread serverThread(&ServerController::startServer, serverController.get());
	serverThread.detach();
}

void RobotSystem::startLife()
{
	std::thread lifeThread(&RobotSystem::startLifeFunc, this);
	lifeThread.join();
}

void RobotSystem::startLidarDistanceDetectionProc()
{
	std::cout << currentLidarDistance << "startLidarDistanceDetectionProc111 \n";
	int TFMINI_DATA_Len = 9;
	int TFMINT_DATA_HEAD = 0x59;
	uint16_t cordist = 0;
	uint8_t chk_cal = 0;
	uint8_t ar[9];
	for (int i = 0; i < 9; i++)
		ar[i] = 0;
	int counter = 0;
	try
	{

		SimpleSerial serial("/dev/ttyUSB0", 115200);
		// std::cout << currentLidarDistance << "currentLidarDistance 2\n";
		// serial.writeString("Hello world\n");

		while (true)
		{
			// std::cout << currentLidarDistance << "currentLidarDistance \n";
			char readChar = serial.readChar();
			// std::cout << readChar << std::endl;

			if (readChar == 'Y' && counter == 0)
			{
				ar[counter] = readChar;
				++counter;
				continue;
			}

			if (readChar == 'Y' && counter == 1)
			{
				ar[counter] = readChar;
				++counter;
				continue;
			}

			if (counter >= 2)
			{
				ar[counter] = readChar;
				++counter;
			}

			if (counter >= 4)
			{
				cordist = ar[2] | (ar[3] << 8);
				// std::cout << "distance = " << cordist << "\n";
				if (cordist > 0 && cordist < 12000)
				{
					currentLidarDistance = cordist;
					// std::cout << currentLidarDistance << "currentLidarDistance \n";
					if (!shouldTurnOffVision)
						visionController->setCurrentLidarDistance(std::to_string(currentLidarDistance));
				}
				cordist = 0;
			}

			if (counter == 8)
			{
				// std::this_thread::sleep_for(std::chrono::milliseconds(200));
				counter = 0;
			}

			// std::cout << currentLidarDistance << "currentLidarDistance 3\n";
		}
	}
	catch (boost::system::system_error &e)
	{
		cout << "Error: " << e.what() << endl;
		return;
	}
}

void RobotSystem::startUpdateTimeOfLifeEveryMinute()
{
	while (1)
	{
		// do something
		std::this_thread::sleep_for(60s); // this function holds the foo execution for 60 sec

		// INSERT INTO SolarBeamStartTime VALUES (1, '2016-06-22 19:10:25-07');
		// std::string sqlUpdateStartTime = "INSERT INTO SolarBeamStartTime VALUES (1, "<< "\'" << 2016-06-22 19:10:25-07 <<  "\');";
		std::cout << "time was updated \n";
		db->execSqlAsync(
			"UPDATE SolarBeam set timeoflifeinminutes = timeoflifeinminutes + 1 WHERE id = 1",
			[](const drogon::orm::Result &result)
			{
				std::cout << result.size() << " rows selected!" << std::endl;
				int i = 0;
				for (auto row : result)
				{
					std::cout << i++ << ": timeoflifeinminutes is " << row["timeoflifeinminutes"].as<std::string>() << std::endl;
				}
			},
			[](const drogon::orm::DrogonDbException &e)
			{
				std::cerr << "error:" << e.base().what() << std::endl;
			});
		// should add an exit condition or this function will non stop
	}
}

void RobotSystem::startUpdateTimeOfLifeEveryMinuteThread()
{
	std::thread thread_(&RobotSystem::startUpdateTimeOfLifeEveryMinute, this);
	thread_.detach();
}

void RobotSystem::startLidarDistanceDetection()
{
	std::thread thread_(&RobotSystem::startLidarDistanceDetectionProc, this);
	thread_.detach();
}

void RobotSystem::startWebServer()
{
	std::thread thread_(&WebServer::startWebServer, ws.get());
	thread_.detach();
}

void RobotSystem::sendStartingTime()
{
	// auto db = drogon::app().getDbClient();

	// INSERT INTO SolarBeamStartTime VALUES (1, '2016-06-22 19:10:25-07');
	// std::string sqlUpdateStartTime = "INSERT INTO SolarBeamStartTime VALUES (1, "<< "\'" << 2016-06-22 19:10:25-07 <<  "\');";
	// std::cout << "time was updated \n";
	db->execSqlAsync(
		"select current_timestamp(0);",
		[this](const drogon::orm::Result &result)
		{
			std::cout << result.size() << " rows selected!" << std::endl;
			int i = 0;
			for (auto row : result)
			{
				currentStartTimeUTC = row["current_timestamp"].as<std::string>();
				std::cout << i++ << ": currentStartTimeUTC is " << currentStartTimeUTC << std::endl;
			}
			// std::cout << " currentStartTimeUTC= " << currentStartTimeUTC << "\n";
			std::string psqlInsertRequest = "INSERT INTO SolarBeamStartTime VALUES (1, \'" + currentStartTimeUTC + "\');";
			// std::cout << "psqlInsertRequest=" << psqlInsertRequest << std::endl;
			db->execSqlAsync(
				psqlInsertRequest,
				[](const drogon::orm::Result &result)
				{
					std::cout << "current time was sent to DB. \n";
				},
				[](const drogon::orm::DrogonDbException &e)
				{
					std::cerr << "error:" << e.base().what() << std::endl;
				});
		},
		[](const drogon::orm::DrogonDbException &e)
		{
			std::cerr << "error:" << e.base().what() << std::endl;
		});
}

void RobotSystem::startRobotSystem()
{

	// std::unique_ptr<MotorController> motorController;
	// std::unique_ptr<VoiceController> voiceController;
	// std::unique_ptr<ServerController> serverController;
	shared_cout(" STart1.11 \n");
	startServer();
	shared_cout(" STart1.111 \n");
	startLidarDistanceDetection();
	shared_cout(" STart1.1111 \n");
	startWebServer();
	std::this_thread::sleep_for(200ms);
	db = drogon::app().getDbClient();
	startUpdateTimeOfLifeEveryMinuteThread();
	sendStartingTime();
	// start
	if (!shouldTurnOffVision)
		startLifeFunc();
	shared_cout(" STart1.11111 \n");
}
