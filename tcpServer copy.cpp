
#include "tcpServer.h"
#include "MotorController.h"
#include <wiringPi.h>
//    std::string message = "C M " + std::to_string(steps1) + " " + std::to_string(steps2) + " " + std::to_string(commandID) + "\n";
//    std::string message = "C H " + std::to_string(commandID) + " " + std::to_string(Axis1Direction) + "\n";
//

void chatRoom::enter(std::shared_ptr<participant> participant, const std::string &nickname)
{
	participants_.insert(participant);
	name_table_[participant] = nickname;
	std::for_each(recent_msgs_.begin(), recent_msgs_.end(),
				  boost::bind(&participant::onMessage, participant, _1));
}

void chatRoom::leave(std::shared_ptr<participant> participant)
{
	participants_.erase(participant);
	name_table_.erase(participant);
}

void chatRoom::broadcast(std::array<char, MAX_IP_PACK_SIZE> &msg, std::shared_ptr<participant> participant)
{
	// std::string timestamp = getTimestamp();
	// std::string nickname = getNickname(participant);
	// std::array<char, MAX_IP_PACK_SIZE> formatted_msg;

	// // boundary correctness is guarded by protocol.hpp
	// strcpy(formatted_msg.data(), timestamp.c_str());
	// strcat(formatted_msg.data(), nickname.c_str());
	// strcat(formatted_msg.data(), msg.data());

	// recent_msgs_.push_back(formatted_msg);
	// while (recent_msgs_.size() > max_recent_msgs)
	// {
	// 	recent_msgs_.pop_front();
	// }

	// std::for_each(participants_.begin(), participants_.end(),
	// 			  boost::bind(&participant::onMessage, _1, std::ref(formatted_msg)));
}

std::string chatRoom::getNickname(std::shared_ptr<participant> participant)
{
	return name_table_[participant];
}

personInRoom::personInRoom(boost::asio::io_service &io_service,
						   boost::asio::io_service::strand &strand, chatRoom &room,
						   MotorController *motorController)
	: socket_(io_service), strand_(strand), room_(room), motorController_(motorController)
{
}

void personInRoom::start()
{
	boost::asio::async_read(socket_,
							boost::asio::buffer(nickname_, nickname_.size()),
							strand_.wrap(boost::bind(&personInRoom::nicknameHandler, shared_from_this(), _1)));
}

void personInRoom::onMessage(std::array<char, MAX_IP_PACK_SIZE> &msg)
{
	bool write_in_progress = !write_msgs_.empty();
	write_msgs_.push_back(msg);
	if (!write_in_progress)
	{
		boost::asio::async_write(socket_,
								 boost::asio::buffer(write_msgs_.front(), write_msgs_.front().size()),
								 strand_.wrap(boost::bind(&personInRoom::writeHandler, shared_from_this(), _1)));
	}
}

void personInRoom::nicknameHandler(const boost::system::error_code &error)
{
	// if (strlen(nickname_.data()) <= MAX_NICKNAME)
	// {
	// 	strcat(nickname_.data(), ": ");
	// }
	// else
	// {
	// 	// cut off nickname if too long
	// 	// nickname_[MAX_NICKNAME - 2] = ':';
	// 	// nickname_[MAX_NICKNAME - 1] = ' ';
	// }

	room_.enter(shared_from_this(), std::string(nickname_.data()));

	boost::asio::async_read(socket_,
							boost::asio::buffer(read_msg_, read_msg_.size()),
							strand_.wrap(boost::bind(&personInRoom::readHandler, shared_from_this(), _1)));
}

stringParser::CommandTypes stringParser::detectCommandType(std::array<char, MAX_IP_PACK_SIZE> &read_msg_)
{
	if (read_msg_.data()[0] == 'C')
	{
		if (read_msg_.data()[2] == 'A')
		{
			return CommandTypes::AUTOHOMING;
		}
		else if (read_msg_.data()[2] == 'M' && read_msg_.data()[3] == 'I')
		{
			return CommandTypes::REQUEST_ID;
		}
		else if (read_msg_.data()[2] == 'M')
		{
			return CommandTypes::MOVE;
		}
	}
	return CommandTypes::NOT_DETECTED_COMMAND;
}

//   Axis Angle ID
// C M 0 10 2
bool stringParser::parseCommandMove(std::array<char, MAX_IP_PACK_SIZE> &read_msg_, std::array<int, NUMBER_OF_ANGLES> &angles, int &axis, int &id)
{
	std::string s = read_msg_.data();
	std::string delimiter = " ";

	s = s.substr(4, s.length());
	s.append(" ");
	memset(angles.data(), -1, NUMBER_OF_ANGLES);

	size_t pos = 0;
	std::string currentSubString;
	// int anglesAndID[NUMBER_OF_ANGLES + 1];
	int numberCounter = 0;
	int axis_ = 0;
	int angle_ = 0;
	int id_ = 0;
	int numberOfParams = 3;
	while ((pos = s.find(delimiter)) != std::string::npos)
	{

		currentSubString = s.substr(0, pos);
		std::cout << currentSubString << "\n";
		int currentNumber = 0;

		try
		{
			currentNumber = std::stoi(currentSubString);
		}
		catch (std::invalid_argument &e)
		{
			std::cout << "stoi.invalid_arg\n";
			// memset(angles.data(), -1, NUMBER_OF_ANGLES);
			return -1;
		}
		catch (std::out_of_range &e)
		{
			std::cout << "stoi.out_of_range\n";
			// memset(angles.data(), -1, NUMBER_OF_ANGLES);
			return -1;
		}

		// for ids
		if (numberCounter == 0)
			axis_ = currentNumber;
		else if (numberCounter == 1)
			angle_ = currentNumber;
		else if (numberCounter == 2)
			id_ = currentNumber;

		// else if (numberCounter < NUMBER_OF_ANGLES * 2)
		//	axisMovementID[numberCounter % NUMBER_OF_ANGLES] = currentNumber;

		++numberCounter;

		s.erase(0, pos + delimiter.length());
	}

	std::cout << "parseCommandMove ax ang id=" << axis_ << " " << angle_ << " " << id_ << std::endl;

	if (numberCounter != numberOfParams)
	{
		std::cout << "ERROR.parseCommandMove.numberCounter != 3, but =" << numberCounter << std::endl;
		return false;
	}

	id = id_;
	axis = axis_;
	angles[axis_] = angle_;

	return true;
}

// C MI 0
bool stringParser::parseCommandMoveID(std::array<char, MAX_IP_PACK_SIZE> &read_msg_, int &axis)
{
	std::string s = read_msg_.data();


	// 0 10
	s = s.substr(5, s.length());
	s.append(" ");
	// memset(angles.data(), -1, NUMBER_OF_ANGLES);

	size_t pos = 0;
	std::string currentSubString;
	// int id_ = 0;
	int axis_ = 0;
	int numberCounter = 0;
	std::string delimiter = " ";

	while ((pos = s.find(delimiter)) != std::string::npos)
	{

		// 0
		currentSubString = s.substr(0, pos);
		std::cout << currentSubString << "\n";
		int currentNumber = 0;

		try
		{
			currentNumber = std::stoi(currentSubString);
		}
		catch (std::invalid_argument &e)
		{
			std::cout << "stoi.invalid_arg\n";
			// memset(angles.data(), -1, NUMBER_OF_ANGLES);
			return -1;
		}
		catch (std::out_of_range &e)
		{
			std::cout << "stoi.out_of_range\n";
			// memset(angles.data(), -1, NUMBER_OF_ANGLES);
			return -1;
		}

		// for ids
		if (numberCounter == 0)
			axis_ = currentNumber;

		// if (numberCounter == 1)
		// 	id_ = currentNumber;

		// else if (numberCounter < NUMBER_OF_ANGLES * 2)
		//	axisMovementID[numberCounter % NUMBER_OF_ANGLES] = currentNumber;

		++numberCounter;

		s.erase(0, pos + delimiter.length());
	}

	if (numberCounter != 1)
	{
		std::cout << "ERROR.parseCommandMoveID.numberCounter != 1. \n";
		return false;
	}

	std::cout << "parseCommandMoveID ax = " << axis_ << std::endl;
	axis = axis_;
	return true;
}

void personInRoom::readHandler(const boost::system::error_code &error)
{
	if (!error)
	{

		// room_.broadcast(read_msg_, shared_from_this());

		stringParser parser;

		stringParser::CommandTypes type = parser.detectCommandType(read_msg_);

		// new
		// std::cout << "current type = " << int(type) << std::endl;

		static int counter = 0;

		int axis = -1;
		int id = -1;
		std::string msg;

		if (type == stringParser::CommandTypes::REQUEST_ID)
		{
			// C MI axis
			// is there movement + received request for ID = stop this request. stop condition1.
			if (parser.parseCommandMoveID(read_msg_, axis))
			{
				if (motorController_->isThereMovementToSpecificAngle[axis])
				{
					// msg = "C MS " + std::to_string(axis) + "\n";
					// memset(motorController_->textMsg.data(), '\0', MAX_IP_PACK_SIZE);
					// memcpy(motorController_->textMsg.data(), msg.data(), msg.size());
					// std::this_thread::sleep_for(std::chrono::milliseconds(200));
					// onMessage(motorController_->textMsg);
				}
				else
				{
					// else if no movement = sends current ID
					// C MI axis id

					std::cout << "sends ID = " << motorController_->currentMovementID[axis] << std::endl;
					msg = "C MI " + std::to_string(axis) + " " + std::to_string(motorController_->currentMovementID[axis]) + "\n";
					std::cout << "sended str= " << msg << std::endl;

					memset(motorController_->textMsg.data(), '\0', MAX_IP_PACK_SIZE);
					memcpy(motorController_->textMsg.data(), msg.data(), msg.size());
					// std::this_thread::sleep_for(std::chrono::milliseconds(200));
					onMessage(motorController_->textMsg);
				}
			}
		}

		// C M axis angle id
		else if (type == stringParser::CommandTypes::MOVE)
		{
			std::string msg;
			if (parser.parseCommandMove(read_msg_, motorController_->receivedAngle, axis, id))
			{

				// starts movement if id is correct + no movement
				if (id == motorController_->currentMovementID[axis] && !motorController_->isThereMovementToSpecificAngle[axis])
				{
					motorController_-> isThereMovementToSpecificAngle[axis] = true;		
					std::cout << "starts movement, axis angle id " << axis << " " << motorController_->receivedAngle[axis]<< " " << id << std::endl;

					motorController_->savedReceivedAngle[axis] = motorController_->receivedAngle[axis];
					if (motorController_->shouldInverseSignOfReceivedAngleFromClient[axis])
						motorController_->moveAxisToSomeAngleI(motorController_->currentAngle[axis] - motorController_->savedReceivedAngle[axis], axis);
					else
						motorController_->moveAxisToSomeAngleI(motorController_->currentAngle[axis] + motorController_->savedReceivedAngle[axis], axis);

					// stop condition. id doesnt equal to current id
				}
				else if (id != motorController_->currentMovementID[axis])
				{

					msg = "C MS " + std::to_string(axis) + "\n";
					std::cout << " sends stop =" << msg; 
					memset(motorController_->textMsg.data(), '\0', MAX_IP_PACK_SIZE);
					memcpy(motorController_->textMsg.data(), msg.data(), msg.size());
					// std::this_thread::sleep_for(std::chrono::milliseconds(200));
					onMessage(motorController_->textMsg);
				}

				// std::cout << "received angles = \n";
				// for (auto i : motorController_->receivedAngle)
				// {
				// 	std::cout << i << ' ';
				// }
				// std::cout << "savedReceivedAngle= " << motorController_->savedReceivedAngle[0] << " " << motorController_->savedReceivedAngle[1] << std::endl;
				// std::cout << "savedReceivedAngle+currentAngle= " << motorController_->currentAngle[0] + motorController_->savedReceivedAngle[0]
				// 		  << " " << motorController_->currentAngle[1] + motorController_->savedReceivedAngle[1] << std::endl;

				// // std::cout << "\n";
				// for (int i = 0; i < NUMBER_OF_ANGLES; i++)
				// {
				// 	// if (counter == 10)
				// 	// {
				// 	// 	motorController_->isThereAxisMovement[0] = false;
				// 	// 	motorController_->isThereAxisMovement[1] = false;
				// 	// 	counter = 0;
				// 	// }

				// 	// std::cout << "curangle i = " << i << " = " << std::to_string(motorController_->currentAngle[i]) << std::endl;
				// 	if (motorController_->receivedAngle[i] != 999)
				// 	{

				// 		if (!motorController_->isThereMovementToSpecificAngle[i])
				// 		{
				// 			// 1st stop condition = reached the desired position
				// 			// no movement+position is reached = stops the sender of the angle
				// 			if (motorController_->savedReceivedAngle[i] == motorController_->receivedAngle[i])
				// 			{
				// 				msg = "C M" + std::to_string(i) + "\n";
				// 				memset(motorController_->textMsg.data(), '\0', MAX_IP_PACK_SIZE);
				// 				memcpy(motorController_->textMsg.data(), msg.data(), msg.size());
				// 				std::this_thread::sleep_for(std::chrono::milliseconds(200));
				// 				onMessage(motorController_->textMsg);

				// 				// to prevent blocking the same angle
				// 				motorController_->savedReceivedAngle[i] = 999;
				// 				// std::cout << "1stFinishCondition=" << msg << std::endl;
				// 				continue;
				// 			}

				// 			// no movement + new position received = start movement to new position

				// 			motorController_->savedReceivedAngle[i] = motorController_->receivedAngle[i];
				// 			if (motorController_->shouldInverseSignOfReceivedAngleFromClient[i])
				// 				motorController_->moveAxisToSomeAngleI(motorController_->currentAngle[i] - motorController_->savedReceivedAngle[i], i);
				// 			else
				// 				motorController_->moveAxisToSomeAngleI(motorController_->currentAngle[i] + motorController_->savedReceivedAngle[i], i);

				// 			// 2nd stop condition = there is already movement to some location = terminates all senders except
				// 			// original
				// 		}
				// 		else if (motorController_->savedReceivedAngle[i] != motorController_->receivedAngle[i])
				// 		{

				// 			msg = "C M" + std::to_string(i) + "\n";
				// 			memset(motorController_->textMsg.data(), '\0', MAX_IP_PACK_SIZE);
				// 			memcpy(motorController_->textMsg.data(), msg.data(), msg.size());
				// 			// std::cout << "2stFinishCondition=" << msg << std::endl;
				// 			std::this_thread::sleep_for(std::chrono::milliseconds(200));
				// 			onMessage(motorController_->textMsg);
				// 		}
				// 	}
				// }

				// counter++;

				// std::cout << "curangle i = " << i << " = " << std::to_string(motorController_->currentAngle[i]) << std::endl;
				// sends status
				// std::string msg = "Is there movement = " + std::to_string(motorController_->isThereMovementToSpecificAngle[0]) + " " + std::to_string(motorController_->isThereMovementToSpecificAngle[1]) +
				// 				  ",\nCurrent angles =" + std::to_string(motorController_->currentAngle[0]) + " " + std::to_string(motorController_->currentAngle[1]) + "\n";
				// // std::cout << msg << std::endl;
				// std::this_thread::sleep_for(std::chrono::milliseconds(200));

				// memset(motorCon troller_->textMsg.data(), '\0', MAX_IP_PACK_SIZE);
				// memcpy(motorController_->textMsg.data(), msg.data(), msg.size());
				// onMessage(motorController_->textMsg);
				// std::cout << "IDs = \n";
				// for (auto i : motorController_->axisMovementID) {
				//	std::cout << i << ' '; // will print: "a b c"
				// }
			}
		}
		else if (type == stringParser::CommandTypes::AUTOHOMING)
		{
			std::string msg;

			// stop condition
			if (motorController_->checkIfRobotIsAtHome())
			{
				msg = "C A";
				motorController_->isTheAutohomingStarted = false;
				memset(motorController_->textMsg.data(), '\0', MAX_IP_PACK_SIZE);
				memcpy(motorController_->textMsg.data(), msg.data(), msg.size());
				std::cout << "autohoming is finished=" << msg << std::endl;
				onMessage(motorController_->textMsg);
			}
			else
			{
				if (!motorController_->isTheAutohomingStarted)
				{
					motorController_->isTheAutohomingStarted = true;
					motorController_->startAutohoming();
				}
			}
		}

		// std::string timestamp = getTimestamp();
		// std::string nickname = nickname_.data();
		// std::array<char, MAX_IP_PACK_SIZE> formatted_msg;

		// // boundary correctness is guarded by protocol.hpp
		// strcpy(formatted_msg.data(), timestamp.c_str());
		// strcat(formatted_msg.data(), nickname.c_str());
		// strcat(formatted_msg.data(), read_msg_.data());

		// std::cout << "received command = " << formatted_msg.data() << std::endl;

		// end new
		boost::asio::async_read(socket_,
								boost::asio::buffer(read_msg_, read_msg_.size()),
								strand_.wrap(boost::bind(&personInRoom::readHandler, shared_from_this(), _1)));
	}
	else
	{
		room_.leave(shared_from_this());
	}
}

void personInRoom::writeHandler(const boost::system::error_code &error)
{
	if (!error)
	{
		write_msgs_.pop_front();

		if (!write_msgs_.empty())
		{
			boost::asio::async_write(socket_,
									 boost::asio::buffer(write_msgs_.front(), write_msgs_.front().size()),
									 strand_.wrap(boost::bind(&personInRoom::writeHandler, shared_from_this(), _1)));
		}
	}
	else
	{
		room_.leave(shared_from_this());
	}
}

server::server(boost::asio::io_service &io_service,
			   boost::asio::io_service::strand &strand,
			   const tcp::endpoint &endpoint,
			   MotorController *motorController)
	: io_service_(io_service), strand_(strand), acceptor_(io_service, endpoint),
	  motorController_(motorController)
{
	run();
}

void server::run()
{
	std::shared_ptr<personInRoom> new_participant(new personInRoom(io_service_, strand_, room_, motorController_));
	acceptor_.async_accept(new_participant->socket(), strand_.wrap(boost::bind(&server::onAccept, this, new_participant, _1)));
}

void server::onAccept(std::shared_ptr<personInRoom> new_participant, const boost::system::error_code &error)
{
	if (!error)
	{
		new_participant->start();
	}

	run();
}

bool ServerController::startServer()
{
	try
	{

		// if (argc < 2)
		//{
		//	std::cerr << "Usage: chat_server <port> [<port> ...]\n";
		//	return 1;
		// }

		std::shared_ptr<boost::asio::io_service> io_service(new boost::asio::io_service);
		boost::shared_ptr<boost::asio::io_service::work> work(new boost::asio::io_service::work(*io_service));
		boost::shared_ptr<boost::asio::io_service::strand> strand(new boost::asio::io_service::strand(*io_service));

		std::cout << "[" << std::this_thread::get_id() << "]"
				  << "server starts" << std::endl;

		std::list<std::shared_ptr<server>> servers;

		// for (int i = 1; i < argc; ++i)
		//{

		tcp::endpoint endpoint(tcp::v4(), serverPort);
		std::shared_ptr<server> a_server(new server(*io_service, *strand, endpoint, &motorController));
		servers.push_back(a_server);
		//}

		boost::thread_group workers;
		for (int i = 0; i < 1; ++i)
		{
			boost::thread *t = new boost::thread{boost::bind(&workerThread::run, io_service)};

#ifdef __linux__
			// bind cpu affinity for worker thread in linux
			cpu_set_t cpuset;
			CPU_ZERO(&cpuset);
			CPU_SET(i, &cpuset);
			pthread_setaffinity_np(t->native_handle(), sizeof(cpu_set_t), &cpuset);
#endif
			workers.add_thread(t);
		}

		workers.join_all();
	}
	catch (std::exception &e)
	{
		std::cerr << "Exception: " << e.what() << "\n";
		return false;
	}

	return true;
}

int main(int argc, char *argv[])
{
	wiringPiSetupGpio();
	ServerController serverController;
	serverController.startServer();
}