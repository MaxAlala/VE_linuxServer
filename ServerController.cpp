
#include "ServerController.h"
#include "MotorController.h"
#include "VoiceController.h"

#include <boost/array.hpp>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <wiringSerial.h>
#include "serialController.h"
using boost::asio::ip::tcp;
using namespace std;
using namespace cv;

//    std::string message = "C M " + std::to_string(steps1) + " " + std::to_string(steps2) + " " + std::to_string(commandID) + "\n";
//    std::string message = "C H " + std::to_string(commandID) + " " + std::to_string(Axis1Direction) + "\n";

void chatRoom::enter(std::shared_ptr<participant> participant, const std::string &nickname)
{
    participants_.insert(participant);
    name_table_[participant] = nickname;
    for (int i = 0; i < NUMBER_OF_ANGLES; i++)
        name_to_id[i][nickname] = 0;

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
                           MotorController *motorController,
                           VoiceController *voiceController)
    : socket_(io_service), strand_(strand), room_(room), motorController_(motorController), voiceController_(voiceController)
{

    // memset(read_msg_.data(), '\0', MAX_IP_PACK_SIZE);
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
    nicknameStr = nickname_.data();
    std::cout << "nickname=" << nickname_.data() << std::endl;
    room_.enter(shared_from_this(), std::string(nickname_.data()));

    boost::asio::async_read(socket_,
                            boost::asio::buffer(read_msg_, read_msg_.size()),
                            strand_.wrap(boost::bind(&personInRoom::readHandler, shared_from_this(), _1)));
}

stringParser::CommandTypes stringParser::detectCommandType(std::array<char, MAX_IP_PACK_SIZE> &read_msg_)
{
    std::cout << "type detection \n";
    if (read_msg_.data()[0] == 'C')
    {
        if (read_msg_.data()[2] == 'A')
        {
            return CommandTypes::AUTOHOMING;
        }
        // else if (read_msg_.data()[2] == 'M' && read_msg_.data()[3] == 'I')
        // {
        //     return CommandTypes::REQUEST_ID;
        // }
        else if (read_msg_.data()[2] == 'M')
        {
            return CommandTypes::MOVE;
        }
        else if (read_msg_.data()[2] == 'V')
        {
            return CommandTypes::VOICE;
        }
    }
    return CommandTypes::NOT_DETECTED_COMMAND;
}

bool stringParser::removeSomeTrash(std::array<char, MAX_IP_PACK_SIZE> &read_msg_)
{
    std::string s = read_msg_.data();
    std::string delimiter = "C";

    // s = s.substr(4, s.length());
    // s.append(" ");
    // memset(angles.data(), -1, NUMBER_OF_ANGLES);

    size_t pos = 0;
    std::string currentSubString;
    // int anglesAndID[NUMBER_OF_ANGLES + 1];
    // int numberCounter = 0;
    // int axis_ = 0;
    // int angle_ = 0;
    // int id_ = 0;
    // int numberOfParams = 3;
    // while ((pos = s.find(delimiter)) != std::string::npos)
    // {
    // 	currentSubString = s.substr(0, pos);
    // 	std::cout << currentSubString << "\n";
    // 	currentSubString = s.substr(pos,s.length());
    // 	std::cout << currentSubString << "\n";
    // 	int currentNumber = 0;
    // 	s.erase(0, pos);
    // 	currentSubString.copy(read_msg_.data(),currentSubString.length());
    // 	std::cout <<"new read_msg = " << read_msg_.data() << "\n";
    // 	// read_msg_.data() = currentSubString;
    // }

    if (pos = s.find(delimiter))
    {
        currentSubString = s.substr(pos + 1, s.length());
        std::cout << currentSubString << "\n";
        currentSubString.copy(read_msg_.data(), currentSubString.length());
        std::cout << "new read_msg = " << read_msg_.data() << "\n";
    }
    return true;
}

//   Relative||Absolute Axis Angle ID
// C M A 0 10 2
bool stringParser::parseCommandMove(std::array<char, MAX_IP_PACK_SIZE> &read_msg_, std::array<int, NUMBER_OF_ANGLES> &angles, int &axis, int &id, bool& isRelativeMovement)
{
    std::string s = read_msg_.data();
    std::string delimiter = " ";
    // std::cout << " s =" << s << "\n";
    // std::cout << " s[5] =" << s[5] << "\n";

    if(s[4] == 'A')
      isRelativeMovement = false;
    else if(s[4] == 'R')
      isRelativeMovement = true;
    else return false;
    // std::cout << " s1=" << "\n";

    s = s.substr(6, s.length());
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

    // enum class VoiceCommandTypes
    // {
    // 	NOT_DETECTED_COMMAND = 0,
    // 	GREETINGS,
    // };

// C V GR = greetings
// 
bool stringParser::parseVoiceCommand(std::array<char, MAX_IP_PACK_SIZE> &read_msg_, VoiceCommandTypes &voiceComTyp)
{
    std::string s = read_msg_.data();

    if (s[4] == 'G' && s[5] == 'R') {
        voiceComTyp = VoiceCommandTypes::GREETINGS;
        std::cout << "parseVoiceCommand GREETINGS \n ";
        return true;
    }

    std::cout << "parseVoiceCommand NOTHING \n ";

    return false;
}

// protocol 3
// C M 0 10 0
//  axis angle id
// if map[name] id == received id + !M = start movement DONE
// if movement finished = inc users id DONE
// if map[name] id == received id + M = do nothing = there is many users = someones users id will be increased = to him will be send a command to inc its id + other
// user will probably start movement with his current id

// if map[name] id > received id = sends stop + inc
// if map[name] id < received id + M = do nothing // waits for finishing == id inc, then it will start

// if movement finished == increase map[name].id

void personInRoom::readHandler(const boost::system::error_code &error)
{
    if (!error)
    {

        // room_.broadcast(read_msg_, shared_from_this());
        // read_msg_.data()[MAX_IP_PACK_SIZE-1]=0;
        std::cout << read_msg_.data() << " =received str" << std::endl;
        stringParser parser;
        // parser.removeSomeTrash(read_msg_);
        stringParser::CommandTypes type = parser.detectCommandType(read_msg_);

		// NOT_DETECTED_COMMAND = 0,
		// AUTOHOMING,
		// MOVE,
		// REQUEST_ID,
		// VOICE,

        // new
        std::cout << "current type = " << int(type) << std::endl;

        static int counter = 0;

        int axis = -1;
        int id = -1;
        bool isRelativeMovement = true;
        std::string msg;

        if (motorController_->isTheAutohomingRunning && (type == stringParser::CommandTypes::MOVE))
        {
            boost::asio::async_read(socket_,
                                    boost::asio::buffer(read_msg_, read_msg_.size()),
                                    strand_.wrap(boost::bind(&personInRoom::readHandler, shared_from_this(), _1)));
            return;
        }

        if (type == stringParser::CommandTypes::VOICE)
        {
            stringParser::VoiceCommandTypes voiceComType;
            if (parser.parseVoiceCommand(read_msg_, voiceComType))
            {
                if (voiceController_->isVoiceCommandActive) {

                    //C V GR
                    msg = "C V\n";

                    std::cout << " sends stop C V =" << msg;
                    memset(motorController_->textMsg.data(), '\0', MAX_IP_PACK_SIZE);
                    memcpy(motorController_->textMsg.data(), msg.data(), msg.size());
                    // std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    onMessage(motorController_->textMsg);
                }

                if (voiceComType==stringParser::VoiceCommandTypes::GREETINGS) {
                    voiceController_->sayGreetings();
                }
            }
        }

        // // sends current id of axis
        // if (type == stringParser::CommandTypes::REQUEST_ID)
        // {
        //     // C MI axis
        //     // is there movement + received request for ID = stop this request. stop condition1.
        //     if (parser.parseCommandMoveID(read_msg_, axis))
        //     {
        //         if (!motorController_->isThereMovementToSpecificAngle[axis])
        //         {
        //             // else if no movement = sends current ID
        //             // C MI axis id

        //             std::cout << "sends ID = " << motorController_->currentMovementID[axis] << std::endl;
        //             msg = "C MI " + std::to_string(axis) + " " + std::to_string(motorController_->currentMovementID[axis]) + "\n";
        //             std::cout << "sended str= " << msg << std::endl;

        //             memset(motorController_->textMsg.data(), '\0', MAX_IP_PACK_SIZE);
        //             memcpy(motorController_->textMsg.data(), msg.data(), msg.size());
        //             // std::this_thread::sleep_for(std::chrono::milliseconds(200));
        //             onMessage(motorController_->textMsg);
        //         }
        //     }
        // }

        // protocol 3
        // C M 0 10 0
        //  axis angle id

        // if movement finished == increase map[name].id = increase server user's id
        // if map[name] id == received id + M = sends stop + inc
        // if map[name] id > received id = sends stop + inc
        // if map[name] id < received id + M = do nothing // waits for finishing 
        // if map[name] id <= received id + !M = set new map.id+start M // waits for finishing == id inc, then it will start

        // C M axis angle id
        else if (type == stringParser::CommandTypes::MOVE)
        {

            std::string msg;
            // is it a move command?
            if (parser.parseCommandMove(read_msg_, motorController_->receivedAngle, axis, id, isRelativeMovement))
            {
                // starts movement if id is bigger than current id + no movement
                // one person has id 9 at robot + 10 at client, second person has id 20 at robot and 19 at client.
                // 2nd starts actions = now id of person at server is 20+stops+incs=21, 1st sends id = it moves cause 9 < 10 and !movement.
                // std::string nickStr(nickname_.data());
                std::cout << "	name_to_id[nickname]= " << room_.name_to_id[axis][nicknameStr] << std::endl;
                if (id >= room_.name_to_id[axis][nicknameStr] && !motorController_->isThereMovementToSpecificAngle[axis])
                {
                    room_.name_to_id[axis][nicknameStr] = id;
                    motorController_->personControllingAxis[axis] = this;

                    std::cout << "starts movement, axis angle id " << axis << " " << motorController_->receivedAngle[axis] << " " << id << std::endl;

                    motorController_->savedReceivedAngle[axis] = motorController_->receivedAngle[axis];

//need to inverse angle, cause math model angle is different from magnet encoder angle.
// in math model ccw increases a value, in magnet decoder model ccw decreases a value 
    // 				 shouldInverseSignOfReceivedAngleFromClient[0] = true;
    //				 shouldInverseSignOfReceivedAngleFromClient[1] = false;

// so we need to invert a sign of math model angle so that magnet model was correct after adding new math model angle
    // isCcwIncreasesValueOfMagnetEncoder[0] = false; // ccw decreases angle val from magnet, but in math model ccw increases an angle
    // isCcwIncreasesValueOfMagnetEncoder[1] = true; //+
// axisBorders[0].left = -60;
// axisBorders[0].right = 300;
// axisBorders[0].home = 120;

// axisBorders[1].left = -13;
// axisBorders[1].right = 167;
// axisBorders[1].home = 77;

                    motorController_->moveAxisToSomeAngleI(axis, isRelativeMovement, motorController_->savedReceivedAngle[axis]);

                    // stop condition. id doesnt equal to current id
                }

                // sends id back and requests to increase user clients id untill it matches
                // MSI = move stop increment
                else if (room_.name_to_id[axis][nicknameStr] > id)
                {
                    // increment val=move+stop+increment
                    msg = "C MSI " + std::to_string(axis) + " " + std::to_string(id) + "\n";
                    std::cout << " sends stop+inc =" << msg;
                    memset(motorController_->textMsg.data(), '\0', MAX_IP_PACK_SIZE);
                    memcpy(motorController_->textMsg.data(), msg.data(), msg.size());
                    // std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    onMessage(motorController_->textMsg);
                }
                // requests to increase id + stop sending movement command
                else if (id == room_.name_to_id[axis][nicknameStr] && motorController_->isThereMovementToSpecificAngle[axis])
                {

                    // increment val
                    msg = "C MSI " + std::to_string(axis) + " " + std::to_string(id) + "\n";
                    std::cout << " sends stop+inc =" << msg;
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
                motorController_->isTheAutohomingRunning = false;
                memset(motorController_->textMsg.data(), '\0', MAX_IP_PACK_SIZE);
                memcpy(motorController_->textMsg.data(), msg.data(), msg.size());
                std::cout << "autohoming is finished=" << msg << std::endl;
                onMessage(motorController_->textMsg);
            }
            else
            {
                if (!motorController_->isTheAutohomingRunning)
                {
                    motorController_->isTheAutohomingRunning = true;
                    motorController_->startAutohoming();
                }
            }
        }
        // else if (type == stringParser::CommandTypes::STOP_AUTOHOMING)
        // {
        // 	std::string msg;

        // 	// stop condition
        // 	if (motorController_->isTheAutohomingStarted == false)
        // 	{
        // 		msg = "C AS";
        // 		motorController_->isTheAutohomingStarted = false;
        // 		memset(motorController_->textMsg.data(), '\0', MAX_IP_PACK_SIZE);
        // 		memcpy(motorController_->textMsg.data(), msg.data(), msg.size());
        // 		std::cout << "autohoming is finished=" << msg << std::endl;
        // 		onMessage(motorController_->textMsg);
        // 	}
        // 	else
        // 	{
        // 		motorController_->isTheAutohomingStarted = true;
        // 		motorController_->startAutohoming();
        // 	}
        // }
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
               MotorController* motorController,
               VoiceController* voiceController)
    : io_service_(io_service), strand_(strand), acceptor_(io_service, endpoint),
      motorController_(motorController), voiceController_(voiceController)
{
    run();
}

void server::run()
{
    std::shared_ptr<personInRoom> new_participant(new personInRoom(io_service_, strand_, room_, motorController_, voiceController_));
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
        std::shared_ptr<server> a_server(new server(*io_service, *strand, endpoint, motorController.get(), voiceController.get()));
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
void ServerController::startCamBroadCasting()
{
    std::thread camThread(&ServerController::startCamBroadCastingProc, this);
    camThread.detach();
}

void ServerController::startAngleBroadcasting()
{
    std::thread thread_(&ServerController::startAngleBroadcastingProc, this);
    thread_.detach();
}

void ServerController::startLidarDistanceDetection()
{
    std::thread thread_(&ServerController::startLidarDistanceDetectionProc, this);
    thread_.detach();
}

void ServerController::startLidarDistanceBroadcasting()
{
    std::thread thread_(&ServerController::startLidarDistanceBroadcastingProc, this);
    thread_.detach();
}


cv::Mat ServerController::retrieve_data()
{

    // std::string image_path = "img.jpg";
    cv::Mat image;
    cap.read(image);
    // cout << "rows + cols = " << image.rows << " " << image.cols << endl;

    if (image.empty())
    {
        assert("ERROR! blank frame grabbed\n");
    }

    // image = imread(image_path, cv::IMREAD_COLOR);
    if (!image.data)
    {
        std::cout << "Could not open or find the image" << std::endl;
    }
    return image;
}

void ServerController::startCamBroadCastingProc()
{

    cap.open(0);
    if (!cap.isOpened())
    {
        assert("ERROR! Unable to open camera\n");
    }

    // boost::thread thrd(&servershow);
    try
    {
        boost::asio::io_service io_service;
        tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), camPort));

        for (;;)
        {
            tcp::socket socket(io_service);
            acceptor.accept(socket);
            boost::system::error_code ignored_error;

            // while (true) {
            // retrieve the frame to be sent
            cv::Mat frame = retrieve_data();
            std::vector<std::uint8_t> buff;
            cv::imencode(".jpg", frame, buff);

            // now we send the header message
            std::string image_dimensions = "640Wx480H";
            std::string image_buff_bytes = std::to_string(buff.size());
            std::string message_header = image_dimensions + "," + image_buff_bytes;
            // std::cout << "sending measage header of " << std::to_string(message_header.length()) << " bytes...." << std::endl;
            message_header.append(63 - message_header.length(), ' ');
            message_header = message_header + '\0';

            socket.write_some(boost::asio::buffer(message_header), ignored_error);

            socket.write_some(boost::asio::buffer(buff), ignored_error);
            //}
            // flag = true;
        }
    }
    catch (std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
    // thrd.join();
}

void ServerController::startLidarDistanceDetectionProc()
{
    int TFMINI_DATA_Len = 9;
    int TFMINT_DATA_HEAD = 0x59;
    uint16_t cordist = 0;
    uint8_t chk_cal = 0;
    uint8_t ar[9];
    int counter = 0;
    try
    {

        SimpleSerial serial("/dev/ttyUSB0", 115200);

        // serial.writeString("Hello world\n");

        while (true)
        {
            char readChar = serial.readChar();
            // std::cout << readChar << std::endl;
            
            if (readChar == 'Y' && counter == 0) {
                ar[counter] = readChar;
                ++counter;
                continue;
            }

            if (readChar == 'Y' && counter == 1) {
                ar[counter] = readChar;
                ++counter;
                continue;
            }


            if (counter >= 2) {
                ar[counter] = readChar;
                ++counter;
                
            }

            if(counter >= 4) {
                cordist = ar[2] | (ar[3] << 8);
                // std::cout << "distance = " << cordist << "\n";
                if (cordist > 0 && cordist < 12000) {
                    currentLidarDistance = cordist;
                }
                cordist = 0;
            }

            if(counter == 8){
                // std::this_thread::sleep_for(std::chrono::milliseconds(200));
                counter = 0;
            }


        }
    }
    catch (boost::system::system_error &e)
    {
        cout << "Error: " << e.what() << endl;
        return;
    }
}

void ServerController::startAngleBroadcastingProc()
{

    // cap.open(0);
    // if (!cap.isOpened()) {
    //     assert("ERROR! Unable to open camera\n");
    // }

    // boost::thread thrd(&servershow);
    try
    {
        boost::asio::io_service io_service;
        tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), anglesPort));

        for (;;)
        {
            tcp::socket socket(io_service);
            acceptor.accept(socket);
            boost::system::error_code ignored_error;

            // while (true) {
            // retrieve the frame to be sent
            //  cv::Mat frame = retrieve_data();
            //  std::vector<std::uint8_t> buff;
            //  cv::imencode(".jpg", frame, buff);

            // now we send the header message
            std::string msgToSend;
            AngleFilterMakeHomeToBeZero homeFilter;
            for (int i = 0; i < NUMBER_OF_ANGLES; i++)
            {
                msgToSend.append(std::to_string(homeFilter.returnAngleWhereHomeAngleIsZero(motorController->axisBorders[i].home, motorController->currentAngle[i], motorController->isCcwIncreasesValueOfMagnetEncoder[i])) + " ");
            }

            msgToSend += "\n";
            // std::string image_buff_bytes = std::to_string(buff.size());
            // std::string message_header = image_dimensions + "," + image_buff_bytes;
            // std::cout << "sending measage header of " << std::to_string(message_header.length()) << " bytes...." << std::endl;
            // message_header.append(63 - message_header.length(), ' ');
            // message_header = message_header + '\0';

            std::cout << "sending these angles= " << msgToSend << std::endl;
            socket.write_some(boost::asio::buffer(msgToSend), ignored_error);

            // socket.write_some(boost::asio::buffer(buff), ignored_error);
            //}
            // flag = true;
        }
    }
    catch (std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
    // thrd.join();
}

void ServerController::startLidarDistanceBroadcastingProc()
{
    try
    {
        boost::asio::io_service io_service;
        tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), lidarDistancePort));

        for (;;)
        {
            // std::this_thread::sleep_for(std::chrono::milliseconds(200));
            tcp::socket socket(io_service);
            acceptor.accept(socket);
            boost::system::error_code ignored_error;
            std::string msgToSend;
            // std::cout << " currentLidarDistance = " << currentLidarDistance << "\n";
            msgToSend = std::to_string(currentLidarDistance);
            socket.write_some(boost::asio::buffer(msgToSend), ignored_error);
        }
    }
    catch (std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
}

