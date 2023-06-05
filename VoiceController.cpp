#include "VoiceController.h"
#include <thread>
#include<stdlib.h>
void VoiceController::sayGreetings() {
        if (isVoiceCommandActive == false) {

    auto lambdaSpeakingCommand = [this]()
    {
        isVoiceCommandActive = true;
        system("nvlc greetings.mp3 --play-and-exit");
        isVoiceCommandActive = false;
    };

    std::thread threadSpeakingCommand(lambdaSpeakingCommand);
    threadSpeakingCommand.detach();
    }
}

