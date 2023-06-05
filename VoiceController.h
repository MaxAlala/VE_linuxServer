#pragma once

class VoiceController {

public:
VoiceController() {
    isVoiceCommandActive = false;
}

void sayGreetings(); 
    bool isVoiceCommandActive;
private:

};