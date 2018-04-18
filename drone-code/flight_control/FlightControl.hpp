/*
    Copyright (C) 2014 Parrot SA

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Parrot nor the names
      of its contributors may be used to endorse or promote products
      derived from this software without specific prior written
      permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
    OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
    OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
    SUCH DAMAGE.
*/

#pragma once

#include <functional>
#include <vector>

extern "C" {
#include <libARController/ARController.h>
#include <libARDiscovery/ARDiscovery.h>
#include <libARSAL/ARSAL.h>
}

enum class StateName {
    Landed,
    Flying,
    CalibrationFront,  // front of aircraft calibration
    CalibrationTail,   // at back
    WaypointTravel,
    WaypointStabilizing,
    WaypointStable
};

struct DroneState {
    // video streaming
    int video_stream_state = -1;
    // autonomous max rotation speed
    float max_auto_rotation_speed = 0.0;
    // max pitch/roll
    float current_max_tilt = 0.0;
    float range_min_tilt = 0.0;
    float range_max_tilt = 0.0;
    // max autonomous vertical speed
    float max_auto_vert_speed = 0.0;
    // max autonomous horizontal speed
    float max_auto_horz_speed = 0.0;
    // picture format
    int pic_format = -1;
    // white balance
    int white_balance = -1;
    // picture state
    bool pic_attempted = false;
    // move to target
    int moveto_status = -1;
};

class StateHandler {
public:
    StateHandler(StateName beginState) : currentState(beginState){};

    void transitionState(StateName newState) {
        printf("Current state: %u, New State: %u\r\n", (int)currentState, (int)newState);
        if (newState == currentState) return;
        if (states.find(currentState) != states.end() && states[currentState].exitState)
            states[currentState].exitState();
        currentState = newState;
        if (states.find(newState) != states.end() && states[newState].enterState) states[newState].enterState();
    }
    void continueState(const std::vector<int>& axes_vals, const std::vector<int>& pressed_buttons) {
        if (states.find(currentState) != states.end() && states[currentState].continueState)
            states[currentState].continueState(axes_vals, pressed_buttons);
    }
    void commandReceived(eARCONTROLLER_DICTIONARY_KEY commandKey, ARCONTROLLER_DICTIONARY_ELEMENT_t* elementDictionary,
                         ARCONTROLLER_Device_t* deviceController) {
        if (states.find(currentState) != states.end() && states[currentState].commandRecieved)
            states[currentState].commandRecieved(commandKey, elementDictionary, deviceController);
    }

    void setEnterFunc(StateName stateName, std::function<void()> func) { states[stateName].enterState = func; }
    void setContinueFunc(StateName stateName,
                         std::function<void(const std::vector<int>&, const std::vector<int>&)> func) {
        states[stateName].continueState = func;
    }
    void setExitFunc(StateName stateName, std::function<void()> func) { states[stateName].exitState = func; }
    void setCommandFunc(StateName stateName,
                        std::function<void(eARCONTROLLER_DICTIONARY_KEY, ARCONTROLLER_DICTIONARY_ELEMENT_t*,
                                           ARCONTROLLER_Device_t*)> func) {
        states[stateName].commandRecieved = func;
    }
    StateName state() { return currentState; }

private:
    struct StateFunctions {
        std::function<void()> enterState;
        std::function<void(const std::vector<int>&, const std::vector<int>&)> continueState;
        std::function<void()> exitState;
        std::function<void(eARCONTROLLER_DICTIONARY_KEY, ARCONTROLLER_DICTIONARY_ELEMENT_t*, ARCONTROLLER_Device_t*)>
            commandRecieved;
    };
    std::map<StateName, StateFunctions> states;
    StateName currentState;
};

void manualControl(const std::vector<int>& axes_vals, const std::vector<int>& pressed_buttons);

// called when the state of the device controller has changed
void stateChanged(eARCONTROLLER_DEVICE_STATE newState, eARCONTROLLER_ERROR error, void* customData);

// called when a command has been received from the drone
void commandReceived(eARCONTROLLER_DICTIONARY_KEY commandKey, ARCONTROLLER_DICTIONARY_ELEMENT_t* elementDictionary,
                     void* customData);

void cleanup();
void deviceSetup();

// called when a streaming frame has been received
eARCONTROLLER_ERROR didReceiveFrameCallback(ARCONTROLLER_Frame_t* frame, void* customData);

eARCONTROLLER_ERROR decoderConfigCallback(ARCONTROLLER_Stream_Codec_t codec, void* customData);
