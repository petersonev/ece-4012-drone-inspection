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

#include <curses.h>
#include <errno.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <chrono>
#include <ctime>
#include <iostream>
#include <map>
#include <mutex>

#include "FlightControl.hpp"
#include "Gamepad.hpp"

#include "path_planner.h"

// Defines for Bebop SDK
#define TAG "BebopDroneInspection"

#define BEBOP_IP_ADDRESS "192.168.42.1"
#define BEBOP_DISCOVERY_PORT 44444

#define DISPLAY_WITH_MPLAYER 1

#define FIFO_DIR_PATTERN "/tmp/arsdk_XXXXXX"
#define FIFO_NAME "arsdk_fifo"

// Global variables for bebop
static char fifo_dir[] = FIFO_DIR_PATTERN;
static char fifo_name[128] = "";

FILE* videoOut = nullptr;
pid_t videoChild = 0;

std::chrono::time_point<std::chrono::system_clock> start;
// std::chrono::time_point<std::chrono::system_clock> send_vid_stream_enable;
std::ofstream pose_log;

ARSAL_Sem_t stateSem;
ARCONTROLLER_Device_t* deviceController = nullptr;
ARCONTROLLER_FEATURE_ARDrone3_t*
    drone;  // Hacky c solution so functioned referenced even if deviceController is nullptr

// Begin our actual implementation

constexpr bool debug = false;
constexpr bool debug_axes = false;

bool taking_video = false;
/* List of possible csv path files
 *
example_output.csv
matlab_out_c141_no_engines_2.csv
matlab_out_c141_no_engines_4.csv
matlab_out_c141_no_engines.csv
matlab_out_c141_with_engines_2.csv
matlab_out_c141_with_engines_4.csv
matlab_out_c141_with_engines.csv
matlab_out_car.csv
matlab_out_car_reduced.csv
matlab_out.csv
matlab_out_f84_2.csv
matlab_out_f84_4.csv
matlab_out_f84.csv
matlab_out_plane_4m.csv
matlab_out_plane_large.csv
matlab_out_reduced.csv
 *
 */
// constexpr auto matlab_path_filename = "path_planning/test_data/matlab_out_car.csv";
// constexpr auto matlab_path_filename = "path_planning/test_data/matlab_out_f84_4.csv";
// constexpr auto matlab_path_filename = "path_planning/test_data/matlab_out_f84.csv";
constexpr auto matlab_path_filename = "path_planning/test_data/matlab_out_c130.csv";
// constexpr auto matlab_path_filename = "path_planning/test_data/matlab_out_car_reduced.csv";
constexpr double standoff_dst = 5;  // meters from aircraft for calibration position
constexpr auto waypoint_alt = 7.7;

std::mutex mutex;  // mutex to protect state information

// Used to handle states, start with state landed
StateHandler stateHandler(StateName::Landed);

DroneState state;  // mainly state of settings right now
path::Pose pose;   // current position/orientation of the drone
double battery = -1;

path::GeoCoord front_coord;
path::GeoCoord tail_coord;

int currentWaypoint = 0;
int nextWaypoint = currentWaypoint;

const std::vector<path::Waypoint> debug_waypoints = {
    path::Waypoint(33.778869, -84.402260, 300, 2.5, true), path::Waypoint(33.778763, -84.402465, 30, 2.5, true),
    path::Waypoint(33.778743, -84.402591, 30, 2.5, false), path::Waypoint(33.778918, -84.402667, 120, 2.5, true),
    path::Waypoint(33.779020, -84.402382, 200, 2.5, true), path::Waypoint(33.778940, -84.402269, 200, 2.5, false)};

std::vector<path::Waypoint> waypoints = debug_waypoints;

std::string logging_path = "";

// this is a bit gross, but using chrono seemed worse
std::string timestamp() {
    static char name[40];
    time_t now = time(0);
    strftime(name, sizeof(name), "%Y-%m-%d_%H:%M:%S", localtime(&now));
    return std::string(name);
}

bool almost_equal(float a, float b, float tol) { return abs(a - b) < tol; }

// Template for calling aRDrone3 functions, checks success() to determine if
// the message was acknowledged by the drone, if it isn't retries at some rate
template <class T, class... Args>
auto droneCall(T func, Args&&... args) {
    if (debug || !func) return false;
    func(deviceController->aRDrone3, std::forward<Args>(args)...);
    return true;
}

// Transition program states based on the state returned from the drone
void flightState(eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE state) {
    if (state == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_LANDED ||
        state == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_TAKINGOFF ||
        state == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_LANDING ||
        state == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_EMERGENCY_LANDING) {
        // Transition to landed if landed/landing/taking off
        stateHandler.transitionState(StateName::Landed);
    } else if (stateHandler.state() == StateName::Landed) {
        if (state == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_HOVERING ||
            state == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_FLYING) {
            stateHandler.transitionState(StateName::CalibrationFront);
        }
    }
}

// Transition program states based on status in the move to event
void moveToStatus(eARCOMMANDS_ARDRONE3_PILOTINGSTATE_MOVETOCHANGED_STATUS status) {
    if (stateHandler.state() == StateName::Flying || stateHandler.state() == StateName::WaypointStable) {
        if (status == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_MOVETOCHANGED_STATUS_RUNNING)
            stateHandler.transitionState(StateName::WaypointTravel);
    } else if (stateHandler.state() == StateName::WaypointTravel) {
        if (status == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_MOVETOCHANGED_STATUS_DONE) {
            currentWaypoint = nextWaypoint;
            stateHandler.transitionState(StateName::WaypointStabilizing);
        } else if (status == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_MOVETOCHANGED_STATUS_ERROR ||
                   status == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_MOVETOCHANGED_STATUS_CANCELED) {
            stateHandler.transitionState(StateName::Flying);
        }
    }
}

// Callback for for when in the flying state
void flyingContinue(const std::vector<int>& axes_vals, const std::vector<int>& pressed_buttons) {
    for (auto bt : pressed_buttons) {
        switch (bt) {
            case Gamepad::START:
                std::cout << "Start waypoint " << waypoints[currentWaypoint] << std::endl;
                droneCall(drone->sendPilotingMoveTo, waypoints[currentWaypoint].lat(), waypoints[currentWaypoint].lon(),
                          waypoints[currentWaypoint].alt(),
                          ARCOMMANDS_ARDRONE3_PILOTING_MOVETO_ORIENTATION_MODE_HEADING_DURING,
                          waypoints[currentWaypoint].heading());
                break;
            case Gamepad::BACK:
                std::cout << "End waypoint" << std::endl;
                nextWaypoint = (currentWaypoint + waypoints.size() - 1) % waypoints.size();
                droneCall(drone->sendPilotingMoveTo, waypoints[nextWaypoint].lat(), waypoints[nextWaypoint].lon(),
                          waypoints[nextWaypoint].alt(),
                          ARCOMMANDS_ARDRONE3_PILOTING_MOVETO_ORIENTATION_MODE_HEADING_DURING,
                          waypoints[nextWaypoint].heading());
                break;
        }
    }
}

// Callback for when in the stabilized waypoint state
void WaypointStableContinue(const std::vector<int>& axes_vals, const std::vector<int>& pressed_buttons) {
    for (auto bt : pressed_buttons) {
        switch (bt) {
            case Gamepad::START:
                std::cout << "Next waypoint" << std::endl;
                nextWaypoint = (currentWaypoint + 1) % waypoints.size();
                droneCall(drone->sendPilotingMoveTo, waypoints[nextWaypoint].lat(), waypoints[nextWaypoint].lon(),
                          waypoints[nextWaypoint].alt(),
                          ARCOMMANDS_ARDRONE3_PILOTING_MOVETO_ORIENTATION_MODE_HEADING_DURING,
                          waypoints[nextWaypoint].heading());
                break;
            case Gamepad::BACK:
                std::cout << "Previous waypoint" << std::endl;
                nextWaypoint = (currentWaypoint + waypoints.size() - 1) % waypoints.size();
                droneCall(drone->sendPilotingMoveTo, waypoints[nextWaypoint].lat(), waypoints[nextWaypoint].lon(),
                          waypoints[nextWaypoint].alt(),
                          ARCOMMANDS_ARDRONE3_PILOTING_MOVETO_ORIENTATION_MODE_HEADING_DURING,
                          waypoints[nextWaypoint].heading());
                break;
        }
    }
}

// Callback for when in the calibration front state
void CalibrationFrontContinue(const std::vector<int>& axes_vals, const std::vector<int>& pressed_buttons) {
    {
        std::lock_guard<std::mutex> lock(mutex);
        front_coord = pose.coord;
    }

    for (auto bt : pressed_buttons) {
        switch (bt) {
            case Gamepad::START:
                std::cout << "Done with front calibration "
                          << "Front coord: " << front_coord << std::endl;
                stateHandler.transitionState(StateName::CalibrationTail);
                break;
        }
    }
}

// Callback for when in the calibration tail state
void CalibrationTailContinue(const std::vector<int>& axes_vals, const std::vector<int>& pressed_buttons) {
    {
        std::lock_guard<std::mutex> lock(mutex);
        tail_coord = pose.coord;
    }

    for (auto bt : pressed_buttons) {
        switch (bt) {
            case Gamepad::START:
                std::cout << "Done with tail calibration "
                          << "Front coord: " << front_coord << " "
                          << "Back coord: " << tail_coord << " " << std::endl;

                printf("Writing ideal pose log\n");
                waypoints = plan_path(matlab_path_filename, front_coord, tail_coord, waypoint_alt, standoff_dst);

                auto ideal_pose_log = std::ofstream(logging_path + "/ideal_poselog.csv", std::ofstream::out);

                ideal_pose_log << "t " + path::Pose::get_field_names() << std::endl;

                for (std::size_t i = 0; i < waypoints.size(); ++i) {
                    ideal_pose_log << i << " " << waypoints[i].pose << std::endl;
                }

                ideal_pose_log.close();

                currentWaypoint = 0;
                std::cout << "Num waypoints: " << waypoints.size() << std::endl;

                // back to waiting for start to go to waypoints
                stateHandler.transitionState(StateName::Flying);
                break;
        }
    }
}

int main(int argc, char* argv[]) {
    Gamepad gp(8000);

    drone = ARCONTROLLER_FEATURE_ARDrone3_New(nullptr, nullptr);

    // under logs, create a timestamped directory for this flight
    logging_path += "logs/" + timestamp();

    // this should be cross platform compatible
    const int dir_err = mkdir(logging_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    if (dir_err == -1) {
        std::cout << "Failed to create logs directory at: " << logging_path;
        return 1;
    }

    pose_log = std::ofstream(logging_path + "/poselog.csv", std::ofstream::out);

    pose_log << "t " + path::Pose::get_field_names() << std::endl;

    if (!debug) deviceSetup();

    stateHandler.setContinueFunc(StateName::Flying, flyingContinue);
    stateHandler.setContinueFunc(StateName::WaypointStable, WaypointStableContinue);
    stateHandler.setEnterFunc(StateName::WaypointStabilizing, []() {
        sleep(2);
        stateHandler.transitionState(StateName::WaypointStable);
    });

    // in calibration front, continuously set front lat/lon point to current pose
    // until user says it's close enough, should let us manually adjust
    stateHandler.setContinueFunc(StateName::CalibrationFront, CalibrationFrontContinue);
    // in calibration back, continuously set back lat/lon to current pose
    // until user says it's close enough, should let us manually adjust
    stateHandler.setContinueFunc(StateName::CalibrationTail, CalibrationTailContinue);

    // Callback for when waypoint has stabilized (on state enter)
    stateHandler.setEnterFunc(StateName::WaypointStable, [&]() {
        nextWaypoint = (currentWaypoint + 1) % waypoints.size();
        printf("Next waypoint: %d\n", nextWaypoint);

        if (!taking_video) {
            auto desired_pic_format = ARCOMMANDS_ARDRONE3_PICTURESETTINGS_PICTUREFORMATSELECTION_TYPE_JPEG_FISHEYE;
            droneCall(drone->sendPictureSettingsPictureFormatSelection, desired_pic_format);

            sleep(1);

            droneCall(drone->sendMediaRecordPictureV2);
            printf("Took JPEG_FISHEYE pic\n");
        }

        droneCall(drone->sendPilotingMoveTo, waypoints[nextWaypoint].lat(), waypoints[nextWaypoint].lon(),
                  waypoints[nextWaypoint].alt(), ARCOMMANDS_ARDRONE3_PILOTING_MOVETO_ORIENTATION_MODE_HEADING_DURING,
                  waypoints[nextWaypoint].heading());
    });

    // Reset the current waypoint when landed
    stateHandler.setEnterFunc(StateName::Landed, []() { currentWaypoint = 0; });

    printf("- send StreamingVideoEnable ...\n");

    auto vid_enable = 1;
    constexpr float tol = 0.01f;
    bool suc = false;

    // send_vid_stream_enable = std::chrono::system_clock::now();

    suc = droneCall(drone->sendMediaStreamingVideoEnable, vid_enable);

    if (!suc) printf("Failed to set video enable \r\n");

    auto desired_pic_format = ARCOMMANDS_ARDRONE3_PICTURESETTINGS_PICTUREFORMATSELECTION_TYPE_JPEG;
    droneCall(drone->sendPictureSettingsPictureFormatSelection, desired_pic_format);

    // TODO: this is fake-failing because it's using max auto rotation speed, not actual
    // max speed
    auto max_rot_speed = 20.0f;
    suc = droneCall(drone->sendSpeedSettingsMaxRotationSpeed, max_rot_speed);

    if (!suc) printf("Failed to set max rotation speed\r\n");

    auto max_tilt = 30.0f;
    suc = droneCall(drone->sendPilotingSettingsMaxTilt, max_tilt);

    if (!suc) printf("Failed to set max tilt: %d\r\n", almost_equal(max_tilt, state.current_max_tilt, 0.01f));

    auto max_vert_speed = 3.0f;
    suc = droneCall(drone->sendSpeedSettingsMaxVerticalSpeed, max_vert_speed);

    if (!suc) printf("Failed to set max vert speed\r\n");

    auto max_horz_speed = 0.5f;
    suc = droneCall(drone->sendPilotingSettingsSetAutonomousFlightMaxHorizontalSpeed, max_horz_speed);

    if (!suc) printf("Failed to set max horz speed\r\n");

    auto wb_setting = ARCOMMANDS_ARDRONE3_PICTURESETTINGS_AUTOWHITEBALANCESELECTION_TYPE_DAYLIGHT;
    suc = droneCall(drone->sendPictureSettingsAutoWhiteBalanceSelection, wb_setting);

    if (!suc) printf("Failed to set white balance\r\n");

    printf("Entering main loop\n");

    std::vector<int> axes_vals(4);
    std::vector<int> pressed_buttons;

    start = std::chrono::system_clock::now();

    bool sdlRunning = true;
    while (sdlRunning) {
        // get inputs, issue commands based on inputs
        sdlRunning = gp.poll(axes_vals, pressed_buttons);

        manualControl(axes_vals, pressed_buttons);

        for (auto bt : pressed_buttons) {
            switch (bt) {
                case Gamepad::L: {
                    auto value = !taking_video ? ARCOMMANDS_ARDRONE3_MEDIARECORD_VIDEOV2_RECORD_START
                                               : ARCOMMANDS_ARDRONE3_MEDIARECORD_VIDEOV2_RECORD_STOP;

                    printf("Take video? : %d\n", !taking_video);
                    droneCall(drone->sendMediaRecordVideoV2, value);

                    taking_video = !taking_video;
                    break;
                }
                case Gamepad::R: {
                    printf("Taking picture\n");
                    droneCall(drone->sendMediaRecordPictureV2);
                    break;
                }
                default:
                    break;
            }
        }

        {
            // synchronize before the continueState call
            stateHandler.continueState(axes_vals, pressed_buttons);
        }

        usleep(50);
    }

    pose_log.close();

    if (!debug) cleanup();

    return 0;
}

// Manual control that is continuously running
void manualControl(const std::vector<int>& axes_vals, const std::vector<int>& pressed_buttons) {
    for (auto bt : pressed_buttons) {
        switch (bt) {
            case Gamepad::A:
                std::cout << "Taking off" << std::endl;
                droneCall(drone->sendPilotingTakeOff);
                break;
            case Gamepad::B:
                std::cout << "Landing" << std::endl;
                droneCall(drone->sendPilotingLanding);
                break;
            case Gamepad::X:
                std::cout << "X" << std::endl;
                break;
            case Gamepad::Y:
                std::cout << "ISSUING EMERGENCY - MOTORS OFF" << std::endl;
                droneCall(drone->sendPilotingEmergency);
                break;
            case Gamepad::DPAD_L:
                std::cout << "Flip left" << std::endl;
                droneCall(drone->sendAnimationsFlip, ARCOMMANDS_ARDRONE3_ANIMATIONS_FLIP_DIRECTION_LEFT);
                break;
            case Gamepad::DPAD_R:
                std::cout << "Flip right" << std::endl;
                droneCall(drone->sendAnimationsFlip, ARCOMMANDS_ARDRONE3_ANIMATIONS_FLIP_DIRECTION_RIGHT);
                break;
            case Gamepad::DPAD_U:
                std::cout << "Flip front" << std::endl;
                droneCall(drone->sendAnimationsFlip, ARCOMMANDS_ARDRONE3_ANIMATIONS_FLIP_DIRECTION_FRONT);
                break;
            case Gamepad::DPAD_D:
                std::cout << "Flip back" << std::endl;
                droneCall(drone->sendAnimationsFlip, ARCOMMANDS_ARDRONE3_ANIMATIONS_FLIP_DIRECTION_BACK);
                break;
            case Gamepad::R:
                std::cout << "RB" << std::endl;
                break;
            default:
                break;
        }
    }

    std::vector<int> cur_commanded(4);
    for (std::size_t i = 0; i < axes_vals.size(); ++i) {
        float normalized = axes_vals[i] / 65536.0f;
        cur_commanded[i] = static_cast<int>(normalized * 100 * 2);
    }

    if (debug && debug_axes) {
        for (auto c : cur_commanded) {
            std::cout << std::setprecision(5) << c << " ";
        }
        std::cout << std::endl;
    }

    // Yaw handling
    droneCall(drone->setPilotingPCMDYaw, cur_commanded[0]);
    // Altitude handling
    droneCall(drone->setPilotingPCMDGaz, -cur_commanded[1]);
    // Roll handling
    droneCall(drone->setPilotingPCMDRoll, cur_commanded[2]);
    // Pitch handling
    droneCall(drone->setPilotingPCMDPitch, -cur_commanded[3]);

    // bool set_piloting_flag = (abs(cur_commanded[2]) + abs(cur_commanded[3])) >= 1;
    bool set_piloting_flag = true;
    droneCall(drone->setPilotingPCMDFlag, set_piloting_flag);
}

/********************************************
 *                                          *
 *     Parrot Stuff (mostly leave alone)    *
 *                                          *
 ********************************************/

// called when the state of the device controller has changed
void stateChanged(eARCONTROLLER_DEVICE_STATE newState, eARCONTROLLER_ERROR error, void* customData) {
    printf("    - stateChanged newState: %d .....\n", newState);

    switch (newState) {
        case ARCONTROLLER_DEVICE_STATE_STOPPED:
            ARSAL_Sem_Post(&(stateSem));
            break;
        case ARCONTROLLER_DEVICE_STATE_RUNNING:
            ARSAL_Sem_Post(&(stateSem));
            break;
        default:
            break;
    }
}

// called when a command has been received from the drone
void commandReceived(eARCONTROLLER_DICTIONARY_KEY commandKey, ARCONTROLLER_DICTIONARY_ELEMENT_t* elementDictionary,
                     void* customData) {
    // synchronize, don't allow state machine to run until the command has been received
    // we use a unique lock so we get access to a memeber to unlock before doing the
    // state transitions
    std::unique_lock<std::mutex> lock(mutex);

    ARCONTROLLER_Device_t* deviceController = static_cast<ARCONTROLLER_Device_t*>(customData);

    if (deviceController == nullptr) return;

    ARCONTROLLER_DICTIONARY_ARG_t* arg = nullptr;
    ARCONTROLLER_DICTIONARY_ELEMENT_t* element = nullptr;
    // if the command received is a battery state changed
    switch (commandKey) {
        case ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED: {
            // Called on battery percent changed
            ARCONTROLLER_DICTIONARY_ELEMENT_t* singleElement = nullptr;
            if (elementDictionary == nullptr) break;

            HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, singleElement);
            if (singleElement == nullptr) break;

            HASH_FIND_STR(singleElement->arguments,
                          ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED_PERCENT, arg);
            if (arg == nullptr) break;

            printf("Battery: %d\n", arg->value.U8);
            ::battery = arg->value.U8;
        } break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_GPSLOCATIONCHANGED:
            // Called when drone position changes
            // int8_t latitudeAccuracy, longitudeAccuracy, altitudeAccuracy;
            HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
            if (element != NULL) {
                HASH_FIND_STR(element->arguments,
                              ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_GPSLOCATIONCHANGED_LATITUDE, arg);
                if (arg != NULL) {
                    pose.coord.lat = arg->value.Double;
                }
                HASH_FIND_STR(element->arguments,
                              ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_GPSLOCATIONCHANGED_LONGITUDE, arg);
                if (arg != NULL) {
                    pose.coord.lon = arg->value.Double;
                }
                HASH_FIND_STR(element->arguments,
                              ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_GPSLOCATIONCHANGED_ALTITUDE, arg);
                if (arg != NULL) {
                    pose.alt = arg->value.Double;
                    // printf("Alt: %f\n", pose.alt);
                }
                HASH_FIND_STR(element->arguments,
                              ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_GPSLOCATIONCHANGED_LATITUDE_ACCURACY,
                              arg);
                if (arg != NULL) {
                    // latitudeAccuracy = arg->value.I8;
                }
                HASH_FIND_STR(element->arguments,
                              ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_GPSLOCATIONCHANGED_LONGITUDE_ACCURACY,
                              arg);
                if (arg != NULL) {
                    // longitudeAccuracy = arg->value.I8;
                }
                HASH_FIND_STR(element->arguments,
                              ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_GPSLOCATIONCHANGED_ALTITUDE_ACCURACY,
                              arg);
                if (arg != NULL) {
                    // altitudeAccuracy = arg->value.I8;
                }
                auto cur = std::chrono::system_clock::now();
                std::chrono::duration<double> diff = cur - start;
                pose_log << diff.count() << " " << pose << std::endl;
                // std::cout << pose << std::endl;
            }
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ATTITUDECHANGED:
            HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
            if (element != NULL) {
                HASH_FIND_STR(element->arguments,
                              ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ATTITUDECHANGED_ROLL, arg);
                if (arg != NULL) {
                    pose.roll = arg->value.Float;
                }
                HASH_FIND_STR(element->arguments,
                              ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ATTITUDECHANGED_PITCH, arg);
                if (arg != NULL) {
                    pose.pitch = arg->value.Float;
                }
                HASH_FIND_STR(element->arguments,
                              ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ATTITUDECHANGED_YAW, arg);
                if (arg != NULL) {
                    pose.heading = arg->value.Float;
                }
            }
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED:
            // Called when flying state changes
            HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
            if (element != nullptr) {
                HASH_FIND_STR(element->arguments,
                              ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE, arg);
                if (arg != nullptr) {
                    lock.unlock();
                    // state transition, uses mutex
                    flightState(
                        static_cast<eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE>(arg->value.I32));
                    lock.lock();
                }
            }
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_MOVETOCHANGED:
            HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
            if (element != nullptr) {
                HASH_FIND_STR(element->arguments,
                              ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_MOVETOCHANGED_STATUS, arg);
                if (arg != nullptr) {
                    auto status = static_cast<eARCOMMANDS_ARDRONE3_PILOTINGSTATE_MOVETOCHANGED_STATUS>(arg->value.I32);
                    state.moveto_status = status;
                    lock.unlock();
                    moveToStatus(status);  // state transition, uses mutex
                    lock.lock();
                }
            }
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED:
            HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
            if (element != NULL) {
                HASH_FIND_STR(element->arguments,
                              ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED_ENABLED, arg);
                if (arg != NULL) {
                    // auto cur = std::chrono::system_clock::now();
                    // std::chrono::duration<double> diff = cur - send_vid_stream_enable;
                    // std::cout << "End send media streaming video enable: " << diff.count() << std::endl;
                    state.video_stream_state = arg->value.I32;
                }
            }
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSETTINGSSTATE_AUTONOMOUSFLIGHTMAXROTATIONSPEED:
            HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
            if (element != NULL) {
                HASH_FIND_STR(
                    element->arguments,
                    ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSETTINGSSTATE_AUTONOMOUSFLIGHTMAXROTATIONSPEED_VALUE,
                    arg);
                if (arg != NULL) {
                    // this is never actually getting called, why?
                    state.max_auto_rotation_speed = arg->value.Float;
                }
            }
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSETTINGSSTATE_MAXTILTCHANGED:
            HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
            if (element != NULL) {
                HASH_FIND_STR(element->arguments,
                              ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSETTINGSSTATE_MAXTILTCHANGED_CURRENT, arg);
                if (arg != NULL) {
                    state.current_max_tilt = arg->value.Float;
                }
                HASH_FIND_STR(element->arguments,
                              ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSETTINGSSTATE_MAXTILTCHANGED_MIN, arg);
                if (arg != NULL) {
                    state.range_min_tilt = arg->value.Float;
                }
                HASH_FIND_STR(element->arguments,
                              ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSETTINGSSTATE_MAXTILTCHANGED_MAX, arg);
                if (arg != NULL) {
                    state.range_max_tilt = arg->value.Float;
                }
            }
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSETTINGSSTATE_AUTONOMOUSFLIGHTMAXVERTICALSPEED:
            HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
            if (element != NULL) {
                HASH_FIND_STR(
                    element->arguments,
                    ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSETTINGSSTATE_AUTONOMOUSFLIGHTMAXVERTICALSPEED_VALUE,
                    arg);
                if (arg != NULL) {
                    state.max_auto_vert_speed = arg->value.Float;
                }
            }
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSETTINGSSTATE_AUTONOMOUSFLIGHTMAXHORIZONTALSPEED:
            HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
            if (element != NULL) {
                HASH_FIND_STR(
                    element->arguments,
                    ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSETTINGSSTATE_AUTONOMOUSFLIGHTMAXHORIZONTALSPEED_VALUE,
                    arg);
                if (arg != NULL) {
                    state.max_auto_horz_speed = arg->value.Float;
                }
            }
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PICTURESETTINGSSTATE_PICTUREFORMATCHANGED:
            HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
            if (element != NULL) {
                HASH_FIND_STR(element->arguments,
                              ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PICTURESETTINGSSTATE_PICTUREFORMATCHANGED_TYPE, arg);
                if (arg != NULL) {
                    state.pic_format = arg->value.I32;
                }
            }
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PICTURESETTINGSSTATE_AUTOWHITEBALANCECHANGED:
            HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
            if (element != NULL) {
                HASH_FIND_STR(element->arguments,
                              ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PICTURESETTINGSSTATE_AUTOWHITEBALANCECHANGED_TYPE,
                              arg);
                if (arg != NULL) {
                    state.white_balance = arg->value.I32;
                }
            }
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_MEDIARECORDEVENT_PICTUREEVENTCHANGED:
            HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
            if (element != NULL) {
                HASH_FIND_STR(element->arguments,
                              ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_MEDIARECORDEVENT_PICTUREEVENTCHANGED_EVENT, arg);
                if (arg != NULL) {
                    // this could theoretically fail, but if it fails once, it'll probably
                    // keep failing, so I don't think it's very useful data right now
                    state.pic_attempted = true;
                }
            }
            break;
        default:
            break;
    }
    stateHandler.commandReceived(commandKey, elementDictionary, deviceController);
}

eARCONTROLLER_ERROR decoderConfigCallback(ARCONTROLLER_Stream_Codec_t codec, void* customData) {
    if (videoOut != nullptr) {
        if (codec.type == ARCONTROLLER_STREAM_CODEC_TYPE_H264) {
            if (DISPLAY_WITH_MPLAYER) {
                fwrite(codec.parameters.h264parameters.spsBuffer, codec.parameters.h264parameters.spsSize, 1, videoOut);
                fwrite(codec.parameters.h264parameters.ppsBuffer, codec.parameters.h264parameters.ppsSize, 1, videoOut);

                fflush(videoOut);
            }
        }

    } else {
        printf("videoOut is nullptr.\n");
    }

    return ARCONTROLLER_OK;
}

eARCONTROLLER_ERROR didReceiveFrameCallback(ARCONTROLLER_Frame_t* frame, void* customData) {
    if (videoOut != nullptr) {
        if (frame != nullptr) {
            if (DISPLAY_WITH_MPLAYER) {
                fwrite(frame->data, frame->used, 1, videoOut);

                fflush(videoOut);
            }
        } else {
            printf("frame is nullptr.\n");
        }
    } else {
        printf("videoOut is nullptr.\n");
    }

    return ARCONTROLLER_OK;
}

void cleanup() {
    eARCONTROLLER_ERROR error = ARCONTROLLER_OK;
    // we are here because of a disconnection or user has quit, so safely delete
    // everything
    if (deviceController != nullptr) {
        eARCONTROLLER_DEVICE_STATE deviceState = ARCONTROLLER_Device_GetState(deviceController, &error);
        if ((error == ARCONTROLLER_OK) && (deviceState != ARCONTROLLER_DEVICE_STATE_STOPPED)) {
            printf("Disconnecting ...\n");

            error = ARCONTROLLER_Device_Stop(deviceController);

            if (error == ARCONTROLLER_OK) {
                // wait state update update
                ARSAL_Sem_Wait(&(stateSem));
            }
        }

        printf("ARCONTROLLER_Device_Delete ...\n");
        ARCONTROLLER_Device_Delete(&deviceController);

        if (DISPLAY_WITH_MPLAYER) {
            fflush(videoOut);
            fclose(videoOut);

            if (videoChild > 0) {
                kill(videoChild, SIGKILL);
            }
        }
    }

    ARSAL_Sem_Destroy(&(stateSem));

    unlink(fifo_name);
    rmdir(fifo_dir);

    printf("-- END --\n");
    exit(0);
}

void deviceSetup() {
    // local declarations
    ARDISCOVERY_Device_t* device = nullptr;
    eARCONTROLLER_ERROR error = ARCONTROLLER_OK;

    if (mkdtemp(fifo_dir) == nullptr) {
        printf("Mkdtemp failed.\n");
        exit(1);
    }
    snprintf(fifo_name, sizeof(fifo_name), "%s/%s", fifo_dir, FIFO_NAME);

    if (mkfifo(fifo_name, 0666) < 0) {
        printf("Mkfifo failed: %d, %s\n", errno, strerror(errno));
        exit(1);
    }

    ARSAL_Sem_Init(&(stateSem), 0, 0);

    int isBebop2 = 1;  // for this project, we're always flying with Bebop2

    printf("-- Bebop 2 Sample --\n");

    if (DISPLAY_WITH_MPLAYER) {
        // fork the process to launch mplayer
        if ((videoChild = fork()) == 0) {
            execlp("xterm", "xterm", "-e", "mplayer", "-demuxer", "h264es", fifo_name, "-benchmark", "-really-quiet",
                   nullptr);
            printf(
                "Missing mplayer, you will not see "
                "the video. Please install mplayer "
                "and xterm.\n");
            exit(1);
        }
    }

    if (DISPLAY_WITH_MPLAYER) {
        videoOut = fopen(fifo_name, "w");
    }

    // create a discovery device
    printf("- init discovey device ... \n");
    eARDISCOVERY_ERROR errorDiscovery = ARDISCOVERY_OK;

    device = ARDISCOVERY_Device_New(&errorDiscovery);
    if (errorDiscovery != ARDISCOVERY_OK) {
        printf("Discovery error :%s\n", ARDISCOVERY_Error_ToString(errorDiscovery));
        cleanup();
    }

    printf("    - ARDISCOVERY_Device_InitWifi ...\n");
    // create a Bebop drone discovery device
    // (ARDISCOVERY_PRODUCT_ARDRONE)

    if (isBebop2) {
        errorDiscovery = ARDISCOVERY_Device_InitWifi(device, ARDISCOVERY_PRODUCT_BEBOP_2, "bebop2", BEBOP_IP_ADDRESS,
                                                     BEBOP_DISCOVERY_PORT);
    } else {
        errorDiscovery = ARDISCOVERY_Device_InitWifi(device, ARDISCOVERY_PRODUCT_ARDRONE, "bebop", BEBOP_IP_ADDRESS,
                                                     BEBOP_DISCOVERY_PORT);
    }

    if (errorDiscovery != ARDISCOVERY_OK) {
        printf("Discovery error :%s\n", ARDISCOVERY_Error_ToString(errorDiscovery));
        cleanup();
    }

    // create a device controller
    deviceController = ARCONTROLLER_Device_New(device, &error);

    if (error != ARCONTROLLER_OK) {
        printf("Creation of deviceController failed.\n");
        cleanup();
    }

    printf("- delete discovey device ... \n");
    ARDISCOVERY_Device_Delete(&device);

    // add the state change callback to be informed when the device controller
    // starts, stops...
    error = ARCONTROLLER_Device_AddStateChangedCallback(deviceController, stateChanged, deviceController);

    if (error != ARCONTROLLER_OK) {
        printf("add State callback failed.\n");
        cleanup();
    }

    // add the command received callback to be informed when a command has been
    // received from the device
    error = ARCONTROLLER_Device_AddCommandReceivedCallback(deviceController, commandReceived, deviceController);

    if (error != ARCONTROLLER_OK) {
        printf("add callback failed.\n");
        cleanup();
    }

    // add the frame received callback to be informed when a streaming frame has
    // been received from the device
    printf("- set Video callback ... \n");
    error = ARCONTROLLER_Device_SetVideoStreamCallbacks(deviceController, decoderConfigCallback,
                                                        didReceiveFrameCallback, nullptr, nullptr);

    if (error != ARCONTROLLER_OK) {
        printf("- error: %s\n", ARCONTROLLER_Error_ToString(error));
        cleanup();
    }

    printf("Connecting ...\n");
    error = ARCONTROLLER_Device_Start(deviceController);

    if (error != ARCONTROLLER_OK) {
        printf("- error :%s\n", ARCONTROLLER_Error_ToString(error));
        cleanup();
    }

    // wait state update update
    ARSAL_Sem_Wait(&(stateSem));

    eARCONTROLLER_DEVICE_STATE deviceState = ARCONTROLLER_Device_GetState(deviceController, &error);

    if ((error != ARCONTROLLER_OK) || (deviceState != ARCONTROLLER_DEVICE_STATE_RUNNING)) {
        printf("- deviceState :%d\n", deviceState);
        printf("- error :%s\n", ARCONTROLLER_Error_ToString(error));
        cleanup();
    }
}
