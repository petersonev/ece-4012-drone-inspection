#pragma once

#include <SDL.h>
#include <iostream>

#include <vector>

class Gamepad {
public:
    enum Axes {
        LX = SDL_CONTROLLER_AXIS_LEFTX,
        LY = SDL_CONTROLLER_AXIS_LEFTY,
        RX = SDL_CONTROLLER_AXIS_RIGHTX,
        RY = SDL_CONTROLLER_AXIS_RIGHTY,
        ZL = SDL_CONTROLLER_AXIS_TRIGGERLEFT,
        ZR = SDL_CONTROLLER_AXIS_TRIGGERRIGHT
    };

    enum Buttons {
        A = SDL_CONTROLLER_BUTTON_A,
        B = SDL_CONTROLLER_BUTTON_B,
        X = SDL_CONTROLLER_BUTTON_X,
        Y = SDL_CONTROLLER_BUTTON_Y,
        BACK = SDL_CONTROLLER_BUTTON_BACK,
        GUIDE = SDL_CONTROLLER_BUTTON_GUIDE,
        START = SDL_CONTROLLER_BUTTON_START,
        LS = SDL_CONTROLLER_BUTTON_LEFTSTICK,
        RS = SDL_CONTROLLER_BUTTON_RIGHTSTICK,
        L = SDL_CONTROLLER_BUTTON_LEFTSHOULDER,
        R = SDL_CONTROLLER_BUTTON_RIGHTSHOULDER,
        DPAD_U = SDL_CONTROLLER_BUTTON_DPAD_UP,
        DPAD_D = SDL_CONTROLLER_BUTTON_DPAD_DOWN,
        DPAD_L = SDL_CONTROLLER_BUTTON_DPAD_LEFT,
        DPAD_R = SDL_CONTROLLER_BUTTON_DPAD_RIGHT
    };

    Gamepad(int deadzone) : controller(nullptr), deadzone(deadzone) {
        // SDL2 will only report events when the window has focus, so set
        // this hint as we don't have a window
        SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
        const auto sdl_flags =
            SDL_INIT_TIMER | SDL_INIT_VIDEO | SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER | SDL_INIT_HAPTIC;

        if (SDL_Init(sdl_flags) < 0) {
            std::cout << "Failed to initialize SDL, gamepad won't work." << std::endl;
        }

        SDL_GameControllerAddMappingsFromFile("gamecontrollerdb.txt");

        int num_joysticks = SDL_NumJoysticks();
        std::cout << "Gamepad count: " << num_joysticks << std::endl;
    }

    ~Gamepad() {
        if (SDL_GameControllerGetAttached(controller)) {
            SDL_GameControllerClose(controller);
        }
        SDL_Quit();
    }

    /*
     * ls_x - left stick,  x-dimension
     * ls_y - left stick,  y-dimension
     * rs_x - right stick, x-dimension
     * rs_y - right stick, y-dimension
     */
    bool poll(std::vector<int>& axes, std::vector<int>& pressed_buttons) {
        if (!SDL_GameControllerGetAttached(controller)) {
            if (SDL_IsGameController(0)) {
                controller = SDL_GameControllerOpen(0);
            }
            // clear axes
            for (auto& e : axes) {
                e = 0;
            }
        }

        bool sdl_running = true;

        if (axes.size() < 4) axes.resize(4, 0);
        pressed_buttons.clear();

        // process all current events
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
                case SDL_CONTROLLERAXISMOTION:
                    if (abs(event.caxis.value) < deadzone) {
                        axes[event.caxis.axis] = 0;
                    } else {
                        axes[event.caxis.axis] = event.caxis.value;
                    }
                    break;
                case SDL_CONTROLLERBUTTONDOWN:
                    pressed_buttons.push_back(event.cbutton.button);
                    break;
                case SDL_QUIT:
                    sdl_running = false;
                    break;
                default:
                    break;
            }
        }

        return sdl_running;
    }

private:
    SDL_GameController* controller;
    // SDL_Joystick* pad;
    SDL_Event event;

    int deadzone;
};
