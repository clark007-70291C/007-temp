#include "brain_screen.hpp"

#include "lemlib/chassis/chassis.hpp" // IWYU pragma: keep
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/display/lv_display.h"
#include "liblvgl/misc/lv_timer.h"
#include "liblvgl/widgets/bar/lv_bar.h"
#include "liblvgl/widgets/button/lv_button.h"
#include "liblvgl/widgets/label/lv_label.h"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

extern lemlib::Chassis chassis;
extern lemlib::ControllerSettings linearController;
extern lemlib::ControllerSettings angularController;
extern pros::MotorGroup liftMotors;
extern pros::Rotation liftRot;
extern double liftCentiDegreesToInches(double positionCentiDegrees);

namespace brain_screen {
std::atomic<AutonRoutine> selectedAutonRoutine{AutonRoutine::CLOSE};
lv_obj_t* autonSelectorScreen = nullptr;
lv_obj_t* matrixRainScreen = nullptr;
lv_obj_t* motorTemperatureScreen = nullptr;
lv_obj_t* motorTemperatureReturnScreen = nullptr;
lv_obj_t* pidTunerScreen = nullptr;
lv_obj_t* pidTunerReturnScreen = nullptr;
lv_timer_t* matrixRainTimer = nullptr;
lv_timer_t* motorTemperatureTimer = nullptr;
lv_obj_t* motorTemperatureLabels[2]{};
lv_obj_t* motorTemperatureBars[2]{};
lv_obj_t* poseLabels[3]{};
lv_obj_t* matrixPoseLabels[3]{};
lv_obj_t* selectorLiftHeightLabel = nullptr;
lv_obj_t* matrixLiftHeightLabel = nullptr;
lv_obj_t* selectorTempsButton = nullptr;
lv_obj_t* matrixTempsButton = nullptr;
lv_obj_t* selectorPidTunerButton = nullptr;
lv_obj_t* matrixPidTunerButton = nullptr;
std::atomic<bool> pidTunerDisabledForCompetition{false};
lv_obj_t* pidPoseLabels[3]{};
lv_obj_t* pidGainLabels[2][2]{};
float pidTunerStep = 0.1f;
struct PidGainControl {
    bool angular;
    std::uint8_t gain;
    bool increase;
};
PidGainControl pidGainControls[2][2][2]{};
float pidTunerSteps[] = {0.1f, 1.0f, 5.0f};
lv_obj_t* autonButtons[3]{};
lv_obj_t* autonButtonLabels[3]{};
struct SelectorRainStream {
    lv_obj_t* label;
    std::int32_t y;
    std::int32_t height;
    std::int32_t speed;
};

constexpr std::uint8_t selectorRainStreamCount = 32;
SelectorRainStream selectorRainStreams[selectorRainStreamCount]{};
std::int32_t selectorScreenHeight = 0;
bool selectorUiRevealed = false;
std::atomic<bool> autonScreenTransitionRequested{false};
struct MatrixStream {
    lv_obj_t* label;
    std::int32_t y;
    std::int32_t height;
    std::int32_t speed;
};

constexpr std::uint8_t matrixStreamCount = 36;
MatrixStream matrixStreams[matrixStreamCount]{};
std::int32_t matrixScreenHeight = 0;
AutonRoutine autonRoutineOptions[] = {
    AutonRoutine::CLOSE,
    AutonRoutine::FAR,
    AutonRoutine::SKILLS
};

std::int32_t selectorCenteredButtonX() {
    return (lv_obj_get_width(autonSelectorScreen) - 90) / 2;
}

std::int32_t selectorSideButtonLeftX(std::int32_t buttonWidth) {
    return selectorCenteredButtonX() - 12 - buttonWidth;
}

std::int32_t selectorSideButtonRightX() {
    return selectorCenteredButtonX() + 90 + 12;
}

std::int32_t selectorAutoButtonY(std::uint8_t index) {
    constexpr std::int32_t rowY[] = {8, 58, 108};
    return rowY[index];
}


void refreshPidTunerValues() {
    const lemlib::PID* controllers[] = {&chassis.lateralPID, &chassis.angularPID};
    const char* names[] = {"P", "D"};
    for (std::uint8_t controllerIndex = 0; controllerIndex < 2; ++controllerIndex) {
        const float gains[] = {controllers[controllerIndex]->getKP(),
                               controllers[controllerIndex]->getKD()};
        for (std::uint8_t gainIndex = 0; gainIndex < 2; ++gainIndex) {
            if (pidGainLabels[controllerIndex][gainIndex] != nullptr) {
                const auto hundredths = static_cast<std::int32_t>(
                    std::round(std::abs(gains[gainIndex]) * 100.0f));
                lv_label_set_text_fmt(pidGainLabels[controllerIndex][gainIndex], "%s: %s%d.%02d",
                                      names[gainIndex], gains[gainIndex] < 0 ? "-" : "",
                                      hundredths / 100, hundredths % 100);
            }
        }
    }
}

void changePidGain(lv_event_t* event) {
    lv_event_stop_bubbling(event);
    const auto* control = static_cast<const PidGainControl*>(lv_event_get_user_data(event));
    if (control == nullptr || control->gain > 2) return;

    lemlib::PID& pid = control->angular ? chassis.angularPID : chassis.lateralPID;
    lemlib::ControllerSettings& settings = control->angular ? angularController : linearController;
    float gains[] = {pid.getKP(), pid.getKI(), pid.getKD()};
    gains[control->gain] = std::max(0.0f, gains[control->gain] +
        (control->increase ? pidTunerStep : -pidTunerStep));
    settings.kP = gains[0];
    settings.kI = gains[1];
    settings.kD = gains[2];
    pid.setGains(gains[0], gains[1], gains[2]);
    refreshPidTunerValues();
}

void setPidTunerStep(lv_event_t* event) {
    lv_event_stop_bubbling(event);
    const auto* step = static_cast<const float*>(lv_event_get_user_data(event));
    if (step != nullptr) pidTunerStep = *step;
}

void returnFromPidTuner(lv_event_t* event) {
    lv_event_stop_bubbling(event);
    if (pidTunerReturnScreen == nullptr) return;
    lv_screen_load(pidTunerReturnScreen);
    if (pidTunerReturnScreen == matrixRainScreen && matrixRainTimer != nullptr) {
        lv_timer_resume(matrixRainTimer);
    }
}

void openPidTuner(lv_event_t* event) {
    lv_event_stop_bubbling(event);
    if (pros::competition::is_competition_switch()) {
        pidTunerDisabledForCompetition.store(true);
    }
    if (pidTunerDisabledForCompetition.load()) return;

    pidTunerReturnScreen = lv_screen_active();
    if (pidTunerReturnScreen == matrixRainScreen && matrixRainTimer != nullptr) {
        lv_timer_pause(matrixRainTimer);
    }

    if (pidTunerScreen == nullptr) {
        pidTunerScreen = lv_obj_create(nullptr);
        lv_obj_set_style_bg_color(pidTunerScreen, lv_color_hex(0x101820), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(pidTunerScreen, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_remove_flag(pidTunerScreen, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t* title = lv_label_create(pidTunerScreen);
        lv_label_set_text(title, "LIVE POSE / PID TUNER");
        lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 5);
        lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), LV_PART_MAIN);

        constexpr std::int32_t poseLabelX[] = {8, 82, 156};
        constexpr std::int32_t poseLabelWidth[] = {68, 68, 84};
        for (std::uint8_t i = 0; i < 3; ++i) {
            pidPoseLabels[i] = lv_label_create(pidTunerScreen);
            lv_obj_set_pos(pidPoseLabels[i], poseLabelX[i], 32);
            lv_obj_set_width(pidPoseLabels[i], poseLabelWidth[i]);
            lv_obj_set_style_text_font(pidPoseLabels[i], &lv_font_montserrat_12, LV_PART_MAIN);
            lv_obj_set_style_text_color(pidPoseLabels[i], lv_color_hex(0xFFFFFF), LV_PART_MAIN);
        }

        const char* controllerNames[] = {"LINEAR", "ANGULAR"};
        constexpr std::int32_t columnX[] = {8, 248};
        constexpr std::int32_t rowY[] = {82, 122};
        constexpr std::uint8_t gainsToShow[] = {0, 2};
        const lv_color_t white = lv_color_hex(0xFFFFFF);
        const lv_color_t black = lv_color_hex(0x000000);

        for (std::uint8_t controllerIndex = 0; controllerIndex < 2; ++controllerIndex) {
            lv_obj_t* heading = lv_label_create(pidTunerScreen);
            lv_label_set_text(heading, controllerNames[controllerIndex]);
            lv_obj_set_pos(heading, columnX[controllerIndex], 59);
            lv_obj_set_style_text_color(heading, white, LV_PART_MAIN);

            for (std::uint8_t gainRow = 0; gainRow < 2; ++gainRow) {
                pidGainLabels[controllerIndex][gainRow] = lv_label_create(pidTunerScreen);
                lv_obj_set_pos(pidGainLabels[controllerIndex][gainRow], columnX[controllerIndex],
                               rowY[gainRow] + 10);
                lv_obj_set_width(pidGainLabels[controllerIndex][gainRow], 78);
                lv_obj_set_style_text_font(pidGainLabels[controllerIndex][gainRow],
                                           &lv_font_montserrat_12, LV_PART_MAIN);
                lv_obj_set_style_text_color(pidGainLabels[controllerIndex][gainRow], white,
                                            LV_PART_MAIN);

                for (std::uint8_t direction = 0; direction < 2; ++direction) {
                    pidGainControls[controllerIndex][gainRow][direction] = {
                        controllerIndex == 1, gainsToShow[gainRow], direction == 1
                    };
                    lv_obj_t* button = lv_button_create(pidTunerScreen);
                    lv_obj_set_size(button, 70, 34);
                    lv_obj_set_pos(button, columnX[controllerIndex] + 82 + direction * 72,
                                   rowY[gainRow]);
                    lv_obj_set_style_bg_color(button, white, LV_PART_MAIN);
                    lv_obj_set_style_bg_opa(button, LV_OPA_COVER, LV_PART_MAIN);
                    lv_obj_set_style_radius(button, 5, LV_PART_MAIN);
                    lv_obj_t* label = lv_label_create(button);
                    lv_label_set_text(label, direction == 0 ? "DECREASE" : "INCREASE");
                    lv_obj_set_style_text_font(label, &lv_font_montserrat_12, LV_PART_MAIN);
                    lv_obj_set_style_text_color(label, black, LV_PART_MAIN);
                    lv_obj_center(label);
                    lv_obj_add_event_cb(button, changePidGain, LV_EVENT_CLICKED,
                                        &pidGainControls[controllerIndex][gainRow][direction]);
                }
            }
        }

        const char* stepNames[] = {"Step 0.1", "Step 1", "Step 5"};
        for (std::uint8_t i = 0; i < 3; ++i) {
            lv_obj_t* button = lv_button_create(pidTunerScreen);
            lv_obj_set_size(button, 78, 28);
            lv_obj_set_pos(button, 72 + i * 84, 169);
            lv_obj_t* label = lv_label_create(button);
            lv_label_set_text(label, stepNames[i]);
            lv_obj_center(label);
            lv_obj_add_event_cb(button, setPidTunerStep, LV_EVENT_CLICKED,
                                &pidTunerSteps[i]);
        }

        lv_obj_t* backButton = lv_button_create(pidTunerScreen);
        lv_obj_set_size(backButton, 72, 28);
        lv_obj_align(backButton, LV_ALIGN_BOTTOM_RIGHT, -8, -8);
        lv_obj_t* backLabel = lv_label_create(backButton);
        lv_label_set_text(backLabel, "Back");
        lv_obj_center(backLabel);
        lv_obj_add_event_cb(backButton, returnFromPidTuner, LV_EVENT_CLICKED, nullptr);
    }

    refreshPidTunerValues();
    lv_screen_load(pidTunerScreen);
}

void onAutonRoutineSelected(lv_event_t* event) {
    const auto* routine = static_cast<const AutonRoutine*>(lv_event_get_user_data(event));
    if (routine == nullptr) return;

    selectedAutonRoutine.store(*routine);

    for (std::uint8_t i = 0; i < 3; ++i) {
        if (autonRoutineOptions[i] == *routine) {
            lv_obj_add_state(autonButtons[i], LV_STATE_CHECKED);
            lv_obj_set_style_text_color(autonButtonLabels[i], lv_color_hex(0x000000), LV_PART_MAIN);
        } else {
            lv_obj_remove_state(autonButtons[i], LV_STATE_CHECKED);
            lv_obj_set_style_text_color(autonButtonLabels[i], lv_color_hex(0xFFFFFF), LV_PART_MAIN);
        }
    }
}

void updatePoseLabels(lv_timer_t*) {
    const lemlib::Pose pose = chassis.getPose();
    const double coordinates[] = {pose.x, pose.y, pose.theta};
    const char* names[] = {"X", "Y", "Theta"};
    const auto liftHeightTenths = static_cast<std::int32_t>(std::round(
        std::abs(liftCentiDegreesToInches(liftRot.get_position())) * 10.0));
    const auto liftHeightMagnitude = std::abs(liftHeightTenths);

    for (std::uint8_t i = 0; i < 3; ++i) {
        const auto tenths = static_cast<std::int32_t>(std::round(coordinates[i] * 10.0));
        const auto magnitude = std::abs(tenths);
        const char* sign = tenths < 0 ? "-" : "";
        if (pidPoseLabels[i] != nullptr) {
            lv_label_set_text_fmt(pidPoseLabels[i], "%s: %s%d.%d",
                                  names[i], sign, magnitude / 10, magnitude % 10);
        }
        if (poseLabels[i] != nullptr) {
            lv_label_set_text_fmt(poseLabels[i], "%s: %s%d.%d", names[i], sign,
                                  magnitude / 10, magnitude % 10);
        }
        if (matrixPoseLabels[i] != nullptr) {
            lv_label_set_text_fmt(matrixPoseLabels[i], "%s: %s%d.%d", names[i], sign,
                                  magnitude / 10, magnitude % 10);
        }
    }

    if (selectorLiftHeightLabel != nullptr) {
        lv_label_set_text_fmt(selectorLiftHeightLabel, "Lift: %d.%d in",
                              liftHeightMagnitude / 10, liftHeightMagnitude % 10);
    }
    if (matrixLiftHeightLabel != nullptr) {
        lv_label_set_text_fmt(matrixLiftHeightLabel, "Lift: %d.%d in",
                              liftHeightMagnitude / 10, liftHeightMagnitude % 10);
    }
}

lv_color_t rainTextColorAtY(std::int32_t y, std::int32_t screenHeight) {
    if (screenHeight <= 1) return lv_color_hex(0x000000);

    const std::int32_t clampedY = y < 0 ? 0 :
        (y >= screenHeight ? screenHeight - 1 : y);
    const std::int32_t progress = clampedY * 100 / (screenHeight - 1);
    const std::int32_t transition = progress <= 40 ? 0 :
        (progress >= 60 ? 255 : (progress - 40) * 255 / 20);
    const std::int32_t smoothTransition =
        transition * transition * (765 - 2 * transition) / (255 * 255);
    return lv_color_mix(lv_color_hex(0xFFFFFF), lv_color_hex(0x000000),
                        static_cast<std::uint8_t>(smoothTransition));
}

void createMatrixRainScreen();
void animateSelectorButtonsOut();
void openMotorTemperatureScreen(lv_event_t* event);
void updateMotorTemperatureLabels(lv_timer_t*);
void addMotorTemperatureButton(lv_obj_t* screen, bool aboveAutonButtons);

void disablePidTunerForCompetitionSwitch() {
    if (pros::competition::is_competition_switch()) {
        pidTunerDisabledForCompetition.store(true);
    }
    if (!pidTunerDisabledForCompetition.load()) return;

    if (selectorPidTunerButton != nullptr) {
        lv_obj_add_flag(selectorPidTunerButton, LV_OBJ_FLAG_HIDDEN);
    }
    if (matrixPidTunerButton != nullptr) {
        lv_obj_add_flag(matrixPidTunerButton, LV_OBJ_FLAG_HIDDEN);
    }
    if (lv_screen_active() == pidTunerScreen) {
        lv_screen_load(pidTunerReturnScreen != nullptr
            ? pidTunerReturnScreen : autonSelectorScreen);
        if (pidTunerReturnScreen == matrixRainScreen && matrixRainTimer != nullptr) {
            lv_timer_resume(matrixRainTimer);
        }
    }
}

void updateSelectorRain(lv_timer_t*) {
    disablePidTunerForCompetitionSwitch();

    if (autonScreenTransitionRequested.exchange(false)) {
        if (selectorUiRevealed) {
            animateSelectorButtonsOut();
        } else {
            createMatrixRainScreen();
        }
        return;
    }

    if (lv_screen_active() != autonSelectorScreen) return;

    for (SelectorRainStream& stream : selectorRainStreams) {
        stream.y += stream.speed;
        if (stream.y >= selectorScreenHeight) {
            stream.y = -stream.height;
        }

        lv_obj_set_y(stream.label, stream.y);
        lv_obj_set_style_text_color(
            stream.label,
            rainTextColorAtY(stream.y + stream.height / 2, selectorScreenHeight),
            LV_PART_MAIN);
    }
}

void setAnimatedObjectY(void* object, std::int32_t y) {
    lv_obj_set_y(static_cast<lv_obj_t*>(object), y);
}

void setAnimatedObjectX(void* object, std::int32_t x);

void setAnimatedObjectOpa(void* object, std::int32_t opa) {
    lv_obj_set_style_opa(static_cast<lv_obj_t*>(object), static_cast<lv_opa_t>(opa), LV_PART_MAIN);
}

void revealSelectorUi(lv_event_t*) {
    if (selectorUiRevealed) return;
    selectorUiRevealed = true;

    for (std::uint8_t i = 0; i < 3; ++i) {
        lv_obj_t* button = autonButtons[i];
        const std::int32_t buttonTargetX = selectorCenteredButtonX();
        const std::int32_t buttonTargetY = selectorAutoButtonY(i);
        lv_obj_remove_flag(button, LV_OBJ_FLAG_HIDDEN);

        lv_anim_t buttonAnimation;
        lv_anim_init(&buttonAnimation);
        lv_anim_set_var(&buttonAnimation, button);
        lv_obj_set_pos(button, buttonTargetX, -34);
        lv_anim_set_exec_cb(&buttonAnimation, setAnimatedObjectY);
        lv_anim_set_values(&buttonAnimation, -34, buttonTargetY);
        lv_anim_set_duration(&buttonAnimation, 450);
        lv_anim_set_delay(&buttonAnimation, i * 90);
        lv_anim_set_path_cb(&buttonAnimation, lv_anim_path_ease_out);
        lv_anim_start(&buttonAnimation);

        lv_obj_t* poseLabel = poseLabels[i];
        lv_obj_remove_flag(poseLabel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_opa(poseLabel, LV_OPA_TRANSP, LV_PART_MAIN);

        lv_anim_t poseAnimation;
        lv_anim_init(&poseAnimation);
        lv_anim_set_var(&poseAnimation, poseLabel);
        lv_anim_set_exec_cb(&poseAnimation, setAnimatedObjectOpa);
        lv_anim_set_values(&poseAnimation, LV_OPA_TRANSP, LV_OPA_COVER);
        lv_anim_set_duration(&poseAnimation, 450);
        lv_anim_set_delay(&poseAnimation, 220 + i * 70);
        lv_anim_set_path_cb(&poseAnimation, lv_anim_path_ease_out);
        lv_anim_start(&poseAnimation);
    }

    lv_obj_remove_flag(selectorLiftHeightLabel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_opa(selectorLiftHeightLabel, LV_OPA_TRANSP, LV_PART_MAIN);

    lv_anim_t liftHeightAnimation;
    lv_anim_init(&liftHeightAnimation);
    lv_anim_set_var(&liftHeightAnimation, selectorLiftHeightLabel);
    lv_anim_set_exec_cb(&liftHeightAnimation, setAnimatedObjectOpa);
    lv_anim_set_values(&liftHeightAnimation, LV_OPA_TRANSP, LV_OPA_COVER);
    lv_anim_set_duration(&liftHeightAnimation, 450);
    lv_anim_set_delay(&liftHeightAnimation, 150);
    lv_anim_set_path_cb(&liftHeightAnimation, lv_anim_path_ease_out);
    lv_anim_start(&liftHeightAnimation);

    lv_obj_remove_flag(selectorTempsButton, LV_OBJ_FLAG_HIDDEN);
    const std::int32_t tempsTargetX = selectorSideButtonLeftX(90);
    const std::int32_t sideRowY = selectorAutoButtonY(1) + 3;
    lv_obj_set_pos(selectorTempsButton, -90, sideRowY);
    lv_anim_t tempsAnimation;
    lv_anim_init(&tempsAnimation);
    lv_anim_set_var(&tempsAnimation, selectorTempsButton);
    lv_anim_set_exec_cb(&tempsAnimation, setAnimatedObjectX);
    lv_anim_set_values(&tempsAnimation, -90, tempsTargetX);
    lv_anim_set_duration(&tempsAnimation, 450);
    lv_anim_set_path_cb(&tempsAnimation, lv_anim_path_ease_out);
    lv_anim_start(&tempsAnimation);

    if (selectorPidTunerButton != nullptr) {
        lv_obj_remove_flag(selectorPidTunerButton, LV_OBJ_FLAG_HIDDEN);
        const std::int32_t pidTargetX = selectorSideButtonRightX();
        lv_obj_set_pos(selectorPidTunerButton, lv_obj_get_width(autonSelectorScreen), sideRowY);
        lv_anim_t pidButtonAnimation;
        lv_anim_init(&pidButtonAnimation);
        lv_anim_set_var(&pidButtonAnimation, selectorPidTunerButton);
        lv_anim_set_exec_cb(&pidButtonAnimation, setAnimatedObjectX);
        lv_anim_set_values(&pidButtonAnimation, lv_obj_get_width(autonSelectorScreen), pidTargetX);
        lv_anim_set_duration(&pidButtonAnimation, 450);
        lv_anim_set_delay(&pidButtonAnimation, 100);
        lv_anim_set_path_cb(&pidButtonAnimation, lv_anim_path_ease_out);
        lv_anim_start(&pidButtonAnimation);
    }
}

void setAnimatedObjectX(void* object, std::int32_t x) {
    lv_obj_set_x(static_cast<lv_obj_t*>(object), x);
}

void animateSelectorButtonsOut() {
    if (!selectorUiRevealed || autonSelectorScreen == nullptr) return;

    for (std::uint8_t i = 0; i < 3; ++i) {
        lv_obj_t* button = autonButtons[i];
        lv_anim_t animation;
        lv_anim_init(&animation);
        lv_anim_set_var(&animation, button);
        lv_anim_set_exec_cb(&animation, setAnimatedObjectY);
        lv_anim_set_values(&animation, 8, -34);
        lv_anim_set_duration(&animation, 450);
        lv_anim_set_delay(&animation, i * 90);
        lv_anim_set_path_cb(&animation, lv_anim_path_ease_in);
        if (i == 2) {
            lv_anim_set_completed_cb(&animation, [](lv_anim_t*) {
                createMatrixRainScreen();
            });
        }
        lv_anim_start(&animation);
    }
}

void updateMatrixRain(lv_timer_t*) {
    for (MatrixStream& stream : matrixStreams) {
        stream.y += stream.speed;
        if (stream.y >= matrixScreenHeight) {
            stream.y = -stream.height;
        }
        lv_obj_set_y(stream.label, stream.y);
        lv_obj_set_style_text_color(
            stream.label,
            rainTextColorAtY(stream.y + stream.height / 2, matrixScreenHeight),
            LV_PART_MAIN);
    }
}

void returnToAutonSelector(lv_event_t*) {
    if (autonSelectorScreen == nullptr) return;

    if (matrixRainTimer != nullptr) {
        lv_timer_pause(matrixRainTimer);
    }
    if (selectorUiRevealed) {
        for (std::uint8_t i = 0; i < 3; ++i) {
            lv_obj_set_style_opa(autonButtons[i], LV_OPA_COVER, LV_PART_MAIN);
            lv_obj_set_style_opa(autonButtonLabels[i], LV_OPA_COVER, LV_PART_MAIN);
            lv_obj_set_style_opa(poseLabels[i], LV_OPA_COVER, LV_PART_MAIN);
            lv_obj_set_pos(autonButtons[i], selectorCenteredButtonX(),
                           selectorAutoButtonY(i));
        }
        lv_obj_set_style_opa(selectorLiftHeightLabel, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_pos(selectorTempsButton, selectorSideButtonLeftX(90),
                       selectorAutoButtonY(1) + 3);
        if (selectorPidTunerButton != nullptr) {
            lv_obj_set_pos(selectorPidTunerButton, selectorSideButtonRightX(),
                           selectorAutoButtonY(1) + 3);
        }
    }
    lv_screen_load(autonSelectorScreen);
}

void returnFromMotorTemperatureScreen(lv_event_t*) {
    if (motorTemperatureReturnScreen == nullptr) return;
    if (motorTemperatureTimer != nullptr) {
        lv_timer_pause(motorTemperatureTimer);
    }
    if (matrixRainTimer != nullptr) {
        if (motorTemperatureReturnScreen == matrixRainScreen) {
            lv_timer_resume(matrixRainTimer);
        } else {
            lv_timer_pause(matrixRainTimer);
        }
    }
    lv_screen_load(motorTemperatureReturnScreen);
}

void addMotorTemperatureButton(lv_obj_t* screen, bool aboveAutonButtons) {
    constexpr std::int32_t buttonWidth = 90;
    constexpr std::int32_t buttonHeight = 28;
    lv_obj_t* button = lv_button_create(screen);
    lv_obj_set_size(button, buttonWidth, buttonHeight);
    lv_obj_set_style_bg_color(button, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(button, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(
        button, lv_color_hex(0xFFFFFF),
        static_cast<lv_style_selector_t>(LV_PART_MAIN) |
            static_cast<lv_style_selector_t>(LV_STATE_CHECKED));
    lv_obj_set_style_radius(button, 8, LV_PART_MAIN);
    lv_obj_set_style_border_width(button, 0, LV_PART_MAIN);
    if (aboveAutonButtons) {
        selectorTempsButton = button;
        lv_obj_set_pos(button, selectorSideButtonLeftX(buttonWidth),
                       selectorAutoButtonY(1) + 3);
        lv_obj_add_flag(button, LV_OBJ_FLAG_HIDDEN);
    } else {
        matrixTempsButton = button;
        lv_obj_set_pos(button, 8, 50);
    }

    lv_obj_t* label = lv_label_create(button);
    lv_label_set_text(label, "Temps");
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_center(label);
    lv_obj_add_event_cb(button, openMotorTemperatureScreen, LV_EVENT_CLICKED, nullptr);
}

void addPidTunerButton(lv_obj_t* screen) {
    if (pros::competition::is_competition_switch() ||
        pidTunerDisabledForCompetition.load()) {
        pidTunerDisabledForCompetition.store(true);
        return;
    }

    lv_obj_t* button = lv_button_create(screen);
    lv_obj_set_size(button, 76, 28);
    if (screen == autonSelectorScreen) {
        selectorPidTunerButton = button;
        lv_obj_set_pos(button, selectorSideButtonRightX(), selectorAutoButtonY(1) + 3);
        lv_obj_add_flag(button, LV_OBJ_FLAG_HIDDEN);
    } else if (screen == matrixRainScreen) {
        matrixPidTunerButton = button;
        lv_obj_align(button, LV_ALIGN_TOP_RIGHT, -102, -6);
    } else {
        lv_obj_align(button, LV_ALIGN_TOP_RIGHT, -102, -6);
    }
    lv_obj_set_style_bg_color(button, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(button, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(button, 8, LV_PART_MAIN);
    lv_obj_set_style_border_width(button, 0, LV_PART_MAIN);
    lv_obj_t* label = lv_label_create(button);
    lv_label_set_text(label, "PID");
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_center(label);
    lv_obj_add_event_cb(button, openPidTuner, LV_EVENT_CLICKED, nullptr);
}

void openMotorTemperatureScreen(lv_event_t* event) {
    lv_event_stop_bubbling(event);
    motorTemperatureReturnScreen = lv_screen_active();
    if (motorTemperatureReturnScreen == matrixRainScreen && matrixRainTimer != nullptr) {
        lv_timer_pause(matrixRainTimer);
    }

    if (motorTemperatureScreen == nullptr) {
        motorTemperatureScreen = lv_obj_create(nullptr);
        const lv_color_t black = lv_color_hex(0x000000);
        const lv_color_t white = lv_color_hex(0xFFFFFF);
        lv_obj_set_style_bg_color(motorTemperatureScreen, white, LV_PART_MAIN);
        lv_obj_set_style_bg_grad_color(motorTemperatureScreen, black, LV_PART_MAIN);
        lv_obj_set_style_bg_grad_dir(motorTemperatureScreen, LV_GRAD_DIR_VER, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(motorTemperatureScreen, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_remove_flag(motorTemperatureScreen, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t* title = lv_label_create(motorTemperatureScreen);
        lv_label_set_text(title, "MOTOR TEMPERATURES");
        lv_obj_set_style_text_font(title, &lv_font_montserrat_16, LV_PART_MAIN);
        lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 12);
        lv_obj_set_style_text_color(
            title, rainTextColorAtY(lv_obj_get_y(title) + 8, lv_obj_get_height(motorTemperatureScreen)),
            LV_PART_MAIN);

        const char* names[] = {"Lift Motor 1", "Lift Motor 2"};
        const std::int32_t centerX = lv_obj_get_width(motorTemperatureScreen) / 2;
        for (std::uint8_t i = 0; i < 2; ++i) {
            motorTemperatureLabels[i] = lv_label_create(motorTemperatureScreen);
            lv_label_set_text_fmt(motorTemperatureLabels[i], "%s: -- F", names[i]);
            lv_obj_set_style_text_font(motorTemperatureLabels[i], &lv_font_montserrat_16, LV_PART_MAIN);
            lv_obj_set_width(motorTemperatureLabels[i], 160);
            lv_obj_set_style_text_align(motorTemperatureLabels[i], LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
            lv_obj_set_pos(motorTemperatureLabels[i], centerX - 164 + i * 168, 48);
            lv_obj_set_style_text_color(
                motorTemperatureLabels[i],
                rainTextColorAtY(lv_obj_get_y(motorTemperatureLabels[i]) + 8,
                                 lv_obj_get_height(motorTemperatureScreen)),
                LV_PART_MAIN);

            motorTemperatureBars[i] = lv_bar_create(motorTemperatureScreen);
            lv_obj_set_size(motorTemperatureBars[i], 34, 112);
            lv_bar_set_orientation(motorTemperatureBars[i], LV_BAR_ORIENTATION_VERTICAL);
            lv_bar_set_range(motorTemperatureBars[i], 70, 160);
            lv_bar_set_value(motorTemperatureBars[i], 70, LV_ANIM_OFF);
            lv_obj_set_style_bg_color(motorTemperatureBars[i], lv_color_hex(0x303030), LV_PART_MAIN);
            lv_obj_set_style_bg_opa(motorTemperatureBars[i], LV_OPA_COVER, LV_PART_MAIN);
            lv_obj_set_style_bg_color(
                motorTemperatureBars[i], lv_color_hex(0x00C853), LV_PART_INDICATOR);
            lv_obj_set_style_bg_opa(motorTemperatureBars[i], LV_OPA_COVER, LV_PART_INDICATOR);
            lv_obj_set_pos(motorTemperatureBars[i], centerX - 100 + i * 166, 78);
        }

        lv_obj_t* backButton = lv_button_create(motorTemperatureScreen);
        lv_obj_set_size(backButton, 90, 28);
        lv_obj_set_style_bg_color(backButton, black, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(backButton, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_radius(backButton, 8, LV_PART_MAIN);
        lv_obj_set_style_border_width(backButton, 0, LV_PART_MAIN);
        lv_obj_align(backButton, LV_ALIGN_BOTTOM_MID, 0, -8);
        lv_obj_t* backLabel = lv_label_create(backButton);
        lv_label_set_text(backLabel, "Back");
        lv_obj_set_style_text_color(backLabel, white, LV_PART_MAIN);
        lv_obj_center(backLabel);
        lv_obj_add_event_cb(backButton, returnFromMotorTemperatureScreen, LV_EVENT_CLICKED, nullptr);
    }

    lv_screen_load(motorTemperatureScreen);
    updateMotorTemperatureLabels(nullptr);
    if (motorTemperatureTimer == nullptr) {
        motorTemperatureTimer = lv_timer_create(updateMotorTemperatureLabels, 500, nullptr);
    } else {
        lv_timer_resume(motorTemperatureTimer);
    }
}

void updateMotorTemperatureLabels(lv_timer_t*) {
    if (lv_screen_active() != motorTemperatureScreen) return;

    const double temperatures[] = {
        liftMotors.get_temperature(0), liftMotors.get_temperature(1)
    };
    const char* names[] = {"Lift Motor 1", "Lift Motor 2"};
    for (std::uint8_t i = 0; i < 2; ++i) {
        lv_obj_set_style_text_color(
            motorTemperatureLabels[i],
            rainTextColorAtY(lv_obj_get_y(motorTemperatureLabels[i]) + 8,
                             lv_obj_get_height(motorTemperatureScreen)),
            LV_PART_MAIN);
        if (std::isfinite(temperatures[i])) {
            const auto fahrenheit = static_cast<std::int32_t>(
                std::round(temperatures[i] * 9.0 / 5.0 + 32.0));
            lv_label_set_text_fmt(motorTemperatureLabels[i], "%s: %d F", names[i], fahrenheit);
            lv_bar_set_value(motorTemperatureBars[i], fahrenheit, LV_ANIM_ON);

            const lv_color_t barColor = fahrenheit < 100
                ? lv_color_hex(0x00C853)
                : (fahrenheit < 125 ? lv_color_hex(0xFFD600) : lv_color_hex(0xFF1744));
            lv_obj_set_style_bg_color(motorTemperatureBars[i], barColor, LV_PART_INDICATOR);
        } else {
            lv_label_set_text_fmt(motorTemperatureLabels[i], "%s: Read error", names[i]);
            lv_bar_set_value(motorTemperatureBars[i], 70, LV_ANIM_OFF);
        }
    }
}

void createMatrixRainScreen() {
    if (matrixRainScreen != nullptr) {
        if (matrixRainTimer != nullptr) {
            lv_timer_resume(matrixRainTimer);
        }
        lv_screen_load(matrixRainScreen);
        return;
    }

    lv_obj_t* screen = lv_obj_create(nullptr);
    matrixRainScreen = screen;
    const lv_color_t black = lv_color_hex(0x000000);
    lv_obj_set_style_bg_color(screen, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_bg_grad_color(screen, black, LV_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(screen, LV_GRAD_DIR_VER, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(screen, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(screen, 0, LV_PART_MAIN);
    lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(screen, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(screen, returnToAutonSelector, LV_EVENT_CLICKED, nullptr);

    const std::int32_t screenWidth = lv_obj_get_width(screen);
    matrixScreenHeight = lv_obj_get_height(screen);
    const std::int32_t streamSpacing = screenWidth / matrixStreamCount;
    std::uint32_t randomState = 0x007B007U;

    for (std::uint8_t i = 0; i < matrixStreamCount; ++i) {
        randomState = randomState * 1664525U + 1013904223U;
        const std::int32_t rows = 18 + static_cast<std::int32_t>((randomState >> 16) % 14);
        char digits[64]{};
        for (std::int32_t row = 0; row < rows; ++row) {
            randomState = randomState * 1664525U + 1013904223U;
            digits[row * 2] = ((randomState >> 31) & 1U) ? '1' : '0';
            digits[row * 2 + 1] = '\n';
        }

        lv_obj_t* stream = lv_label_create(screen);
        lv_obj_add_flag(stream, LV_OBJ_FLAG_EVENT_BUBBLE);
        lv_label_set_text(stream, digits);
        lv_obj_set_style_text_font(stream, &lv_font_montserrat_12, LV_PART_MAIN);
        lv_obj_set_style_text_line_space(stream, 1, LV_PART_MAIN);
        const std::int32_t streamHeight = rows * 14;
        const std::int32_t startY = -static_cast<std::int32_t>(i % 12) * (streamHeight / 12);
        lv_obj_set_pos(stream, i * streamSpacing, startY);
        lv_obj_set_style_text_color(
            stream, rainTextColorAtY(startY + streamHeight / 2, matrixScreenHeight), LV_PART_MAIN);
        matrixStreams[i] = {stream, startY, streamHeight, 2 + static_cast<std::int32_t>(i % 4)};
    }

    for (std::uint8_t i = 0; i < 3; ++i) {
        matrixPoseLabels[i] = lv_label_create(screen);
        lv_obj_set_style_text_color(matrixPoseLabels[i], lv_color_hex(0xFFFFFF), LV_PART_MAIN);
        lv_obj_set_style_text_font(matrixPoseLabels[i], &lv_font_montserrat_16, LV_PART_MAIN);
        lv_obj_set_width(matrixPoseLabels[i], 180);
        lv_obj_set_style_text_align(matrixPoseLabels[i], LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
        lv_obj_set_pos(matrixPoseLabels[i], (screenWidth - 180) / 2,
                       matrixScreenHeight / 2 + 58 + static_cast<std::int32_t>(i) * 20);
    }
    matrixLiftHeightLabel = lv_label_create(screen);
    lv_obj_set_style_text_color(matrixLiftHeightLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_text_font(matrixLiftHeightLabel, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_width(matrixLiftHeightLabel, 180);
    lv_obj_set_style_text_align(matrixLiftHeightLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_pos(matrixLiftHeightLabel, (screenWidth - 180) / 2,
                   matrixScreenHeight / 2 + 38);
    updatePoseLabels(nullptr);
    addMotorTemperatureButton(screen, false);
    addPidTunerButton(screen);

    lv_screen_load(screen);
    constexpr std::int32_t tempsTargetX = 8;
    lv_obj_set_x(matrixTempsButton, -96);
    lv_anim_t tempsAnimation;
    lv_anim_init(&tempsAnimation);
    lv_anim_set_var(&tempsAnimation, matrixTempsButton);
    lv_anim_set_exec_cb(&tempsAnimation, setAnimatedObjectX);
    lv_anim_set_values(&tempsAnimation, -96, tempsTargetX);
    lv_anim_set_duration(&tempsAnimation, 450);
    lv_anim_set_path_cb(&tempsAnimation, lv_anim_path_ease_out);
    lv_anim_start(&tempsAnimation);
    matrixRainTimer = lv_timer_create(updateMatrixRain, 30, nullptr);
}

void createAutonSelector() {
    lv_obj_t* screen = lv_screen_active();
    autonSelectorScreen = screen;
    selectorUiRevealed = false;
    const lv_color_t black = lv_color_hex(0x000000);
    const lv_color_t white = lv_color_hex(0xFFFFFF);

    lv_obj_set_style_bg_color(screen, white, LV_PART_MAIN);
    lv_obj_set_style_bg_grad_color(screen, black, LV_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(screen, LV_GRAD_DIR_VER, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_add_flag(screen, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(screen, revealSelectorUi, LV_EVENT_CLICKED, nullptr);

    const std::int32_t screenWidth = lv_obj_get_width(screen);
    selectorScreenHeight = lv_obj_get_height(screen);
    std::uint32_t randomState = 0x007B007U;
    constexpr std::int32_t selectorRainRows = 32;
    for (std::uint8_t column = 0; column < selectorRainStreamCount; ++column) {
        char digits[selectorRainRows * 2 + 1]{};
        for (std::int32_t row = 0; row < selectorRainRows; ++row) {
            randomState = randomState * 1664525U + 1013904223U;
            digits[row * 2] = ((randomState >> 31) & 1U) ? '1' : '0';
            digits[row * 2 + 1] = '\n';
        }

        const std::int32_t streamHeight = selectorRainRows * 14;
        const std::int32_t startY = -static_cast<std::int32_t>(column % 8) *
                                    (streamHeight / 8);
        lv_obj_t* streamLabel = lv_label_create(screen);
        lv_obj_add_flag(streamLabel, LV_OBJ_FLAG_EVENT_BUBBLE);
        lv_label_set_text(streamLabel, digits);
        lv_obj_set_style_text_font(streamLabel, &lv_font_montserrat_12, LV_PART_MAIN);
        lv_obj_set_style_text_line_space(streamLabel, 1, LV_PART_MAIN);
        lv_obj_set_pos(streamLabel,
                       static_cast<std::int32_t>(column) * (screenWidth - 8) /
                           (selectorRainStreamCount - 1),
                       startY);
        lv_obj_set_style_text_color(
            streamLabel, rainTextColorAtY(startY + streamHeight / 2, selectorScreenHeight),
            LV_PART_MAIN);
        selectorRainStreams[column] = {
            streamLabel, startY, streamHeight, 2 + static_cast<std::int32_t>(column % 3)
        };
    }
    lv_timer_create(updateSelectorRain, 30, nullptr);

    const char* buttonNames[] = {"Close", "Far", "Skills"};

    for (std::uint8_t i = 0; i < 3; ++i) {
        autonButtons[i] = lv_button_create(screen);
        lv_obj_add_flag(autonButtons[i], LV_OBJ_FLAG_CHECKABLE);
        lv_obj_set_size(autonButtons[i], 90, 34);
        lv_obj_set_style_bg_color(autonButtons[i], black, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(autonButtons[i], LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_bg_color(
            autonButtons[i], white,
            static_cast<lv_style_selector_t>(LV_PART_MAIN) |
                static_cast<lv_style_selector_t>(LV_STATE_CHECKED));
        lv_obj_set_style_radius(autonButtons[i], 8, LV_PART_MAIN);
        lv_obj_set_style_border_width(autonButtons[i], 0, LV_PART_MAIN);
        lv_obj_set_pos(autonButtons[i], selectorCenteredButtonX(), selectorAutoButtonY(i));
        lv_obj_add_flag(autonButtons[i], LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_event_cb(
            autonButtons[i], onAutonRoutineSelected, LV_EVENT_CLICKED, &autonRoutineOptions[i]);

        autonButtonLabels[i] = lv_label_create(autonButtons[i]);
        lv_label_set_text(autonButtonLabels[i], buttonNames[i]);
        lv_obj_set_style_text_color(autonButtonLabels[i], white, LV_PART_MAIN);
        lv_obj_center(autonButtonLabels[i]);
    }

    lv_obj_add_state(autonButtons[0], LV_STATE_CHECKED);
    lv_obj_set_style_text_color(autonButtonLabels[0], black, LV_PART_MAIN);

    for (std::uint8_t i = 0; i < 3; ++i) {
        poseLabels[i] = lv_label_create(screen);
        lv_obj_set_style_text_color(poseLabels[i], white, LV_PART_MAIN);
        lv_obj_set_style_text_font(poseLabels[i], &lv_font_montserrat_16, LV_PART_MAIN);
        lv_obj_set_width(poseLabels[i], 180);
        lv_obj_set_style_text_align(poseLabels[i], LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
        lv_obj_set_pos(poseLabels[i], (screenWidth - 180) / 2,
                       selectorScreenHeight / 2 + 58 + static_cast<std::int32_t>(i) * 20);
        lv_obj_add_flag(poseLabels[i], LV_OBJ_FLAG_HIDDEN);
    }

    selectorLiftHeightLabel = lv_label_create(screen);
    lv_obj_set_style_text_color(selectorLiftHeightLabel, white, LV_PART_MAIN);
    lv_obj_set_style_text_font(selectorLiftHeightLabel, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_width(selectorLiftHeightLabel, 180);
    lv_obj_set_style_text_align(selectorLiftHeightLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_pos(selectorLiftHeightLabel, (screenWidth - 180) / 2,
                   selectorScreenHeight / 2 + 38);
    lv_obj_add_flag(selectorLiftHeightLabel, LV_OBJ_FLAG_HIDDEN);

    updatePoseLabels(nullptr);
    lv_timer_create(updatePoseLabels, 100, nullptr);
    addMotorTemperatureButton(screen, true);
    addPidTunerButton(screen);
}

void updateSimpleAutonScreen(lv_timer_t*) {
    disablePidTunerForCompetitionSwitch();
    if (autonScreenTransitionRequested.exchange(false)) {
        createMatrixRainScreen();
    }
}

void createSimpleAutonSelector() {
    lv_obj_t* screen = lv_obj_create(nullptr);
    autonSelectorScreen = screen;
    selectorUiRevealed = false;
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    const char* buttonNames[] = {"Close", "Far", "Skills"};
    const std::int32_t buttonWidth = 120;
    const std::int32_t buttonX = (lv_obj_get_width(screen) - buttonWidth) / 2;
    constexpr std::int32_t buttonY[] = {22, 70, 118};
    const lv_color_t black = lv_color_hex(0x000000);
    const lv_color_t white = lv_color_hex(0xFFFFFF);
    const AutonRoutine selectedRoutine = selectedAutonRoutine.load();

    for (std::uint8_t i = 0; i < 3; ++i) {
        autonButtons[i] = lv_button_create(screen);
        lv_obj_add_flag(autonButtons[i], LV_OBJ_FLAG_CHECKABLE);
        lv_obj_set_size(autonButtons[i], buttonWidth, 36);
        lv_obj_set_pos(autonButtons[i], buttonX, buttonY[i]);
        lv_obj_set_style_bg_color(autonButtons[i], black, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(autonButtons[i], LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_bg_color(
            autonButtons[i], white,
            static_cast<lv_style_selector_t>(LV_PART_MAIN) |
                static_cast<lv_style_selector_t>(LV_STATE_CHECKED));
        lv_obj_set_style_radius(autonButtons[i], 6, LV_PART_MAIN);
        lv_obj_set_style_border_width(autonButtons[i], 1, LV_PART_MAIN);
        lv_obj_set_style_border_color(autonButtons[i], white, LV_PART_MAIN);
        lv_obj_add_event_cb(
            autonButtons[i], onAutonRoutineSelected, LV_EVENT_CLICKED, &autonRoutineOptions[i]);

        autonButtonLabels[i] = lv_label_create(autonButtons[i]);
        lv_label_set_text(autonButtonLabels[i], buttonNames[i]);
        lv_obj_set_style_text_color(
            autonButtonLabels[i], selectedRoutine == autonRoutineOptions[i] ? black : white,
            LV_PART_MAIN);
        lv_obj_center(autonButtonLabels[i]);
        if (selectedRoutine == autonRoutineOptions[i]) {
            lv_obj_add_state(autonButtons[i], LV_STATE_CHECKED);
        }
    }

    addPidTunerButton(screen);
    if (selectorPidTunerButton != nullptr) {
        lv_obj_set_pos(selectorPidTunerButton, (lv_obj_get_width(screen) - 76) / 2, 190);
        lv_obj_remove_flag(selectorPidTunerButton, LV_OBJ_FLAG_HIDDEN);
    }

    lv_timer_create(updateSimpleAutonScreen, 30, nullptr);
    lv_screen_load(screen);
}



void initialize() {
    // Keep one line active: use the other to switch between the current screen and the simple one.
    // createAutonSelector();
    createSimpleAutonSelector();
}

void requestAutonScreenTransition() {
    autonScreenTransitionRequested.store(true);
}

AutonRoutine getSelectedAutonRoutine() {
    return selectedAutonRoutine.load();
}

} // namespace brain_screen