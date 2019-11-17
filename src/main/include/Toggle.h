/*
 * Toggle.h
 *
 *  Created on: Aug 16, 2017
 *      Author: RatPack
 * https://github.com/FRC830/Lib830/blob/f2e63950678d7579074bd25ff3be07d5b41bf80e/input/Toggle.h
 */

class Toggle {
   private:
    bool prev_state;
    bool toggle_state;

   public:
    explicit Toggle(bool toggle_state = false) : prev_state(false), toggle_state(toggle_state) {}
    bool toggle(bool button_state) {
        if (button_state && prev_state != button_state) {
            toggle_state = !toggle_state;
        }
        prev_state = button_state;
        return toggle_state;
    }

    bool operator=(const bool state) {
        toggle_state = state;
        return toggle_state;
    }

    operator bool() {
        return toggle_state;
    }
};