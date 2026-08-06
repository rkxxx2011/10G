#include "auto_selector.hpp"
#include "grape.hpp"
#include "pros/llemu.hpp"
#include <string>

namespace {

struct AutoOption {
    const char* name;
    void (*run)();
};

void do_nothing() {}

// Add new autonomous routines here as more get written — nothing else
// needs to change to add an entry.
const AutoOption AUTO_OPTIONS[] = {
    {"Do Nothing", do_nothing},
    {"Four Pin", fourpin},
};
constexpr int AUTO_OPTION_COUNT = sizeof(AUTO_OPTIONS) / sizeof(AUTO_OPTIONS[0]);

int selected_index = 0;

void print_selection() {
    pros::lcd::set_text(3, std::string("Auto: ") + AUTO_OPTIONS[selected_index].name);
}

void select_previous() {
    selected_index = (selected_index - 1 + AUTO_OPTION_COUNT) % AUTO_OPTION_COUNT;
    print_selection();
}

void select_next() {
    selected_index = (selected_index + 1) % AUTO_OPTION_COUNT;
    print_selection();
}

} // namespace

void auto_selector_init() {
    pros::lcd::set_text(4, "< / > (btn0/btn2) to pick auto");
    print_selection();
    pros::lcd::register_btn0_cb(select_previous);
    pros::lcd::register_btn2_cb(select_next);
}

void auto_selector_run() {
    AUTO_OPTIONS[selected_index].run();
}
