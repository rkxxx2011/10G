#pragma once

// Registers the LCD left/right buttons (btn0/btn2) for cycling through the
// selectable autonomous routines below, and prints the current selection to
// the screen. Call once from competition_initialize().
void auto_selector_init();

// Runs whichever autonomous routine is currently selected. Call from
// autonomous().
void auto_selector_run();
