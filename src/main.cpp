#include "common.hpp"

Engine* new_app ();

int main () {
	Engine* app = new_app();
	int ret = app->main_loop();
	delete app;
	return ret;
}
