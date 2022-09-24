#pragma once
#include "ofMain.h"

class Gui {
public:
	Gui();
	void update(float altitude, bool altSense, bool exploded, bool bLanderLoaded);
	void draw();
	
	bool altitudeSensor;
	bool newGame;
	bool landerLoaded;

	int textWidth;
	int textHeight;
	int frameRate;
	string fps;
	string alt;
	string reset;

	glm::vec3 fpsPos;
	glm::vec3 altPos;
	glm::vec3 resetPos;
};