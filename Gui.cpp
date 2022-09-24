#include "Gui.h"
#include <string>
#include <format>

Gui::Gui() {
	textWidth = 8;
	textHeight = 14;
	altitudeSensor = false;
	float w = ofGetWindowWidth();
	float h = ofGetWindowHeight();

	frameRate = (int)ofGetFrameRate();
	fpsPos = glm::vec3(w - w / 8, textHeight, 0);
	altPos = glm::vec3(w - w / 8, textHeight * 2, 0);
}

void Gui::update(float altitude, bool altSense, bool exploded, bool bLanderLoaded) {
	altitudeSensor = altSense;
	newGame = exploded;
	landerLoaded = bLanderLoaded;
	float w = ofGetWindowWidth();
	float h = ofGetWindowHeight();

	frameRate = (int)ofGetFrameRate();
	fps = to_string(frameRate);
	fpsPos = glm::vec3(w - w / 4, textHeight, 0);

	reset = "";
	if (newGame) {
		if (!landerLoaded) {
			reset = "Load lander.obj to start";
		}
		else {
			reset = "Press Enter to Restart";
		}
	}
	resetPos = glm::vec3(w - w / 4, textHeight * 3, 0);

	alt = "";
	if (altitudeSensor) {
		if (altitude != NULL) {
			alt = to_string(altitude);
		}
		else {
			alt = "Unable to detect";
		}
		altPos = glm::vec3(w - w / 4, textHeight * 2, 0);
	}

	
}

void Gui::draw() {
	ofSetColor(255);
	ofDrawBitmapString("FPS: " + fps, fpsPos);
	if (altitudeSensor) {
		ofDrawBitmapString("Altitude: " + alt, altPos);
	}
	if (newGame) {
		ofDrawBitmapString(reset, resetPos);
	}
}